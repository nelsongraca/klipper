# ADXL345 Probe
#
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import chelper
from mcu import MCU_trsync, TRSYNC_TIMEOUT, TRSYNC_SINGLE_MCU_TIMEOUT, MCU_endstop

from . import bulk_sensor as bulk, probe as kprobe


class Adxl345ProbeBulkDataQueue(bulk.BulkDataQueue):
    def __init__(self, mcu, oid, notify_data):
        super().__init__(mcu, oid=oid)
        self.notify_data = notify_data
        # by default, we are not homing
        self.homing = False

    def _handle_data(self, params):
        if self.homing:
            self.notify_data(params)
        else:
            super()._handle_data(params)


class ADXL345Probe_endstop(MCU_endstop):
    def __init__(self, adxl345):
        self.adxl345 = adxl345
        self.mcu = self.adxl345.mcu
        self._reactor = self.mcu.get_printer().get_reactor()
        self._oid = self.mcu.create_oid()
        self._trigger_completion = None
        self._rest_ticks = 0
        ffi_main, ffi_lib = chelper.get_ffi()
        self._trdispatch = ffi_main.gc(ffi_lib.trdispatch_alloc(), ffi_lib.free)
        self._trsyncs = [MCU_trsync(self.mcu, self._trdispatch)]

        # replace bulk_queue with our own
        self.bulk_queue = self.adxl345.bulk_queue = Adxl345ProbeBulkDataQueue(self.adxl345.mcu, self.adxl345.oid, self.process_accel_data)

        self._triggered = False

        # todo dev
        self.start_time = 0

    def _build_config(self):
        pass

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        # todo dev
        self.start_time = self.mcu.get_printer().get_reactor().monotonic()

        clock = self.mcu.print_time_to_clock(print_time)
        rest_ticks = self.mcu.print_time_to_clock(print_time + rest_time) - clock
        self._rest_ticks = rest_ticks
        self._trigger_completion = self._reactor.completion()
        expire_timeout = TRSYNC_TIMEOUT
        if len(self._trsyncs) == 1:
            expire_timeout = TRSYNC_SINGLE_MCU_TIMEOUT
        for i, trsync in enumerate(self._trsyncs):
            report_offset = float(i) / len(self._trsyncs)
            trsync.start(print_time, report_offset,
                         self._trigger_completion, expire_timeout)
        etrsync = self._trsyncs[0]
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_start(self._trdispatch, etrsync.REASON_HOST_REQUEST)

        # start our ADXL
        # Set bulk_queue to homing mode
        self.bulk_queue.homing = True
        # just call super _start_measurements() to setup ADXL, we will read directly in our Adxl345ProbeBulkDataQueue.
        self.adxl345._start_measurements()

        return self._trigger_completion

    def home_wait(self, home_end_time):
        # stop our ADXL
        self.adxl345._finish_measurements()
        # Set bulk_queue back to regular mode
        self.bulk_queue.homing = False

        etrsync = self._trsyncs[0]
        etrsync.set_home_end_time(home_end_time)
        if self.mcu.is_fileoutput():
            self._trigger_completion.complete(True)
        self._trigger_completion.wait()
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_stop(self._trdispatch)
        res = [trsync.stop() for trsync in self._trsyncs]
        if any([r == etrsync.REASON_COMMS_TIMEOUT for r in res]):
            return -1.

        # Since we force it to stop from the host instead of coming from the MCU we need this hack to ensure it doesn't fail
        if self._triggered and res[0] == etrsync.REASON_HOST_REQUEST:
            self._triggered = False
            return home_end_time

        if res[0] != etrsync.REASON_ENDSTOP_HIT:
            return 0.
        if self.mcu.is_fileoutput():
            return home_end_time
        return home_end_time

    def query_endstop(self, print_time):
        return 0
        # todo improve this? how?

    def trigger(self):
        self._triggered = True
        self._trigger_completion.complete(True)

    def process_accel_data(self, params):
        # for development, if we are homing for over 8seconds trigger endstop
        current = self._reactor.monotonic()
        if current - self.start_time > 8:
            self.trigger()


# ADXL345 "endstop" wrapper
class ADXL345Probe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.position_endstop = config.getfloat('z_offset')
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')

        self.adxl345 = self.printer.load_object(config, config.get('accel_chip'))

        # Create our own virtual endstop
        self.probe_endstop = ADXL345Probe_endstop(self.adxl345)

        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)
        # multi probes state
        self.multi = 'OFF'

        # wrappers
        self.get_mcu = self.probe_endstop.get_mcu
        self.add_stepper = self.probe_endstop.add_stepper
        self.get_steppers = self.probe_endstop.get_steppers
        self.home_start = self.probe_endstop.home_start
        self.home_wait = self.probe_endstop.home_wait
        self.query_endstop = self.probe_endstop.query_endstop

    def _handle_mcu_identify(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)

    def raise_probe(self):
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe activate_gcode script")

    def lower_probe(self):
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe deactivate_gcode script")

    def multi_probe_begin(self):
        self.multi = 'FIRST'

    def multi_probe_end(self):
        self.raise_probe()
        self.multi = 'OFF'

    def probe_prepare(self, hmove):
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self.lower_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'

    def probe_finish(self, hmove):
        if self.multi == 'OFF':
            self.raise_probe()

    def get_position_endstop(self):
        return self.position_endstop


def load_config(config):
    # adxl345 = CustomADXL345(config)
    adxl_probe = ADXL345Probe(config)
    # config.get_printer().add_object('adxl345', adxl345)
    config.get_printer().add_object('probe', kprobe.PrinterProbe(config, adxl_probe))
    return adxl_probe
