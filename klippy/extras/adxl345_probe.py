# ADXL345 Probe
#
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

import chelper
from mcu import MCU_trsync, TRSYNC_TIMEOUT, TRSYNC_SINGLE_MCU_TIMEOUT

from adxl345 import ADXL345
from bulk_sensor import BulkDataQueue
from probe import PrinterProbe


class Adxl345ProbeBulkDataQueue(BulkDataQueue):
    def __init__(self, mcu, oid, endstop):
        super().__init__(mcu, oid)
        # by default, we are not homing
        self.homing = False
        self.endstop = endstop

    def _handle_data(self, params):
        if self.homing:
            # this triggers it
            # self.endstop.trigger(0)
            pass
        else:
            with self.lock:
                self.raw_samples.append(params)


class CustomADXL345(ADXL345):
    def __init__(self, config,probe):
        super().__init__(config)
        # replace bulk_queue with our own
        self.bulk_queue = Adxl345ProbeBulkDataQueue(self.mcu, self.oid, probe.probe_endstop)

    def start(self):
        # just call super _start_measurements() to setup ADXL, we will read directly in our Adxl345ProbeBulkDataQueue.
        super()._start_measurements()

    def stop(self):
        super()._finish_measurements()


class ADXL345Probe_endstop:
    def __init__(self, adxl345):
        self.adxl345 = adxl345
        self.mcu = adxl345.mcu
        self._reactor = self.mcu.get_printer().get_reactor()
        self._oid = self.mcu.create_oid()
        self.mcu.register_config_callback(self._build_config)
        self._trigger_completion = None
        self._rest_ticks = 0
        ffi_main, ffi_lib = chelper.get_ffi()
        self._trdispatch = ffi_main.gc(ffi_lib.trdispatch_alloc(), ffi_lib.free)
        self._trsyncs = [MCU_trsync(self.mcu, self._trdispatch)]

        self._triggered = False

    def get_mcu(self):
        return self.mcu

    def add_stepper(self, stepper):
        logging.info("adxlprobe - add_stepper")
        trsyncs = {trsync.get_mcu(): trsync for trsync in self._trsyncs}
        trsync = trsyncs.get(stepper.get_mcu())
        if trsync is None:
            trsync = MCU_trsync(stepper.get_mcu(), self._trdispatch)
            self._trsyncs.append(trsync)
        trsync.add_stepper(stepper)
        # Check for unsupported multi-mcu shared stepper rails
        sname = stepper.get_name()
        if sname.startswith('stepper_'):
            for ot in self._trsyncs:
                for s in ot.get_steppers():
                    if ot is not trsync and s.get_name().startswith(sname[:9]):
                        cerror = self.mcu.get_printer().config_error
                        raise cerror("Multi-mcu homing not supported on"
                                     " multi-mcu shared axis")

    def get_steppers(self):
        return [s for trsync in self._trsyncs for s in trsync.get_steppers()]

    def _build_config(self):
        pass

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        logging.info(f"adxlprobe - home_start print_time: {print_time} sample_time: {sample_time} sample_count: {sample_count} rest_time: {rest_time} triggered: {triggered}")
        logging.info(f"adxlprobe - home_start time: {self._reactor.monotonic()}")

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
        self.adxl345.start()

        self._reactor.register_timer(self.trigger, self._reactor.monotonic() + 5.0)

        return self._trigger_completion

    def home_wait(self, home_end_time):
        logging.info(f"adxlprobe - home_wait home_end_time: {home_end_time}")
        logging.info(f"adxlprobe - home_wait time: {self._reactor.monotonic()}")

        self.adxl345.stop()

        etrsync = self._trsyncs[0]
        etrsync.set_home_end_time(home_end_time)
        if self.mcu.is_fileoutput():
            self._trigger_completion.complete(True)
        logging.info(f"adxlprobe - will wait")
        self._trigger_completion.wait()
        logging.info(f"adxlprobe - done waiting")
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_stop(self._trdispatch)
        logging.info(f"adxlprobe - before trsync.stop()")
        res = [trsync.stop() for trsync in self._trsyncs]
        logging.info(f"adxlprobe - after trsync.stop() res: {res}")
        if any([r == etrsync.REASON_COMMS_TIMEOUT for r in res]):
            return -1.

        # Since we force it to stop from the host instead of coming from the MCU we need this hack to ensure it doesn't fail
        if self._triggered:
            self._triggered = False
            return home_end_time

        if res[0] != etrsync.REASON_ENDSTOP_HIT:
            return 0.
        if self.mcu.is_fileoutput():
            return home_end_time
        return home_end_time

    def query_endstop(self, print_time):
        logging.info(f"adxlprobe - query_endstop print_time: {print_time}")
        return 0
        # todo improve this? how?

    def trigger(self, eventtime):
        logging.info(f"adxlprobe - _hammer_trigger eventtime: {eventtime}")
        logging.info(f"adxlprobe - _hammer_trigger time: {self._reactor.monotonic()}")
        self._triggered = True
        self._trigger_completion.complete(True)
        return self._reactor.NEVER


# ADXL345 "endstop" wrapper
class ADXL345Probe:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.position_endstop = config.getfloat('z_offset')
        self.stow_on_each_sample = config.getboolean(
            'deactivate_on_each_sample', True)
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')

        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)
        # multi probes state
        self.multi = 'OFF'

        self.adxl345 = CustomADXL345(config,self)
        self.printer.add_object('adxl345', self.adxl345)
        # Create our own virtual endstop
        self.probe_endstop = ADXL345Probe_endstop(self.adxl345)

        # wrappers
        self.get_mcu = self.probe_endstop.get_mcu
        self.add_stepper = self.probe_endstop.add_stepper
        self.get_steppers = self.probe_endstop.get_steppers
        self.home_start = self.probe_endstop.home_start
        self.home_wait = self.probe_endstop.home_wait
        self.query_endstop = self.probe_endstop.query_endstop

    def _handle_mcu_identify(self):
        logging.info(f"adxlprobe - _handle_mcu_identify")
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)

    def raise_probe(self):
        logging.info(f"adxlprobe - raise_probe")
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe activate_gcode script")

    def lower_probe(self):
        logging.info(f"adxlprobe - lower_probe")
        toolhead = self.printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self.printer.command_error(
                "Toolhead moved during probe deactivate_gcode script")

    def multi_probe_begin(self):
        logging.info(f"adxlprobe - multi_probe_begin")
        if self.stow_on_each_sample:
            return
        self.multi = 'FIRST'

    def multi_probe_end(self):
        logging.info(f"adxlprobe - multi_probe_end")
        if self.stow_on_each_sample:
            return
        self.raise_probe()
        self.multi = 'OFF'

    def probe_prepare(self, hmove):
        logging.info(f"adxlprobe - probe_prepare hmove: {hmove}")
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self.lower_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'

    def probe_finish(self, hmove):
        logging.info(f"adxlprobe - probe_finish hmove: {hmove}")
        if self.multi == 'OFF':
            self.raise_probe()

    def get_position_endstop(self):
        logging.info(f"adxlprobe - get_position_endstop")
        return self.position_endstop


def load_config(config):
    adxl_probe = ADXL345Probe(config)
    config.get_printer().add_object('probe', PrinterProbe(config, adxl_probe))
    return adxl_probe
