"""STAR-D Raspberry Pi flight software entry point.

Constructs the subsystems, wires them together, and services them.
All behaviour lives in the modules; this file is wiring.
"""

import threading
import time
import os

from stard.camera.camera_control import CameraControl
from stard.camera.video_writer import VideoWriter
from stard.datalog import csv_manager
from stard.links import messages, protocol
from stard.links.uart_link import UartLink
from stard.storage.storage_manager import StorageManager, LOW_SSD_FOR_FLIGHT_GB

SERIAL_PORT = "/dev/ttyAMA0"
DATA_ROOT = "/mnt/data"
LOOP_PERIOD_S = 0.01          # 100 Hz


class SoftwarePiComputer:
    def __init__(self) -> None:
        self._pi_init_complete = False
        self._stop_event = threading.Event()
        self._storage = StorageManager(DATA_ROOT)
        self._link = UartLink(SERIAL_PORT, self.make_status, self.log_sensors, self.handle_command)
        self._camera_fault = False
        self._camera: CameraControl | None = None

        # A missing camera must not take the Pi down: without the link the
        # ground has no way to learn what failed.

        try:
            self._camera = CameraControl(self._link.mission_time_ms,
                                         self._link.time_ref)
        except Exception:
            self._camera_fault = True
        
        self._writer = VideoWriter(os.path.join(DATA_ROOT, "video"))

        csv_dir = os.path.join(DATA_ROOT, "csv")
        t, tr = self._link.mission_time_ms, self._link.time_ref
        self._sensor_log = csv_manager.SensorCsvLogger(csv_dir, t, tr)
        self._state_log = csv_manager.SystemStateCsvLogger(csv_dir, t, tr)
        self._telemetry_log = csv_manager.TelemetryCsvLogger(csv_dir, t, tr)
        self._command_log = csv_manager.UplinkCommandCsvLogger(csv_dir, t, tr)

        self._capture_thread: threading.Thread | None = None

        # Cadence counters for the periodic CSV writes.
        self._loop_count = 0
        self._last_sensor: messages.Sensor | None = None    # exists because sensors and mode do not arrive at same cadence, and telemetry log needs both

        # SU is only complete if startup actually succeeded.
        self._pi_init_complete = (self._storage.self_test() and not self._camera_fault)

    # --- callbacks the link calls into ---
    def make_status(self) -> messages.Status:
        """Answer a POLL. Runs inside the 50 ms reply window: no blocking
        calls, no filesystem access. Everything read here is cached.

        command_seq_echo and command_result are overwritten by UartLink
        from its own command tracking, so void values are given
        """
        cam = self._camera_status()

        return messages.Status(
            camera_status=cam,
            ssd_free_gb=self._storage.free_gb(),
            command_seq_echo=0,
            command_result=protocol.CommandResult.RESULT_NONE,
        )

    def log_sensors(self, sensor: messages.Sensor) -> None:
        """Receive one SENSOR_DATA message. Called from the link at 10 Hz."""
        self._sensor_log.write_row(sensor)
        self._last_sensor = sensor


    def handle_command(self, cmd: messages.Command) -> protocol.CommandResult:
        """Execute a telecommand forwarded by the ESP32.

        The sequence-number check has already happened in UartLink, so a
        call here always means a new command."""
        started = time.monotonic()

        if cmd.command_flag == protocol.Command.CMD_WIPE:
            self._storage.wipe()
            ok = self._storage.free_gb() >= LOW_SSD_FOR_FLIGHT_GB
            result = (protocol.CommandResult.RESULT_OK if ok
                      else protocol.CommandResult.RESULT_FAILED)
        elif cmd.command_flag == protocol.Command.CMD_TEST:
            result = self.self_test()
        else:
            result = protocol.CommandResult.RESULT_FAILED

        duration_ms = round((time.monotonic() - started) * 1000)
        self._command_log.write_row(csv_manager.CommandRecord(
            command=cmd, result=result, duration_ms=duration_ms))
        return result
            


    # lifecycle
    def effective_mode(self) -> protocol.SoftwareMode:
        if not self._pi_init_complete:
            return protocol.SoftwareMode.MODE_SU
        return self._link.mode()


    def start(self) -> None:
        """Start the writer, the camera, and the capture thread.

        The writer starts first so the queue has a consumer before
        anything begins filling it. Shutdown reverses this order.
        """
        self._writer.start()

        if self._camera is None:
            return

        self._camera.start()
        self._capture_thread = threading.Thread(
            target=self._camera.run_capture_loop,
            args=(self._writer, self._stop_event),
            name="capture",
        )
        self._capture_thread.start()


    def run(self) -> None:
        """Main loop. Services the link and drives the periodic logging.

        Link first: CSV rows written later in the same iteration then
        carry values from this cycle rather than the previous one.
        """
        next_tick = time.monotonic()

        while True:
            self._link.service()

            if self._camera is not None:
                self._camera.set_mode(self.effective_mode())

            # 10 Hz: system state
            if self._loop_count % 10 == 0:
                self._state_log.write_row(self._system_state())

            # 1 Hz: refresh cached free space, mirror the downlink packet
            if self._loop_count % 100 == 0:
                self._storage.refresh()
                if self._last_sensor is not None:
                    self._telemetry_log.write_row(
                        (self._system_state(), self._last_sensor))

            self._loop_count += 1

            next_tick += LOOP_PERIOD_S
            time.sleep(max(0.0, next_tick - time.monotonic()))


    
    def shutdown(self) -> None:
        """Stop everything in reverse order of startup.

        Producers before consumers: capture must stop before the writer,
        or frames continue arriving at a queue nobody is draining.
        """
        self._stop_event.set()
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=5.0)
            self._capture_thread = None

        if self._camera is not None:
            self._camera.stop()

        self._writer.stop()

        for log in (self._sensor_log, self._state_log,
                    self._telemetry_log, self._command_log):
            log.close()


    def _system_state(self) -> csv_manager.SystemState:
        """Snapshot of the current picture, for the 10 Hz state log."""
        return csv_manager.SystemState(
            mode=self.effective_mode(),
            degraded=self._link.degraded(),
            camera_status=self._camera_status(),
            ssd_free_gb=self._storage.free_gb(),
            status_error_flag=0,          # TODO: merge ESP32 and Pi-side bits
            link_stale=self._link.link_stale(),
            ms_since_last_poll=self._link.ms_since_last_poll(),
        )


    def _camera_status(self) -> protocol.CameraStatus:
        """Ordered by severity: no camera, then write failure, then the
        camera's own view of itself."""
        if self._camera is None:
            return protocol.CameraStatus.CAM_UNKNOWN
        if self._writer.write_failed():
            return protocol.CameraStatus.CAM_WRITE_FAIL
        return self._camera.status()


    def self_test(self) -> protocol.CommandResult:
        """Exercise each subsystem briefly. Called at startup and by CMD_TEST.

        Deliberately touches the real code paths rather than inspecting
        state: a flag can be stale, a write cannot.
        """
        # Storage: mounted, writable, enough room for the experiment.
        if not self._storage.self_test():
            return protocol.CommandResult.RESULT_FAILED

        # Camera: present, and actually producing frames. Frames are
        # discarded, not submitted, so a test does not write video files.
        if self._camera is None:
            return protocol.CommandResult.RESULT_FAILED
        was_running = self._camera.is_running()
        if not was_running:
            self._camera.start()
        try:
            for _ in range(3):
                if self._camera.capture() is None:
                    return protocol.CommandResult.RESULT_FAILED
        except Exception:
            return protocol.CommandResult.RESULT_FAILED
        finally:
            if not was_running:
                self._camera.stop()

        # Logging: a row must reach disk and the file must grow.
        try:
            self._state_log.write_row(self._system_state())
        except Exception:
            return protocol.CommandResult.RESULT_FAILED

        return protocol.CommandResult.RESULT_OK


def main() -> None:
    computer = SoftwarePiComputer()
    computer.start()
    try:
        computer.run()
    except KeyboardInterrupt:
        pass
    finally:
        computer.shutdown()


if __name__ == "__main__":
    main()