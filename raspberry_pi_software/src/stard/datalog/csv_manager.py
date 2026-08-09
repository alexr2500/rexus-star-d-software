"""CSV logging for all non-video data streams.

One rolling-file base class, plus one subclass per stream in the CONOPS
data table. Files roll over on row count rather than elapsed wall time,
because mission time re-anchors at SODS and LO and a time-based trigger
would fire spuriously across those jumps. Row counts below are the CONOPS
rotation period multiplied by the stream rate.

Streams carry different phase offsets so their rollovers - each of which
costs an fsync - never land on the same tick. Video is not handled here:
it runs on its own thread with its own writer.
"""

import csv
import os
from dataclasses import dataclass
from typing import Callable, Optional

from stard.links import messages, protocol
from typing import Any, Callable, IO, Optional

# --- Rotation sizing ----------------------------------------------------
# 20 s rotation for every CSV stream, in both SOE states. Short files bound
# the loss from a corrupted or truncated write, and CSVs are small enough
# that the resulting file count never becomes a problem.

ROTATION_S = 20

ROWS_10HZ = ROTATION_S * 10      # 200
ROWS_1HZ = ROTATION_S * 1        # 20

# Phase offsets spread the three periodic streams so they fsync at 0 s,
# 7 s and 14 s within each 20 s cycle instead of all at once.
OFFSET_SENSOR = 0
OFFSET_SYSTEM = 70               # 7 s at 10 Hz
OFFSET_TELEMETRY = 14            # 14 s at 1 Hz

# flush() only reaches the OS page cache, which is lost on power removal.
# fsync forces data to the device but costs milliseconds each time.
# TODO: measure on the flight SSD and tune.
FSYNC_EVERY_N_ROWS = 10

_TIME_REF_TAG = {
    protocol.TimeRef.TIME_UNSYNCED: "UNSYNCED",
    protocol.TimeRef.TIME_COUNTDOWN: "TMINUS",
    protocol.TimeRef.TIME_MISSION: "TPLUS",
}


# --- Records passed to the loggers --------------------------------------

@dataclass
class SystemState:
    """Snapshot of Pi-side and link state at one instant."""
    mode: protocol.SoftwareMode
    degraded: bool
    camera_status: protocol.CameraStatus
    ssd_free_gb: int
    status_error_flag: int
    link_stale: bool
    ms_since_last_poll: Optional[int]


@dataclass
class CommandRecord:
    """A telecommand and how it turned out."""
    command: messages.Command
    result: protocol.CommandResult
    duration_ms: int


# --- Base class ---------------------------------------------------------

class RollingCsvLogger:
    """Owns file handling, rollover, headers and fsync policy.

    Subclasses supply a filename prefix, a header row, and the conversion
    from an input object to a list of cell values.
    """

    def __init__(
        self,
        directory: str,
        time_source: Callable[[], int],
        time_ref_source: Callable[[], protocol.TimeRef],
        max_rows: Optional[int] = None,
        phase_offset_rows: int = 0,
    ) -> None:
        self._directory = directory
        self._time_source = time_source
        self._time_ref_source = time_ref_source
        self._max_rows = max_rows              # None -> never roll over
        self._phase_offset_rows = phase_offset_rows

        self._file: IO[str] | None = None
        self._writer: Any = None
        self._row_count = 0
        self._rows_since_fsync = 0
        self._first_file = True

        os.makedirs(directory, exist_ok=True)

    # --- Subclass responsibilities -------------------------------------

    def _prefix(self) -> str:
        raise NotImplementedError

    def _header(self) -> list[str]:
        raise NotImplementedError

    def _row(self, item) -> list:
        raise NotImplementedError

    # --- File handling --------------------------------------------------

    def _filename(self) -> str:
        """Encode the time reference so a post-flight reader can tell a
        scrubbed countdown from the real flight at a glance."""
        tag = _TIME_REF_TAG[self._time_ref_source()]
        seconds = abs(self._time_source()) // 1000
        return os.path.join(
            self._directory, f"{self._prefix()}_{tag}_{seconds:07d}.csv"
        )

    def _open_new_file(self) -> None:
        self.close()
        self._file = open(self._filename(), "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self._header())
        self._row_count = 0
        self._rows_since_fsync = 0

    def _rows_before_rollover(self) -> Optional[int]:
        """The first file is shortened by the phase offset; every file
        after it is full length but permanently shifted in phase."""
        if self._max_rows is None:
            return None
        if self._first_file:
            return max(1, self._max_rows - self._phase_offset_rows)
        return self._max_rows

    def _should_roll(self) -> bool:
        if self._file is None:
            return True
        limit = self._rows_before_rollover()
        return limit is not None and self._row_count >= limit

    # --- Public API -------------------------------------------------------

    def write_row(self, item) -> None:
        if self._should_roll():
            if self._file is not None:
                self._first_file = False
            self._open_new_file()

        self._writer.writerow(self._row(item))
        self._row_count += 1
        self._rows_since_fsync += 1

        if self._rows_since_fsync >= FSYNC_EVERY_N_ROWS:
            self._sync()

    def _sync(self) -> None:
        if self._file is not None:
            self._file.flush()
            os.fsync(self._file.fileno())
            self._rows_since_fsync = 0

    def close(self) -> None:
        """Flush to the physical device and close. Safe to call twice."""
        if self._file is not None:
            self._sync()
            self._file.close()
            self._file = None
            self._writer = None


# --- Sensor metadata, 10 Hz ---------------------------------------------

class SensorCsvLogger(RollingCsvLogger):
    """Raw sensor words exactly as received over the UART link.

    Values are stored raw rather than converted: raw is lossless and
    reproducible, and a scale factor that later proves wrong can be
    reapplied in analysis. Scale factors live in protocol.py.
    """

    def __init__(self, directory, time_source, time_ref_source):
        super().__init__(directory, time_source, time_ref_source,
                         max_rows=ROWS_10HZ, phase_offset_rows=OFFSET_SENSOR)

    def _prefix(self) -> str:
        return "sensors"

    def _header(self) -> list[str]:
        return [
            "mission_time_ms", "time_ref",
            "ext_bme_temp_raw", "ext_bme_pressure_raw", "ext_bme_humidity_raw",
            "int_bme_temp_raw", "int_bme_pressure_raw", "int_bme_humidity_raw",
            "abp_pressure_raw", "slf3s_flow_raw", "pt100_temp_raw",
            "imu_accel_x", "imu_accel_y", "imu_accel_z",
            "imu_gyro_x", "imu_gyro_y", "imu_gyro_z",
        ]

    def _row(self, s: messages.Sensor) -> list:
        return [
            self._time_source(), int(self._time_ref_source()),
            s.ext_bme_temp_raw, s.ext_bme_pressure_raw, s.ext_bme_humidity_raw,
            s.int_bme_temp_raw, s.int_bme_pressure_raw, s.int_bme_humidity_raw,
            s.abp_pressure_raw, s.slf3s_flow_raw, s.pt100_temp_raw,
            s.imu_accel_x, s.imu_accel_y, s.imu_accel_z,
            s.imu_gyro_x, s.imu_gyro_y, s.imu_gyro_z,
        ]


# --- System state metadata, 10 Hz ---------------------------------------

class SystemStateCsvLogger(RollingCsvLogger):
    """Mode, error flags and storage state.

    The error flag is written as raw hex and also expanded into named
    columns, so the file is readable without a decoder while remaining
    compact and future-proof if bits are added.
    """

    ERROR_BITS = [
        ("err_pt100", 0), ("err_int_bme", 1), ("err_ext_bme", 2),
        ("err_slf3s", 3), ("err_abp", 4), ("err_lsm6dsm", 5),
        ("err_pi_degraded", 6), ("err_cam_status", 7),
        ("err_low_ssd", 8), ("err_low_ssd_for_flight", 9),
        ("err_thermal", 10), ("err_heater", 11), ("err_fluidic", 12),
        ("err_tcu_uart_link", 13), ("err_rxsm_link", 14),
        ("err_unexpected_signal", 15), ("err_mode_transition", 16),
        ("err_watchdog_reset", 17), ("err_csv_write", 18),
        ("err_config_load", 19),
    ]

    def __init__(self, directory, time_source, time_ref_source):
        super().__init__(directory, time_source, time_ref_source,
                         max_rows=ROWS_10HZ, phase_offset_rows=OFFSET_SYSTEM)

    def _prefix(self) -> str:
        return "system_state"

    def _header(self) -> list[str]:
        return [
            "mission_time_ms", "time_ref", "mode", "degraded",
            "camera_status", "ssd_free_gb",
            "link_stale", "ms_since_last_poll",
            "status_error_flag_hex",
        ] + [name for name, _ in self.ERROR_BITS]

    def _row(self, state: SystemState) -> list:
        flag = state.status_error_flag
        stale_ms = state.ms_since_last_poll
        return [
            self._time_source(), int(self._time_ref_source()),
            int(state.mode), int(state.degraded),
            int(state.camera_status), state.ssd_free_gb,
            int(state.link_stale),
            stale_ms if stale_ms is not None else "",
            f"0x{flag:08X}",
        ] + [(flag >> bit) & 1 for _, bit in self.ERROR_BITS]


# --- Telemetry mirror, 1 Hz ---------------------------------------------

class TelemetryCsvLogger(RollingCsvLogger):
    """Onboard reconstruction of the downlink packet.

    The Pi never sees the 32-byte packet sent to the TCU, so this mirrors
    the same fields at the same 1 Hz cadence. Its value is post-flight:
    comparing it against what ground received separates downlink losses
    from onboard faults.
    """

    def __init__(self, directory, time_source, time_ref_source):
        super().__init__(directory, time_source, time_ref_source,
                         max_rows=ROWS_1HZ, phase_offset_rows=OFFSET_TELEMETRY)

    def _prefix(self) -> str:
        return "telemetry_mirror"

    def _header(self) -> list[str]:
        return [
            "mission_time_ms", "time_ref", "mode", "degraded",
            "status_error_flag_hex", "camera_status", "ssd_free_gb",
            "ext_bme_temp_raw", "int_bme_temp_raw", "pt100_temp_raw",
            "int_bme_pressure_raw", "slf3s_flow_raw",
            "imu_accel_x", "imu_accel_y", "imu_accel_z",
        ]

    def _row(self, item: tuple[SystemState, messages.Sensor]) -> list:
        state, s = item
        return [
            self._time_source(), int(self._time_ref_source()),
            int(state.mode), int(state.degraded),
            f"0x{state.status_error_flag:08X}",
            int(state.camera_status), state.ssd_free_gb,
            s.ext_bme_temp_raw, s.int_bme_temp_raw, s.pt100_temp_raw,
            s.int_bme_pressure_raw, s.slf3s_flow_raw,
            s.imu_accel_x, s.imu_accel_y, s.imu_accel_z,
        ]


# --- Uplink commands, event-triggered -----------------------------------

class UplinkCommandCsvLogger(RollingCsvLogger):
    """Telecommands forwarded by the ESP32, and their outcome.

    Never rolls over: a whole campaign produces a handful of rows, and one
    continuous file is easier to audit than several.
    """

    def __init__(self, directory, time_source, time_ref_source):
        super().__init__(directory, time_source, time_ref_source,
                         max_rows=None)

    def _prefix(self) -> str:
        return "uplink_commands"

    def _header(self) -> list[str]:
        return [
            "mission_time_ms", "time_ref",
            "command_id", "command_name", "seq_number",
            "result", "result_name", "duration_ms",
        ]

    def _row(self, item: CommandRecord) -> list:
        return [
            self._time_source(), int(self._time_ref_source()),
            int(item.command.command_flag),
            protocol.Command(item.command.command_flag).name,
            item.command.seq_number,
            int(item.result), protocol.CommandResult(item.result).name,
            item.duration_ms,
        ]