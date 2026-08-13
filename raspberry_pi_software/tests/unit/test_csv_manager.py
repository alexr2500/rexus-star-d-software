import pytest
import csv
from stard.datalog.csv_manager import(
    SensorCsvLogger,
    SystemState,
    CommandRecord,
    RollingCsvLogger,
    SystemStateCsvLogger,
    TelemetryCsvLogger,
    UplinkCommandCsvLogger,
    ROWS_10HZ,
    ROWS_1HZ,
    OFFSET_SYSTEM
)
from stard.links import messages, protocol

class FakeClock:
    """Callable stand-in for the mission clock, advanceable from a test."""

    def __init__(self, start_ms: int = 0) -> None:
        self.t = start_ms

    def __call__(self) -> int:
        return self.t


def a_sensor() -> messages.Sensor:
    return messages.Sensor(*([1] * 16))


def test_rollover_creates_a_new_file_at_max_rows(tmp_path):
    clock = FakeClock()
    logger = SensorCsvLogger(str(tmp_path), clock,
                             lambda: protocol.TimeRef.TIME_MISSION)

    # Exactly the limit: one full file, not yet rolled.
    for i in range(ROWS_10HZ):
        clock.t = i * 100
        logger.write_row(a_sensor())
    assert len(list(tmp_path.iterdir())) == 1

    # The next row finds the file full and opens a second one.
    clock.t = ROWS_10HZ * 100
    logger.write_row(a_sensor())
    logger.close()

    files = sorted(p.name for p in tmp_path.iterdir())
    assert len(files) == 2
    assert files[0].startswith("sensors_TPLUS_")


def test_first_file_is_shortened_by_the_phase_offset(tmp_path):
    """System state is offset so it never rolls on the same tick as sensors."""
    clock = FakeClock()
    logger = SystemStateCsvLogger(str(tmp_path), clock,
                                  lambda: protocol.TimeRef.TIME_MISSION)

    state = SystemState(
        mode=protocol.SoftwareMode.MODE_NE, degraded=False,
        camera_status=protocol.CameraStatus.CAM_OK, ssd_free_gb=97,
        status_error_flag=0, link_stale=False, ms_since_last_poll=120)

    for i in range(ROWS_10HZ - OFFSET_SYSTEM):
        clock.t = i * 100
        logger.write_row(state)
    logger.close()

    files = list(tmp_path.iterdir())
    assert len(files) == 1

    with open(files[0], newline="") as f:
        rows = list(csv.reader(f))
    assert len(rows) - 1 == ROWS_10HZ - OFFSET_SYSTEM     # minus the header