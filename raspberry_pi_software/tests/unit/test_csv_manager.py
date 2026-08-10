import pytest
from stard.datalog.csv_manager import(
    SensorCsvLogger,
    SystemState,
    CommandRecord,
    RollingCsvLogger,
    SystemStateCsvLogger,
    TelemetryCsvLogger,
    UplinkCommandCsvLogger
)
from stard.links import protocol

def test_rollover_at_max_rows(tmp_path):
    logger = SensorCsvLogger(str(tmp_path), lambda: 0,
                             lambda: protocol.TimeRef.TIME_MISSION)
    
    assert len(list(tmp_path.iterdir())) == 2