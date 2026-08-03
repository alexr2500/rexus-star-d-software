import struct
from dataclasses import dataclass

from stard.links import protocol


@dataclass
class Poll:
    """Contents of a POLL message (ESP32 -> Pi)."""
    mode: int
    degraded: bool
    mission_time_ms: int


POLL_FORMAT = "<BBi"


def encode_poll(p: Poll) -> bytes:
    return struct.pack(POLL_FORMAT, p.mode, int(p.degraded), p.mission_time_ms)


def decode_poll(payload: bytes) -> Poll:
    if len(payload) != protocol.LEN_POLL:
        raise ValueError(f"POLL payload must be {protocol.LEN_POLL} bytes, got {len(payload)}")
    mode, degraded, mission_time = struct.unpack(POLL_FORMAT, payload)
    return Poll(mode=mode, degraded=bool(degraded), mission_time_ms=mission_time)


@dataclass
class Command:
    """Contents of a COMMAND message (ESP32 -> Pi)."""
    command_flag: int
    seq_number: int


COMMAND_FORMAT = "<BB"


def encode_command(c: Command) -> bytes:
    return struct.pack(COMMAND_FORMAT, c.command_flag, c.seq_number)


def decode_command(payload: bytes) -> Command:
    if len(payload) != protocol.LEN_COMMAND:
        raise ValueError(f"COMMAND payload must be {protocol.LEN_COMMAND} bytes, got {len(payload)}")
    command_flag, seq_number = struct.unpack(COMMAND_FORMAT, payload)
    return Command(command_flag=command_flag, seq_number=seq_number)


@dataclass
class Status:
    """Contents of a STATUS message (Pi -> ESP32)"""
    camera_status: int
    ssd_free_gb: int
    command_seq_echo: int
    command_result: int


STATUS_FORMAT = "<BBBB"


def encode_status(s: Status) -> bytes:
    return struct.pack(STATUS_FORMAT, s.camera_status, s.ssd_free_gb, s.command_seq_echo, s.command_result)


def decode_status(payload: bytes) -> Status:
    if len(payload) != protocol.LEN_STATUS:
        raise ValueError(f"STATUS payload must be {protocol.LEN_STATUS} bytes, got {len(payload)}")
    camera_status, ssd_free_gb, command_seq_echo, command_result = struct.unpack(STATUS_FORMAT, payload)
    return Status(camera_status=protocol.CameraStatus(camera_status), ssd_free_gb=ssd_free_gb, command_seq_echo=command_seq_echo, command_result=command_result)


@dataclass
class Sensor:
    """Contents of a SENSOR_DATA message (ESP32 -> Pi)"""
    ext_bme_temp_c: int
    ext_bme_pressure_pa: int
    ext_bme_humidity: int
    int_bme_temp_c: int
    int_bme_pressure_pa: int
    int_bme_humidity: int
    abp_pressure: int
    slf3s_flow_ml: int
    pt100_temp_c: int
    imu_accel_x: int
    imu_accel_y: int
    imu_accel_z: int
    imu_gyro_x: int
    imu_gyro_y: int
    imu_gyro_z: int
    status_error_flag: int


SENSOR_DATA_FORMAT = "<hHHhHHHhhhhhhhhI"


def encode_sensor(se: Sensor) -> bytes:
    return struct.pack(SENSOR_DATA_FORMAT,
        se.ext_bme_temp_c,
        se.ext_bme_pressure_pa,
        se.ext_bme_humidity,
        se.int_bme_temp_c,
        se.int_bme_pressure_pa,
        se.int_bme_humidity,
        se.abp_pressure,
        se.slf3s_flow_ml,
        se.pt100_temp_c,
        se.imu_accel_x,
        se.imu_accel_y,
        se.imu_accel_z,
        se.imu_gyro_x,
        se.imu_gyro_y,
        se.imu_gyro_z,
        se.status_error_flag)

def decode_sensor(payload:bytes) -> Sensor:
    if len(payload) != protocol.LEN_SENSOR_DATA:
        raise ValueError(f"SENSOR_DATA payload must be {protocol.LEN_SENSOR_DATA}, got {len(payload)}")
    (ext_bme_temp_c, ext_bme_pressure_pa, ext_bme_humidity,
    int_bme_temp_c, int_bme_pressure_pa, int_bme_humidity,
    abp_pressure, slf3s_flow_ml, pt100_temp_c,
    imu_accel_x, imu_accel_y, imu_accel_z,
    imu_gyro_x, imu_gyro_y, imu_gyro_z,
    status_error_flag) = struct.unpack(SENSOR_DATA_FORMAT, payload)
    return Sensor(
        ext_bme_temp_c=ext_bme_temp_c,
        ext_bme_pressure_pa=ext_bme_pressure_pa,
        ext_bme_humidity=ext_bme_humidity,
        int_bme_temp_c=int_bme_temp_c,
        int_bme_pressure_pa=int_bme_pressure_pa,
        int_bme_humidity=int_bme_humidity,
        abp_pressure=abp_pressure,
        slf3s_flow_ml=slf3s_flow_ml,
        pt100_temp_c=pt100_temp_c,
        imu_accel_x=imu_accel_x,
        imu_accel_y=imu_accel_y,
        imu_accel_z=imu_accel_z,
        imu_gyro_x=imu_gyro_x,
        imu_gyro_y=imu_gyro_y,
        imu_gyro_z=imu_gyro_z,
        status_error_flag=status_error_flag
    )