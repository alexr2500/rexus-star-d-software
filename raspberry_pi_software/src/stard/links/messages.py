import struct
from dataclasses import dataclass

from stard.links import protocol


@dataclass
class Poll:
    """Contents of a POLL message (ESP32 -> Pi).

    Byte layout is protocol.h section 11 / protocol.py section 9. The
    flags byte is unpacked into named fields here so nothing downstream
    has to know the bit positions.
    """
    mode: protocol.SoftwareMode
    camera_rate: protocol.CameraRate
    log_rate: protocol.LogRate
    mission_time_ms: int
    time_ref: protocol.TimeRef
    sods_active: bool
    soe_active: bool
    lo_active: bool
    flight_confirmed: bool
    degraded: bool


# [0] mode, [1] flags, [2] camera rate, [3] log rate, [4..7] time ms.
#
# The time is SIGNED. Countdown time is negative (T-180 s is -180000),
# and csv_manager takes abs() of it to name files, so an unsigned field
# would turn every countdown stamp into a number near 4.29 billion.
# Kept as a struct format like the other messages so the size assertion
# in the tests still holds and the layout is readable in one line.
POLL_FORMAT = "<BBBBi"


def encode_poll(p: Poll) -> bytes:
    flags = 0
    if p.sods_active:      flags |= protocol.POLL_FLAG_SODS
    if p.soe_active:       flags |= protocol.POLL_FLAG_SOE
    if p.lo_active:        flags |= protocol.POLL_FLAG_LO
    if p.flight_confirmed: flags |= protocol.POLL_FLAG_FLIGHT
    if p.degraded:         flags |= protocol.POLL_FLAG_DEGRADED
    flags |= (int(p.time_ref) << protocol.POLL_TIMEREF_SHIFT) & protocol.POLL_TIMEREF_MASK

    return struct.pack(POLL_FORMAT, int(p.mode), flags,
                       int(p.camera_rate), int(p.log_rate),
                       p.mission_time_ms)


def decode_poll(payload: bytes) -> Poll:
    if len(payload) != protocol.LEN_POLL:
        raise ValueError(
            f"POLL payload must be {protocol.LEN_POLL} bytes, got {len(payload)}"
        )

    mode, flags, cam_rate, log_rate, mission_time_ms = struct.unpack(
        POLL_FORMAT, payload
    )
    time_ref_bits = (flags & protocol.POLL_TIMEREF_MASK) >> protocol.POLL_TIMEREF_SHIFT

    return Poll(
        mode=protocol.SoftwareMode(mode),
        camera_rate=protocol.CameraRate(cam_rate),
        log_rate=protocol.LogRate(log_rate),
        mission_time_ms=mission_time_ms,
        time_ref=protocol.TimeRef(time_ref_bits),
        sods_active=bool(flags & protocol.POLL_FLAG_SODS),
        soe_active=bool(flags & protocol.POLL_FLAG_SOE),
        lo_active=bool(flags & protocol.POLL_FLAG_LO),
        flight_confirmed=bool(flags & protocol.POLL_FLAG_FLIGHT),
        degraded=bool(flags & protocol.POLL_FLAG_DEGRADED),
    )


@dataclass
class Command:
    """Contents of a COMMAND message (ESP32 -> Pi)."""
    command_flag: protocol.Command
    seq_number: int


COMMAND_FORMAT = "<BB"


def encode_command(c: Command) -> bytes:
    return struct.pack(COMMAND_FORMAT, c.command_flag, c.seq_number)


def decode_command(payload: bytes) -> Command:
    if len(payload) != protocol.LEN_COMMAND:
        raise ValueError(f"COMMAND payload must be {protocol.LEN_COMMAND} bytes, got {len(payload)}")
    command_flag, seq_number = struct.unpack(COMMAND_FORMAT, payload)
    return Command(
        command_flag=protocol.Command(command_flag),
        seq_number=seq_number,
    )


@dataclass
class Status:
    """Contents of a STATUS message (Pi -> ESP32)"""
    camera_status: protocol.CameraStatus
    ssd_free_gb: int
    command_seq_echo: int                    # plain counter, not an enum
    command_result: protocol.CommandResult


STATUS_FORMAT = "<BBBB"


def encode_status(s: Status) -> bytes:
    return struct.pack(STATUS_FORMAT, s.camera_status, s.ssd_free_gb,
                       s.command_seq_echo, s.command_result)


def decode_status(payload: bytes) -> Status:
    if len(payload) != protocol.LEN_STATUS:
        raise ValueError(f"STATUS payload must be {protocol.LEN_STATUS} bytes, got {len(payload)}")
    camera_status, ssd_free_gb, command_seq_echo, command_result = struct.unpack(STATUS_FORMAT, payload)
    return Status(
        camera_status=protocol.CameraStatus(camera_status),
        ssd_free_gb=ssd_free_gb,
        command_seq_echo=command_seq_echo,
        command_result=protocol.CommandResult(command_result),
    )


@dataclass
class Sensor:
    """Contents of a SENSOR_DATA message (ESP32 -> Pi)"""
    ext_bme_temp_raw: int
    ext_bme_pressure_raw: int
    ext_bme_humidity_raw: int
    int_bme_temp_raw: int
    int_bme_pressure_raw: int
    int_bme_humidity_raw: int
    abp_pressure_raw: int
    slf3s_flow_raw: int
    pt100_temp_raw: int
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
        se.ext_bme_temp_raw,
        se.ext_bme_pressure_raw,
        se.ext_bme_humidity_raw,
        se.int_bme_temp_raw,
        se.int_bme_pressure_raw,
        se.int_bme_humidity_raw,
        se.abp_pressure_raw,
        se.slf3s_flow_raw,
        se.pt100_temp_raw,
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
    (ext_bme_temp_raw, ext_bme_pressure_raw, ext_bme_humidity_raw,
    int_bme_temp_raw, int_bme_pressure_raw, int_bme_humidity_raw,
    abp_pressure_raw, slf3s_flow_raw, pt100_temp_raw,
    imu_accel_x, imu_accel_y, imu_accel_z,
    imu_gyro_x, imu_gyro_y, imu_gyro_z,
    status_error_flag) = struct.unpack(SENSOR_DATA_FORMAT, payload)
    return Sensor(
            ext_bme_temp_raw=ext_bme_temp_raw,
            ext_bme_pressure_raw=ext_bme_pressure_raw,
            ext_bme_humidity_raw=ext_bme_humidity_raw,
            int_bme_temp_raw=int_bme_temp_raw,
            int_bme_pressure_raw=int_bme_pressure_raw,
            int_bme_humidity_raw=int_bme_humidity_raw,
            abp_pressure_raw=abp_pressure_raw,
            slf3s_flow_raw=slf3s_flow_raw,
            pt100_temp_raw=pt100_temp_raw,
            imu_accel_x=imu_accel_x,
            imu_accel_y=imu_accel_y,
            imu_accel_z=imu_accel_z,
            imu_gyro_x=imu_gyro_x,
            imu_gyro_y=imu_gyro_y,
            imu_gyro_z=imu_gyro_z,
            status_error_flag=status_error_flag
        )