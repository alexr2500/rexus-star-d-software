"""Unit tests for UartLink.

A fake serial port lets the whole link be exercised on any machine with no
hardware attached: bytes are handed in directly and replies captured.
"""

import pytest

from stard.links import messages, protocol
from stard.links.framing import FrameParser, build_frame
from stard.links.uart_link import UartLink


class FakeSerial:
    """Substitute for serial.Serial: rx is fed in, tx is captured."""

    def __init__(self) -> None:
        self.rx = bytearray()
        self.tx = bytearray()
        self.fail_on_read = False

    def feed(self, data: bytes) -> None:
        self.rx.extend(data)

    def read(self, size: int) -> bytes:
        if self.fail_on_read:
            raise OSError("port disconnected")
        chunk = bytes(self.rx[:size])
        del self.rx[:size]
        return chunk

    def write(self, data: bytes) -> int:
        self.tx.extend(data)
        return len(data)


def make_link():
    """A link wired to a fake port, with recording stubs for the callbacks."""
    ser = FakeSerial()
    sensors: list[messages.Sensor] = []
    commands: list[messages.Command] = []

    def status_source() -> messages.Status:
        return messages.Status(
            camera_status=protocol.CameraStatus.CAM_OK,
            ssd_free_gb=97,
            command_seq_echo=0,
            command_result=protocol.CommandResult.RESULT_NONE,
        )

    def command_handler(cmd: messages.Command) -> protocol.CommandResult:
        commands.append(cmd)
        return protocol.CommandResult.RESULT_OK

    link = UartLink(ser, status_source, sensors.append, command_handler)
    return link, ser, sensors, commands


def poll_frame(mission_time_ms: int,
               mode: protocol.SoftwareMode = protocol.SoftwareMode.MODE_NG,
               time_ref: protocol.TimeRef = protocol.TimeRef.TIME_MISSION,
               degraded: bool = False,
               sods_active: bool = False) -> bytes:
    payload = messages.encode_poll(messages.Poll(
        mode=mode, degraded=degraded,
        mission_time_ms=mission_time_ms, time_ref=time_ref,
        sods_active=sods_active))
    return build_frame(protocol.MessageId.MSG_POLL, payload)


def sole_reply(ser: FakeSerial) -> messages.Status:
    """Parse the single frame the link transmitted."""
    frames = FrameParser().feed(bytes(ser.tx))
    assert len(frames) == 1
    msg_id, payload = frames[0]
    assert msg_id == protocol.MessageId.MSG_STATUS
    return messages.decode_status(payload)


# --- POLL handling ------------------------------------------------------

def test_poll_is_answered_with_status():
    link, ser, _, _ = make_link()
    ser.feed(poll_frame(1000))
    link.service()

    status = sole_reply(ser)
    assert status.camera_status == protocol.CameraStatus.CAM_OK
    assert status.ssd_free_gb == 97


def test_poll_adopts_mode_and_degraded_flag():
    link, ser, _, _ = make_link()
    ser.feed(poll_frame(1000, mode=protocol.SoftwareMode.MODE_NE, degraded=True))
    link.service()

    assert link.mode() == protocol.SoftwareMode.MODE_NE
    assert link.degraded() is True


def test_mission_time_tracks_the_anchor():
    """After a poll, local mission time must sit close to the value sent."""
    link, ser, _, _ = make_link()
    ser.feed(poll_frame(142_300))
    link.service()

    assert abs(link.mission_time_ms() - 142_300) < 50
    assert link.time_ref() == protocol.TimeRef.TIME_MISSION


def test_mission_time_survives_link_loss():
    """No further polls: timestamps must remain valid from the last anchor."""
    link, ser, _, _ = make_link()
    ser.feed(poll_frame(50_000))
    link.service()

    for _ in range(10):
        link.service()          # no data arriving

    assert abs(link.mission_time_ms() - 50_000) < 100


def test_liftoff_reanchors_from_countdown():
    """A COUNTDOWN anchor is replaced by MISSION with no special case."""
    link, ser, _, _ = make_link()
    ser.feed(poll_frame(-30_000, time_ref=protocol.TimeRef.TIME_COUNTDOWN))
    link.service()
    assert link.mission_time_ms() < 0

    ser.tx.clear()
    ser.feed(poll_frame(0, time_ref=protocol.TimeRef.TIME_MISSION))
    link.service()

    assert abs(link.mission_time_ms()) < 50
    assert link.time_ref() == protocol.TimeRef.TIME_MISSION


def test_time_ref_starts_unsynced():
    link, _, _, _ = make_link()
    assert link.time_ref() == protocol.TimeRef.TIME_UNSYNCED
    assert link.ms_since_last_poll() is None
    assert link.link_stale() is True


# --- COMMAND handling ---------------------------------------------------

def test_command_is_executed_once():
    link, ser, _, commands = make_link()
    payload = messages.encode_command(messages.Command(
        command_flag=protocol.Command.CMD_WIPE, seq_number=7))
    ser.feed(build_frame(protocol.MessageId.MSG_COMMAND, payload))
    link.service()

    assert len(commands) == 1
    assert commands[0].command_flag == protocol.Command.CMD_WIPE


def test_repeated_command_is_idempotent():
    """Retransmission must not re-execute: this is why seq exists."""
    link, ser, _, commands = make_link()
    payload = messages.encode_command(messages.Command(
        command_flag=protocol.Command.CMD_WIPE, seq_number=7))
    frame = build_frame(protocol.MessageId.MSG_COMMAND, payload)

    ser.feed(frame * 3)
    link.service()

    assert len(commands) == 1


def test_new_sequence_number_executes_again():
    link, ser, _, commands = make_link()
    for seq in (7, 8):
        payload = messages.encode_command(messages.Command(
            command_flag=protocol.Command.CMD_TEST, seq_number=seq))
        ser.feed(build_frame(protocol.MessageId.MSG_COMMAND, payload))
    link.service()

    assert len(commands) == 2


def test_command_result_is_echoed_in_next_status():
    link, ser, _, _ = make_link()
    payload = messages.encode_command(messages.Command(
        command_flag=protocol.Command.CMD_WIPE, seq_number=42))
    ser.feed(build_frame(protocol.MessageId.MSG_COMMAND, payload))
    ser.feed(poll_frame(1000))
    link.service()

    status = sole_reply(ser)
    assert status.command_seq_echo == 42
    assert status.command_result == protocol.CommandResult.RESULT_OK


def test_failing_command_reports_failure():
    ser = FakeSerial()

    def status_source() -> messages.Status:
        return messages.Status(
            camera_status=protocol.CameraStatus.CAM_OK, ssd_free_gb=50,
            command_seq_echo=0, command_result=protocol.CommandResult.RESULT_NONE)

    def failing_handler(cmd: messages.Command) -> protocol.CommandResult:
        raise RuntimeError("disk error")

    link = UartLink(ser, status_source, lambda s: None, failing_handler)

    payload = messages.encode_command(messages.Command(
        command_flag=protocol.Command.CMD_WIPE, seq_number=1))
    ser.feed(build_frame(protocol.MessageId.MSG_COMMAND, payload))
    ser.feed(poll_frame(1000))
    link.service()

    assert sole_reply(ser).command_result == protocol.CommandResult.RESULT_FAILED


# --- SENSOR_DATA handling -----------------------------------------------

def test_sensor_data_reaches_the_sink():
    link, ser, sensors, _ = make_link()
    sensor = messages.Sensor(
        ext_bme_temp_raw=-4000, ext_bme_pressure_raw=50650, ext_bme_humidity_raw=4500,
        int_bme_temp_raw=2345, int_bme_pressure_raw=101325 // 2, int_bme_humidity_raw=5000,
        abp_pressure_raw=12000, slf3s_flow_raw=-500, pt100_temp_raw=8500,
        imu_accel_x=-32768, imu_accel_y=0, imu_accel_z=32767,
        imu_gyro_x=-1000, imu_gyro_y=0, imu_gyro_z=1000,
        status_error_flag=0xDEADBEEF)
    ser.feed(build_frame(protocol.MessageId.MSG_SENSOR_DATA,
                         messages.encode_sensor(sensor)))
    link.service()

    assert sensors == [sensor]


def test_sensor_data_is_not_answered():
    """Only POLL solicits a reply."""
    link, ser, _, _ = make_link()
    sensor = messages.Sensor(*([0] * 16))
    ser.feed(build_frame(protocol.MessageId.MSG_SENSOR_DATA,
                         messages.encode_sensor(sensor)))
    link.service()

    assert ser.tx == b""


# --- Robustness ---------------------------------------------------------

def test_garbage_then_poll_still_answered():
    link, ser, _, _ = make_link()
    ser.feed(b"\x11\x22\x33" + poll_frame(1000))
    link.service()

    assert sole_reply(ser).ssd_free_gb == 97


def test_malformed_payload_does_not_stop_the_loop():
    """Wrong-length payload passes CRC but must not kill the link."""
    link, ser, _, _ = make_link()
    ser.feed(build_frame(protocol.MessageId.MSG_POLL, b"\x00\x00"))
    ser.feed(poll_frame(1000))
    link.service()

    assert link.decode_errors == 1
    assert sole_reply(ser).ssd_free_gb == 97


def test_invalid_enum_value_is_rejected():
    """Mode 99 is undefined: reject rather than propagate it."""
    link, ser, _, _ = make_link()
    import struct
    bad = struct.pack(messages.POLL_FORMAT, 99, 0, 0, 0, 0)
    ser.feed(build_frame(protocol.MessageId.MSG_POLL, bad))
    link.service()

    assert link.decode_errors == 1
    assert ser.tx == b""


def test_unexpected_message_is_counted():
    """The Pi sends STATUS; receiving one means something is wrong."""
    link, ser, _, _ = make_link()
    status = messages.Status(
        camera_status=protocol.CameraStatus.CAM_OK, ssd_free_gb=10,
        command_seq_echo=0, command_result=protocol.CommandResult.RESULT_NONE)
    ser.feed(build_frame(protocol.MessageId.MSG_STATUS,
                         messages.encode_status(status)))
    link.service()

    assert link.unexpected_messages == 1


def test_port_failure_does_not_raise():
    """A disconnected adapter must not stop the Pi recording."""
    link, ser, _, _ = make_link()
    ser.fail_on_read = True
    link.service()          # must not raise

    assert link.port_errors == 1


def test_split_poll_is_answered_once_complete():
    link, ser, _, _ = make_link()
    frame = poll_frame(1000)

    ser.feed(frame[:5])
    link.service()
    assert ser.tx == b""

    ser.feed(frame[5:])
    link.service()
    assert sole_reply(ser).ssd_free_gb == 97