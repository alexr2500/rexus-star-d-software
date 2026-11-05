"""Pi side of the ESP32 <-> Raspberry Pi SPI link.

The Pi is the SPI MASTER and the ESP32 is the slave, because Linux
spidev is master-only. That is electrical, not architectural: the Pi
clocks the bus at a fixed rate and decides nothing. The ESP32 still
owns the mission -- it chooses the content of every frame it sends,
and this module only answers with STATUS and acts on what it is told.

Differences from the UART version that matter:

  * Every transaction is full duplex and fixed length. The Pi sends
    STATUS in all of them, not only in response to a POLL, so the ESP
    gets status at 10 Hz instead of 1 Hz.

  * One transaction = one complete frame. No stream, no resync, so no
    FrameParser -- a frame either starts at byte 0 or the transaction
    was bad.

  * THE PI MUST NEVER BLOCK. It is the clock source now: if this loop
    stalls, the link stops for both ends and the ESP32 cannot tell a
    busy Pi from a dead one. Slow commands run on a worker thread and
    report RESULT_PENDING.

  * The handshake line is not optional. Clocking while the slave has
    no buffer armed shifts out whatever was last in its DMA buffer --
    stale data with a valid CRC, which is worse than noise.

See docs/interfaces/spi_esp32_pi.md.
"""

import threading
import time
from typing import Callable, Optional, Protocol as TypingProtocol

from stard.links import messages, protocol
from stard.links.framing import build_frame, crc16_ccitt


class SpiDevice(TypingProtocol):
    """Minimal spidev interface, so tests can substitute a fake."""

    def xfer2(self, data: list[int]) -> list[int]: ...
    def close(self) -> None: ...


StatusSource = Callable[[], messages.Status]
SensorSink = Callable[[messages.Sensor], None]
CommandHandler = Callable[[messages.Command], protocol.CommandResult]
HandshakeRead = Callable[[], bool]

# Commands expected to outlive one transaction. These run on a worker
# thread and report RESULT_PENDING until they finish. WIPE deletes
# gigabytes; blocking on it would stop the clock for several seconds.
SLOW_COMMANDS = frozenset({protocol.Command.CMD_WIPE})

# How long to wait for the handshake before giving up on this slot.
# Short: a missed slot costs 100 ms, a blocked loop costs the link.
HANDSHAKE_TIMEOUT_S = 0.020


class SpiLink:
    """Clocks the bus, answers with STATUS, forwards commands and data.

    The link is not required for the Pi to keep recording: if the ESP32
    stops responding, imaging and logging continue using the last known
    mode and the locally maintained mission-time offset.
    """

    def __init__(
        self,
        device: tuple[int, int] | SpiDevice,
        status_source: StatusSource,
        sensor_sink: SensorSink,
        command_handler: CommandHandler,
        handshake: Optional[HandshakeRead] = None,
    ) -> None:
        if isinstance(device, tuple):
            import spidev

            spi = spidev.SpiDev()
            spi.open(device[0], device[1])
            spi.mode = protocol.SPI_MODE
            spi.max_speed_hz = protocol.SPI_CLOCK_HZ
            self._spi: SpiDevice = spi
        else:
            self._spi = device

        self._status_source = status_source
        self._sensor_sink = sensor_sink
        self._command_handler = command_handler
        self._handshake = handshake

        self._mode: protocol.SoftwareMode = protocol.SoftwareMode.MODE_SU
        self._degraded: bool = False

        # Pushed by the ESP32's state broadcast once a second. The Pi
        # reacts to these idempotently rather than waiting for commands.
        self._camera_rate: protocol.CameraRate = protocol.CameraRate.OFF
        self._log_rate: protocol.LogRate = protocol.LogRate.OFF
        self._sods_active: bool = False
        self._flight_confirmed: bool = False

        self._time_offset_ms: int = 0
        self._time_ref: protocol.TimeRef = protocol.TimeRef.TIME_UNSYNCED
        self._last_poll_monotonic: Optional[int] = None
        self._last_xfer_monotonic: Optional[int] = None

        self._last_command_seq: Optional[int] = None
        self._last_command_result: protocol.CommandResult = (
            protocol.CommandResult.RESULT_NONE
        )
        self._command_lock = threading.Lock()

        self._next_xfer_at: float = 0.0

        # Diagnostics, surfaced post-flight rather than over the link.
        self.decode_errors: int = 0
        self.unexpected_messages: int = 0
        self.port_errors: int = 0
        self.handshake_timeouts: int = 0
        self.idle_frames: int = 0

    # --- Time -----------------------------------------------------------

    def _monotonic_ms(self) -> int:
        """Milliseconds from a clock that never moves backwards.

        time.time() must not be used: it can jump when the system clock
        is corrected, which would corrupt every interval computed
        across it.
        """
        return round(time.monotonic() * 1000)

    def mission_time_ms(self) -> int:
        """Current time in the reference reported by time_ref().

        The offset is re-established on every POLL, so this keeps
        returning correct values even if the link subsequently fails.
        """
        return self._monotonic_ms() + self._time_offset_ms

    def time_ref(self) -> protocol.TimeRef:
        """What mission_time_ms() currently means.

        UNSYNCED  - ESP32 uptime, ordering only, no mission reference.
        COUNTDOWN - anchored on SODS. An estimate: holds and scrubs
                    invalidate it, so it must not be treated as truth.
        MISSION   - anchored on LO. Ground truth.
        """
        return self._time_ref

    # --- Link state -----------------------------------------------------

    def mode(self) -> protocol.SoftwareMode:
        """Mode last reported by the ESP32."""
        return self._mode

    def camera_rate(self) -> protocol.CameraRate:
        """Capture cadence the ESP32 is asking for. OFF until the first
        poll arrives, so the Pi never records before being told to."""
        return self._camera_rate

    def log_rate(self) -> protocol.LogRate:
        """Logging cadence the ESP32 is asking for."""
        return self._log_rate

    def sods_active(self) -> bool:
        """SODS gates data storage and imaging from about T-180 s."""
        return self._sods_active

    def flight_confirmed(self) -> bool:
        """LO received, or launch detected by the ESP32's accelerometer."""
        return self._flight_confirmed

    def degraded(self) -> bool:
        """Degraded flag last reported by the ESP32."""
        return self._degraded

    def ms_since_last_poll(self) -> Optional[int]:
        """Milliseconds since the last POLL, or None if none has arrived."""
        if self._last_poll_monotonic is None:
            return None
        return self._monotonic_ms() - self._last_poll_monotonic

    def ms_since_last_frame(self) -> Optional[int]:
        """Since the last transaction that carried a valid frame.

        Distinct from ms_since_last_poll: POLL arrives at 1 Hz by
        design, so three seconds without one is normal-ish, while three
        seconds without ANY valid frame means the ESP32 is gone.
        """
        if self._last_xfer_monotonic is None:
            return None
        return self._monotonic_ms() - self._last_xfer_monotonic

    def link_stale(
        self, threshold_ms: int = 3 * protocol.POLL_PERIOD_MS
    ) -> bool:
        """True when valid frames have stopped for longer than threshold.

        The Pi cannot report this to ground; it is for local decisions
        such as whether the reported mode can still be trusted.
        """
        elapsed = self.ms_since_last_frame()
        return elapsed is None or elapsed > threshold_ms

    # --- Main loop ------------------------------------------------------

    def service(self) -> None:
        """One transaction. Call repeatedly; paces itself to 10 Hz.

        Never blocks for longer than HANDSHAKE_TIMEOUT_S. The Pi is the
        clock source, so a stall here stops the link in both directions.
        """
        now = time.monotonic()
        if now < self._next_xfer_at:
            return
        self._next_xfer_at = now + protocol.SPI_TRANSACTION_PERIOD_MS / 1000.0

        if not self._wait_handshake():
            self.handshake_timeouts += 1
            return

        tx = self._build_status_frame()

        try:
            rx = bytes(self._spi.xfer2(list(tx)))
        except OSError:
            # Bus lost. Recording continues regardless of link state.
            self.port_errors += 1
            return

        self._handle_frame(rx)

    def _wait_handshake(self) -> bool:
        """True once the ESP32 says its TX buffer is armed."""
        if self._handshake is None:
            return True
        deadline = time.monotonic() + HANDSHAKE_TIMEOUT_S
        while time.monotonic() < deadline:
            if self._handshake():
                return True
            time.sleep(0.0005)
        return False

    # --- Receive --------------------------------------------------------

    def _handle_frame(self, rx: bytes) -> None:
        """Decode one fixed-length transaction.

        A valid frame starts at byte 0. Anything else means the slave
        had nothing armed, the mode is wrong, or the bus is noisy --
        all of which are "no frame", not "resync and try again".
        """
        if len(rx) < protocol.PROTO_OVERHEAD_BYTES:
            self.decode_errors += 1
            return

        if rx[0] != protocol.PROTO_SYNC0 or rx[1] != protocol.PROTO_SYNC1:
            # Almost always an unarmed slave: all zeros. Not an error
            # worth counting separately from a missing frame.
            return

        msg_id_raw = rx[1 + 1]
        length = rx[3]

        if length + protocol.PROTO_OVERHEAD_BYTES > len(rx):
            self.decode_errors += 1
            return

        payload = rx[4 : 4 + length]
        want = int.from_bytes(rx[4 + length : 6 + length], "little")
        if crc16_ccitt(rx[2 : 4 + length]) != want:
            self.decode_errors += 1
            return

        self._last_xfer_monotonic = self._monotonic_ms()

        try:
            msg_id = protocol.MessageId(msg_id_raw)
        except ValueError:
            self.unexpected_messages += 1
            return

        try:
            if msg_id == protocol.MessageId.MSG_POLL:
                self._handle_poll(payload)
            elif msg_id == protocol.MessageId.MSG_COMMAND:
                self._handle_command(payload)
            elif msg_id == protocol.MessageId.MSG_SENSOR_DATA:
                self._handle_sensor_data(payload)
            elif msg_id == protocol.MessageId.MSG_IDLE:
                # The ESP32 is alive and had nothing queued this slot.
                # Expected in most slots; counted so "alive but quiet"
                # stays distinguishable from "never armed its buffer".
                self.idle_frames += 1
            else:
                # MSG_STATUS is Pi -> ESP32 only. Receiving one means
                # something is looping back or the roles are confused.
                self.unexpected_messages += 1
        except ValueError:
            # CRC passed but the payload is malformed: wrong length or
            # an out-of-range enum. Drop this frame, keep the loop alive.
            self.decode_errors += 1

    def _handle_poll(self, payload: bytes) -> None:
        poll = messages.decode_poll(payload)

        self._mode = poll.mode
        self._degraded = poll.degraded
        self._camera_rate = poll.camera_rate
        self._log_rate = poll.log_rate
        self._sods_active = poll.sods_active
        self._flight_confirmed = poll.flight_confirmed

        # Re-anchor on every poll. This absorbs crystal drift and picks
        # up re-anchoring at SODS and LO with no special case.
        now = self._monotonic_ms()
        self._time_offset_ms = poll.mission_time_ms - now
        self._time_ref = poll.time_ref
        self._last_poll_monotonic = now

    def _handle_command(self, payload: bytes) -> None:
        cmd = messages.decode_command(payload)

        # Idempotence: act only when the sequence number is new, so a
        # retransmitted COMMAND executes exactly once.
        with self._command_lock:
            if cmd.seq_number == self._last_command_seq:
                return
            self._last_command_seq = cmd.seq_number

        if cmd.command_flag in SLOW_COMMANDS:
            # Report PENDING now and finish on a worker. Blocking here
            # would stop the clock for the whole system.
            with self._command_lock:
                self._last_command_result = protocol.CommandResult.RESULT_PENDING
            threading.Thread(
                target=self._run_command, args=(cmd,), daemon=True
            ).start()
        else:
            self._run_command(cmd)

    def _run_command(self, cmd: messages.Command) -> None:
        try:
            result = self._command_handler(cmd)
        except Exception:
            result = protocol.CommandResult.RESULT_FAILED
        with self._command_lock:
            self._last_command_result = result

    def _handle_sensor_data(self, payload: bytes) -> None:
        # The Pi makes no decisions on sensor data; it only logs it.
        self._sensor_sink(messages.decode_sensor(payload))

    # --- Transmit -------------------------------------------------------

    def _build_status_frame(self) -> bytes:
        """STATUS, padded to the fixed transaction length.

        Built fresh every transaction. Must be cheap: it sits directly
        in the clock path, so anything slow here slows the whole link.
        """
        status = self._status_source()
        with self._command_lock:
            status.command_seq_echo = self._last_command_seq or 0
            status.command_result = self._last_command_result

        frame = build_frame(
            protocol.MessageId.MSG_STATUS, messages.encode_status(status)
        )
        return frame.ljust(protocol.SPI_FRAME_LEN, b"\x00")

    # --- Shutdown -------------------------------------------------------

    def close(self) -> None:
        try:
            self._spi.close()
        except Exception:
            pass


# The class was called UartLink while the link was a serial port. The
# transport changed to SPI; the role did not. Kept as an alias so older
# callers and tests keep working rather than failing at import time.
UartLink = SpiLink
