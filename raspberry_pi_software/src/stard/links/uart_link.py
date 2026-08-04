"""Slave side of the ESP32 <-> Raspberry Pi UART link.

The ESP32 is the polling master: it initiates every exchange. This module
never transmits unsolicited data. See docs/interfaces/uart_esp32_pi.md.
"""

import time
from typing import Callable, Optional, Protocol as TypingProtocol

import serial

from stard.links import messages, protocol
from stard.links.framing import FrameParser, build_frame


class SerialPort(TypingProtocol):
    """Minimal serial interface, so tests can substitute a fake."""

    def read(self, size: int) -> bytes: ...
    def write(self, data: bytes) -> int: ...


StatusSource = Callable[[], messages.Status]
SensorSink = Callable[[messages.Sensor], None]
CommandHandler = Callable[[messages.Command], protocol.CommandResult]


class UartLink:
    """Responds to polls, forwards commands, and passes sensor data to a sink.

    The link is not required for the Pi to keep recording: if polls stop
    arriving, imaging and logging continue using the last known mode and
    the locally maintained mission-time offset.
    """

    READ_CHUNK = 256

    def __init__(
        self,
        port: str | SerialPort,
        status_source: StatusSource,
        sensor_sink: SensorSink,
        command_handler: CommandHandler,
    ) -> None:
        if isinstance(port, str):
            # timeout=0 -> non-blocking reads; read() returns whatever is
            # buffered, possibly nothing, and never stalls the main loop.
            self._ser: SerialPort = serial.Serial(port, protocol.BAUD_RATE, timeout=0)
        else:
            self._ser = port

        self._parser = FrameParser()

        self._status_source = status_source
        self._sensor_sink = sensor_sink
        self._command_handler = command_handler

        self._mode: protocol.SoftwareMode = protocol.SoftwareMode.MODE_SU
        self._degraded: bool = False

        self._time_offset_ms: int = 0
        self._time_ref: protocol.TimeRef = protocol.TimeRef.TIME_UNSYNCED
        self._last_poll_monotonic: Optional[int] = None

        self._last_command_seq: Optional[int] = None
        self._last_command_result: protocol.CommandResult = protocol.CommandResult.RESULT_NONE

        # Diagnostics, surfaced post-flight rather than over the link.
        self.decode_errors: int = 0
        self.unexpected_messages: int = 0
        self.port_errors: int = 0

    # --- Time -----------------------------------------------------------

    def _monotonic_ms(self) -> int:
        """Milliseconds from a clock that never moves backwards.

        time.time() must not be used: it can jump when the system clock is
        corrected, which would corrupt every interval computed across it.
        """
        return round(time.monotonic() * 1000)

    def mission_time_ms(self) -> int:
        """Current time in the reference reported by time_ref().

        The offset is re-established on every POLL, so this keeps returning
        correct values even if the link subsequently fails.
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

    def degraded(self) -> bool:
        """Degraded flag last reported by the ESP32."""
        return self._degraded

    def ms_since_last_poll(self) -> Optional[int]:
        """Milliseconds since the last POLL, or None if none has arrived."""
        if self._last_poll_monotonic is None:
            return None
        return self._monotonic_ms() - self._last_poll_monotonic

    def link_stale(self, threshold_ms: int = 3 * protocol.POLL_PERIOD_MS) -> bool:
        """True when polls have stopped for longer than the threshold.

        The Pi cannot report this to ground; it is for local decisions such
        as whether the reported mode can still be trusted.
        """
        elapsed = self.ms_since_last_poll()
        return elapsed is None or elapsed > threshold_ms

    # --- Main loop ------------------------------------------------------

    def service(self) -> None:
        """One iteration of the receive loop. Call repeatedly; never blocks."""
        try:
            data = self._ser.read(self.READ_CHUNK)
        except (serial.SerialException, OSError):
            # Port lost. Recording continues regardless of link state.
            self.port_errors += 1
            return

        for msg_id, payload in self._parser.feed(data):
            try:
                if msg_id == protocol.MessageId.MSG_POLL:
                    self._handle_poll(payload)
                elif msg_id == protocol.MessageId.MSG_COMMAND:
                    self._handle_command(payload)
                elif msg_id == protocol.MessageId.MSG_SENSOR_DATA:
                    self._handle_sensor_data(payload)
                else:
                    # MSG_STATUS is Pi -> ESP32 only; anything else is
                    # undefined. Either indicates a protocol mismatch.
                    self.unexpected_messages += 1
            except ValueError:
                # CRC passed but the payload is malformed: wrong length or
                # an out-of-range enum. Drop this frame, keep the loop alive.
                self.decode_errors += 1

    # --- Handlers -------------------------------------------------------

    def _handle_poll(self, payload: bytes) -> None:
        poll = messages.decode_poll(payload)

        self._mode = poll.mode
        self._degraded = poll.degraded

        # Re-anchor on every poll. This absorbs crystal drift and picks up
        # re-anchoring at SODS and LO with no special case.
        now = self._monotonic_ms()
        self._time_offset_ms = poll.mission_time_ms - now
        self._time_ref = poll.time_ref
        self._last_poll_monotonic = now

        # TODO: act on mode transitions (NE -> full frame rate, etc.)

        self._send_status()

    def _handle_command(self, payload: bytes) -> None:
        cmd = messages.decode_command(payload)

        # Idempotence: act only when the sequence number is new, so a
        # retransmitted COMMAND executes exactly once.
        if cmd.seq_number == self._last_command_seq:
            return

        self._last_command_seq = cmd.seq_number

        # TODO: WIPE may take far longer than the 50 ms reply window. Run
        # slow commands on a worker thread, return RESULT_PENDING here, and
        # update _last_command_result on completion.
        try:
            self._last_command_result = self._command_handler(cmd)
        except Exception:
            self._last_command_result = protocol.CommandResult.RESULT_FAILED

    def _handle_sensor_data(self, payload: bytes) -> None:
        # The Pi makes no decisions on sensor data; it only logs it.
        self._sensor_sink(messages.decode_sensor(payload))

    # --- Transmit -------------------------------------------------------

    def _send_status(self) -> None:
        """Reply to a POLL. Must complete within REPLY_TIMEOUT_MS."""
        status = self._status_source()
        status.command_seq_echo = self._last_command_seq or 0
        status.command_result = self._last_command_result

        frame = build_frame(protocol.MessageId.MSG_STATUS,
                            messages.encode_status(status))
        try:
            self._ser.write(frame)
        except (serial.SerialException, OSError):
            self.port_errors += 1