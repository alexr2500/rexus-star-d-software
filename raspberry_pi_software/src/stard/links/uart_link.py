import time
import serial

from stard.links import protocol
from stard.links.framing import FrameParser, build_frame
from stard.links import messages


class UartLink:
    """Slave side of the ESP32 <-> Pi UART link.

    Responds to polls, executes forwarded commands, and passes sensor
    data to a logging callback. Never initiates an exchange.
    """

    def __init__(self, port: str, status_source, sensor_sink, command_handler) -> None:
        self._ser = serial.Serial(port, protocol.BAUD_RATE, timeout=0)
        self._parser = FrameParser()

        self._status_source = status_source      # callable -> Status
        self._sensor_sink = sensor_sink          # callable(Sensor) -> None
        self._command_handler = command_handler  # callable(Command) -> result

        self._mode = protocol.SoftwareMode.MODE_SU
        self._degraded = False
        self._time_offset_ms = 0
        self._last_poll_monotonic = None

        self._last_command_seq = None
        self._last_command_result = protocol.CommandResult.RESULT_NONE