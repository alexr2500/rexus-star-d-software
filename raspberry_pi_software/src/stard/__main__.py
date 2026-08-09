"""STAR-D Raspberry Pi flight software entry point."""

import sys
import time

import serial

from stard.links import messages, protocol
from stard.links.uart_link import UartLink

LOOP_PERIOD_S = 0.01          # 100 Hz — comfortably faster than 10 Hz sensor data
SERIAL_PORT = "COM3"          # FIXME: read from config/mission_config.yaml / CHECK ELEC SCHEMATICS


def make_status() -> messages.Status:
    """Answer a POLL. Runs inside the 50 ms reply window, so must be fast."""
    # TODO: real camera health and SSD free space.
    # command_seq_echo and command_result are overwritten by the link.
    return messages.Status(
        camera_status=protocol.CameraStatus.CAM_UNKNOWN,
        ssd_free_gb=0,
        command_seq_echo=0,
        command_result=protocol.CommandResult.RESULT_NONE,
    )


def log_sensors(sensor: messages.Sensor) -> None:
    """Receive one SENSOR_DATA message at 10 Hz."""
    # TODO: hand to the CSV writer.
    # Printing every message floods the terminal — throttle it.
    ...


def handle_command(cmd: messages.Command) -> protocol.CommandResult:
    if cmd.command_flag == protocol.Command.CMD_WIPE:
        # TODO: delete oldest files first; must not block the 50 ms window
        return protocol.CommandResult.RESULT_OK
    if cmd.command_flag == protocol.Command.CMD_TEST:
        # TODO: verify camera responds and SSD is writable
        return protocol.CommandResult.RESULT_OK
    return protocol.CommandResult.RESULT_FAILED


def main() -> None:
    link = UartLink(SERIAL_PORT, make_status, log_sensors, handle_command)  # Useful only if port is present
    print(f"Link open on {SERIAL_PORT}")                                    # Do not use if bench testing with no ESP

    next_tick = time.monotonic()
    try:
        while True:
            link.service()

            # TODO later: camera.service(), storage.service()

            next_tick += LOOP_PERIOD_S
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        print("\nShutting down.")
        # TODO: stop recording, flush and close files


if __name__ == "__main__":
    main()