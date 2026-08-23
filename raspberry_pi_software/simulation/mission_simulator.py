"""Standalone mission simulator for the STAR-D Pi flight software.

Runs the real, unmodified flight stack (UartLink, FrameParser, the CSV
loggers, VideoWriter, StorageManager, CameraControl) against an in-process
loopback serial link and a synthetic camera backend, so the whole Pi
software can be exercised on a bench machine with no ESP32 and no IMX219
attached.

Only three things are substituted, all passed in through the constructor
seams added for this purpose (`stard.__main__.SoftwarePiComputer(port=...,
data_root=..., picamera_factory=...)`):
  - LoopbackSerial, standing in for the ESP32 <-> Pi UART link
  - FakePicamera2, standing in for picamera2.Picamera2
  - a scratch output directory

Timeline (see TIMELINE / COMMAND_EVENTS below), expressed as mission time
(T-/T+, anchored on LO) once synced, and uptime (UP+) before that. This is
heavily compressed: real SODS occurs at T-180 s, TEST/WIPE would be sent at
operator discretion during a multi-minute hold, and the whole countdown
runs for the length of the actual REXUS count. Here the ground segment
(SODS -> LO) is compressed to 90 s and the flight segment (LO -> SOE off)
to 60 s, so a bench run finishes in well under a minute even at --speed 1.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import sys
import threading
import time
from pathlib import Path

_SRC = Path(__file__).resolve().parent.parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import stard.__main__ as stard_main  # noqa: E402
from stard.links import messages, protocol  # noqa: E402
from stard.links.framing import FrameParser, build_frame  # noqa: E402
from stard.storage import storage_manager  # noqa: E402


# --- Timeline ---------------------------------------------------------------
# Coordinates below are simulated seconds since power-on ("sim_t"), which is
# also what the ESP32 side would call its own uptime. LO sits at LO_S; every
# mission-time value the simulator prints or puts on the wire is derived
# from sim_t relative to LO_S (see _mission_time_ms), never from sim_t
# itself, so the printed timeline reads in T-/T+ like a real range log.

ESP_READY_S = 5.0     # UP+5:  ESP32 startup complete -> NG
SODS_S = 10.0          # T-90:  SODS asserted, countdown begins
TEST_CMD_S = 40.0      # T-60:  ground sends TEST
WIPE_CMD_S = 70.0      # T-30:  ground sends WIPE
LO_S = 100.0           # T+0:   liftoff -> NF, re-anchor to MISSION
SOE_ON_S = 120.0       # T+20:  SOE on -> NE, full frame rate
SOE_OFF_S = 150.0      # T+50:  SOE off -> NF, idle rate
END_S = 160.0          # T+60:  end of run

TIMELINE = [
    (0.0, "power_on", "Power on, ESP32 in SU"),
    (ESP_READY_S, "esp_ready", "ESP32 startup complete -> NG"),
    (SODS_S, "sods", "SODS asserted (imaging and CSV logging are the flight-design start of the official record)"),
    (LO_S, "lo", "LO -> NF, re-anchor to T+0, MISSION"),
    (SOE_ON_S, "soe_on", "SOE on -> NE, full frame rate"),
    (SOE_OFF_S, "soe_off", "SOE released -> NF, idle rate"),
    (END_S, "end", "End of run, clean shutdown"),
]

# Telecommands, injected as real COMMAND frames (not app method calls) with
# incrementing sequence numbers, exactly as the ESP32 would forward them.
COMMAND_EVENTS = [
    (TEST_CMD_S, protocol.Command.CMD_TEST, "TEST"),
    (WIPE_CMD_S, protocol.Command.CMD_WIPE, "WIPE"),
]


def _apply_event(name: str, state: dict) -> None:
    if name == "power_on":
        state["mode"] = protocol.SoftwareMode.MODE_SU
        state["time_ref"] = protocol.TimeRef.TIME_UNSYNCED
        state["sods_active"] = False
    elif name == "esp_ready":
        state["mode"] = protocol.SoftwareMode.MODE_NG
    elif name == "sods":
        state["time_ref"] = protocol.TimeRef.TIME_COUNTDOWN
        state["sods_active"] = True
    elif name == "lo":
        state["mode"] = protocol.SoftwareMode.MODE_NF
        state["time_ref"] = protocol.TimeRef.TIME_MISSION
    elif name == "soe_on":
        state["mode"] = protocol.SoftwareMode.MODE_NE
    elif name == "soe_off":
        state["mode"] = protocol.SoftwareMode.MODE_NF
    elif name == "end":
        pass


def _mission_time_ms(sim_t: float, state: dict) -> int:
    """Uptime while UNSYNCED; negative-to-zero relative to LO while
    COUNTDOWN; positive from zero once MISSION. LO sits at a fixed
    simulated-time coordinate (LO_S), so both post-SODS references use the
    same anchor and differ only in sign either side of it."""
    if state["time_ref"] == protocol.TimeRef.TIME_UNSYNCED:
        return round(sim_t * 1000)
    return round((sim_t - LO_S) * 1000)


def _format_mission_time(mission_time_ms: int, time_ref: protocol.TimeRef) -> str:
    """Format the way a range operator would read it: T-MM:SS.t / T+MM:SS.t
    once synced, UP+MM:SS.t before that so an unsynced reading can never be
    mistaken for a real mission-time anchor. The prefix itself carries the
    reference, per the task's "keep the reference tag visible"."""
    prefix = "UP" if time_ref == protocol.TimeRef.TIME_UNSYNCED else "T"
    sign = "-" if mission_time_ms < 0 else "+"
    total_tenths = round(abs(mission_time_ms) / 100)
    minutes, rem_tenths = divmod(total_tenths, 600)
    seconds, tenths = divmod(rem_tenths, 10)
    return f"{prefix}{sign}{minutes:02d}:{seconds:02d}.{tenths}"


def _build_sensor(sim_t: float) -> messages.Sensor:
    """Plausible, smoothly varying raw sensor words. Not flight-accurate
    physics - just enough motion that the CSV columns are visibly alive,
    with a bump on the accel channel near LO."""
    accel_spike = 3000 if abs(sim_t - LO_S) < 0.5 else 0
    pressure_drift = int(2000 * math.sin(sim_t / 20))
    return messages.Sensor(
        ext_bme_temp_raw=int(-4000 + 50 * math.sin(sim_t / 15)),
        ext_bme_pressure_raw=max(0, 50650 + pressure_drift),
        ext_bme_humidity_raw=4500,
        int_bme_temp_raw=int(2300 + 20 * math.sin(sim_t / 10)),
        int_bme_pressure_raw=max(0, 50600 + pressure_drift),
        int_bme_humidity_raw=5000,
        abp_pressure_raw=12000,
        slf3s_flow_raw=int(200 * math.sin(sim_t / 5)),
        pt100_temp_raw=int(2200 + 10 * math.sin(sim_t / 8)),
        imu_accel_x=0,
        imu_accel_y=0,
        imu_accel_z=1000 + accel_spike,
        imu_gyro_x=0,
        imu_gyro_y=0,
        imu_gyro_z=0,
        status_error_flag=0,
    )


# --- Loopback serial --------------------------------------------------------

class LoopbackSerial:
    """In-process substitute for serial.Serial: two independent byte
    streams, one per direction, so the Pi side and the simulated-ESP32
    side can run on separate threads without touching real hardware."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._to_pi = bytearray()    # ESP32 -> Pi
        self._to_esp = bytearray()   # Pi -> ESP32

    # --- Pi-facing interface (matches links.uart_link.SerialPort) -----

    def read(self, size: int) -> bytes:
        with self._lock:
            chunk = bytes(self._to_pi[:size])
            del self._to_pi[:size]
            return chunk

    def write(self, data: bytes) -> int:
        with self._lock:
            self._to_esp.extend(data)
        return len(data)

    # --- Simulated-ESP32-facing interface ------------------------------

    def esp_write(self, data: bytes) -> None:
        with self._lock:
            self._to_pi.extend(data)

    def esp_read(self, size: int = 4096) -> bytes:
        with self._lock:
            chunk = bytes(self._to_esp[:size])
            del self._to_esp[:size]
            return chunk


# --- Fake camera ------------------------------------------------------------

class _FakeRequest:
    def __init__(self, buffer: bytes, sensor_timestamp_ns: int) -> None:
        self._buffer = buffer
        self._sensor_timestamp_ns = sensor_timestamp_ns

    def make_buffer(self, name: str) -> bytes:
        return self._buffer

    def get_metadata(self) -> dict:
        return {"SensorTimestamp": self._sensor_timestamp_ns}

    def release(self) -> None:
        pass


class FakePicamera2:
    """Substitute for picamera2.Picamera2.

    A real 1640x1232 YUV420 frame is ~3 MB; 30 s of NE at ~42 fps would be
    ~3.8 GB, far too much for a bench validation run. This produces a
    small synthetic buffer (~4 KB) instead, while preserving the timing
    behaviour CameraControl actually depends on: capture_request() blocks
    for one frame interval, and SensorTimestamp advances by that interval
    every call, so STALL_THRESHOLD_FRAMES-style logic still has real
    timestamps to look at.
    """

    FRAME_BYTES = 4096

    def __init__(self, speed: float = 1.0) -> None:
        self._speed = speed
        self._duration_us = 250_000
        self._next_ts_ns = 0
        self._frame_index = 0

    def create_video_configuration(self, main=None, controls=None, buffer_count=None):
        limits = (controls or {}).get("FrameDurationLimits", (self._duration_us, self._duration_us))
        self._duration_us = limits[0]
        return {"main": main, "controls": controls, "buffer_count": buffer_count}

    def configure(self, config) -> None:
        pass

    def set_controls(self, controls: dict) -> None:
        if "FrameDurationLimits" in controls:
            self._duration_us = controls["FrameDurationLimits"][0]

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def capture_request(self) -> _FakeRequest:
        interval_s = (self._duration_us / 1_000_000) / self._speed
        time.sleep(interval_s)
        self._frame_index += 1
        self._next_ts_ns += self._duration_us * 1000
        buf = bytes([self._frame_index % 256]) * self.FRAME_BYTES
        return _FakeRequest(buf, self._next_ts_ns)


# --- Output-directory accounting --------------------------------------------
# Deliberately black-box: everything below reads the filesystem the way a
# ground operator would, rather than calling into StorageManager/VideoWriter
# internals, so it stays valid evidence of what actually happened on disk.

_VIDEO_RE = re.compile(r"video_(\w+)_(\d{7})\.yuv$")
_CSV_PREFIXES = ("sensors", "system_state", "telemetry_mirror", "uplink_commands")


def _scan_output(out_dir: str) -> tuple[int, int, dict[str, int], set[str]]:
    """One filesystem snapshot: (file_count, total_bytes, bytes_by_ext,
    relative_paths). Called every simulated second plus at both edges of
    WIPE, so it also doubles as the sampling source for rollover counts."""
    total_bytes = 0
    count = 0
    bytes_by_ext: dict[str, int] = {}
    relpaths: set[str] = set()
    for root, _dirs, files in os.walk(out_dir):
        for name in files:
            path = os.path.join(root, name)
            try:
                size = os.path.getsize(path)
            except OSError:
                continue
            total_bytes += size
            count += 1
            ext = os.path.splitext(name)[1]
            bytes_by_ext[ext] = bytes_by_ext.get(ext, 0) + size
            relpaths.add(os.path.relpath(path, out_dir))
    return count, total_bytes, bytes_by_ext, relpaths


def _rollover_counts(all_files_seen: set[str]) -> dict[str, int]:
    """Distinct filenames ever observed for each stream, minus one (the
    first file is not a rollover). `all_files_seen` is a running union
    updated on every scan, so it also remembers files WIPE later deleted -
    without that, a stream wiped mid-run would look like it never rolled."""
    counts: dict[str, int] = {}
    for prefix in _CSV_PREFIXES:
        n = sum(1 for p in all_files_seen if os.path.basename(p).startswith(prefix + "_"))
        counts[prefix] = max(0, n - 1)
    n_video = sum(1 for p in all_files_seen if os.path.basename(p).startswith("video_"))
    counts["video"] = max(0, n_video - 1)
    return counts


# --- Simulated ESP32 --------------------------------------------------------

def esp32_driver(loop: LoopbackSerial, computer, out_dir: str, speed: float,
                  stop_event: threading.Event, state: dict, poll_stats: dict,
                  mode_sequence: list, command_stats: dict,
                  all_files_seen: set) -> None:
    """Acts as polling master exactly as the real ESP32 does: sends
    SENSOR_DATA and POLL on their real cadences (scaled by --speed),
    applies the reply timeout, tracks the degraded rule, injects COMMAND
    frames from the timeline, and - since POLL_PERIOD_MS is exactly 1000 ms
    - prints one live status line per simulated second alongside it."""
    start = time.monotonic()

    def sim_t() -> float:
        return (time.monotonic() - start) * speed

    # Pre-existing bug fixed here: `t` (sim_t()) already runs at `speed`x
    # real time, so a period compared against `t` must be expressed in
    # sim-time units, not divided by speed again - the old code did both,
    # so polls fired ~speed^2 times too often (visible as e.g. a "once per
    # simulated second" status line printing every ~0.06 simulated seconds
    # at --speed 4). REPLY_TIMEOUT_MS, by contrast, is a real UART/hardware
    # budget compared against real time.monotonic() in wait_for_status();
    # it must NOT be compressed by --speed, or the reply deadline becomes
    # unrealistically tight and produces misses unrelated to anything the
    # flight software actually does (see the TEST self_test() capture
    # latency below for the deadline miss that IS representative).
    poll_period_s = protocol.POLL_PERIOD_MS / 1000
    sensor_period_s = protocol.SENSOR_DATA_PERIOD_MS / 1000
    reply_timeout_s = protocol.REPLY_TIMEOUT_MS / 1000

    status_parser = FrameParser()

    def wait_for_status():
        deadline = time.monotonic() + reply_timeout_s
        while time.monotonic() < deadline:
            chunk = loop.esp_read()
            if chunk:
                for msg_id, payload in status_parser.feed(chunk):
                    if msg_id == protocol.MessageId.MSG_STATUS:
                        return messages.decode_status(payload)
            time.sleep(0.0005)
        return None

    event_idx = 0
    cmd_idx = 0
    next_poll = 0.0
    next_sensor = 0.0
    consecutive_misses = 0
    consecutive_hits = 0
    seq_counter = 0
    pending_commands: dict[int, str] = {}     # seq -> command name, awaiting echo
    prev_frames_written = 0
    last_mode_printed = None

    def mt_str(t: float) -> str:
        return _format_mission_time(_mission_time_ms(t, state), state["time_ref"])

    def print_divider(tag: str, t: float, text: str) -> None:
        line = f"[{tag}]  {mt_str(t)}  {text}"
        print(f"\n{'=' * len(line)}")
        print(line)
        print("=" * len(line))

    while not stop_event.is_set():
        t = sim_t()

        # --- Timeline state events -----------------------------------
        while event_idx < len(TIMELINE) and TIMELINE[event_idx][0] <= t:
            ev_t, name, text = TIMELINE[event_idx]
            _apply_event(name, state)
            print_divider("EVENT", t, text)
            event_idx += 1
            if name == "end":
                stop_event.set()
            all_files_seen.update(_scan_output(out_dir)[3])
        if stop_event.is_set():
            break

        # --- Telecommands ----------------------------------------------
        while cmd_idx < len(COMMAND_EVENTS) and COMMAND_EVENTS[cmd_idx][0] <= t:
            _cmd_t, flag, name = COMMAND_EVENTS[cmd_idx]
            seq_counter += 1
            seq = seq_counter

            if flag == protocol.Command.CMD_WIPE:
                before_count, before_bytes, _ext, before_paths = _scan_output(out_dir)
                all_files_seen.update(before_paths)
                command_stats["wipe_files_before"] = before_count
                command_stats["wipe_before_bytes"] = before_bytes

            cmd = messages.Command(command_flag=flag, seq_number=seq)
            loop.esp_write(build_frame(protocol.MessageId.MSG_COMMAND,
                                        messages.encode_command(cmd)))
            print_divider("COMMAND", t, f"Ground sends {name} (seq={seq})")
            pending_commands[seq] = name
            cmd_idx += 1

        # --- SENSOR_DATA -------------------------------------------------
        if t >= next_sensor:
            sensor = _build_sensor(t)
            loop.esp_write(build_frame(protocol.MessageId.MSG_SENSOR_DATA,
                                        messages.encode_sensor(sensor)))
            next_sensor += sensor_period_s

        # --- POLL / STATUS, once per simulated second ---------------------
        if t >= next_poll:
            poll = messages.Poll(
                mode=state["mode"], degraded=state["degraded"],
                mission_time_ms=_mission_time_ms(t, state),
                time_ref=state["time_ref"], sods_active=state["sods_active"])
            loop.esp_write(build_frame(protocol.MessageId.MSG_POLL,
                                        messages.encode_poll(poll)))

            send_monotonic = time.monotonic()
            status = wait_for_status()
            poll_stats["sent"] += 1

            if status is None:
                poll_stats["misses"] += 1
                consecutive_misses += 1
                consecutive_hits = 0
                if consecutive_misses >= protocol.MISS_COUNT_DEGRADED:
                    state["degraded"] = True
            else:
                reply_ms = (time.monotonic() - send_monotonic) * 1000
                poll_stats["answered"] += 1
                poll_stats["last_reply_ms"] = reply_ms
                poll_stats["last_ssd_free_gb"] = status.ssd_free_gb
                poll_stats["reply_ms_sum"] += reply_ms
                poll_stats["reply_ms_count"] += 1
                poll_stats["min_reply_ms"] = min(poll_stats["min_reply_ms"], reply_ms)
                poll_stats["max_reply_ms"] = max(poll_stats["max_reply_ms"], reply_ms)
                consecutive_hits += 1
                consecutive_misses = 0
                if consecutive_hits >= protocol.SUCCESS_COUNT_CLEAR_DEGRADED:
                    state["degraded"] = False

                # Telecommand confirmation: the seq echoed back tells us
                # the command that seq names has finished executing (Pi
                # handles COMMAND synchronously, so by the time any later
                # STATUS echoes the seq, the result is final).
                seq_echo = status.command_seq_echo
                if seq_echo in pending_commands:
                    cmd_name = pending_commands.pop(seq_echo)
                    if cmd_name == "TEST":
                        command_stats["test_result"] = status.command_result
                        print_divider(
                            "RESULT", t,
                            f"TEST (seq={seq_echo}) -> {status.command_result.name}")
                    elif cmd_name == "WIPE":
                        after_count, after_bytes, _ext, after_paths = _scan_output(out_dir)
                        all_files_seen.update(after_paths)
                        freed = max(0, command_stats.get("wipe_before_bytes", 0) - after_bytes)
                        command_stats["wipe_result"] = status.command_result
                        command_stats["wipe_freed_bytes"] = freed
                        command_stats["wipe_files_after"] = after_count
                        print_divider(
                            "RESULT", t,
                            f"WIPE (seq={seq_echo}) -> {status.command_result.name}  "
                            f"freed={freed} bytes  "
                            f"files: {command_stats['wipe_files_before']} -> {after_count}")

            next_poll += poll_period_s

            # --- Live status line, once per POLL (POLL_PERIOD_MS == 1000,
            # so this is exactly once per simulated second as asked). All
            # fields below come from the live objects, not recomputed.
            mode = computer.effective_mode()
            if mode != last_mode_printed:
                mode_sequence.append(mode)
                last_mode_printed = mode
            cam_status = computer._camera_status()
            frames_written = computer._writer.frames_written()
            frames_dropped = computer._writer.frames_dropped()
            delta = frames_written - prev_frames_written
            prev_frames_written = frames_written
            file_count, _bytes, _ext, current_paths = _scan_output(out_dir)
            all_files_seen.update(current_paths)
            ssd = poll_stats["last_ssd_free_gb"]
            reply_ms = poll_stats["last_reply_ms"]
            ssd_str = f"{ssd:>3}GB" if ssd is not None else "  ?GB"
            reply_str = f"{reply_ms:.0f}ms" if reply_ms is not None else "--"
            miss_flag = "  MISS!" if status is None else ""

            print(f"{mt_str(t)}  {mode.name:4s} cam={cam_status.name:13s} "
                  f"frames={frames_written:5d} (+{delta:<3d}) dropped={frames_dropped:<3d} "
                  f"files={file_count:<3d} ssd={ssd_str} "
                  f"poll#{poll_stats['sent']:<4d} reply={reply_str}{miss_flag}")

        time.sleep(min(poll_period_s, sensor_period_s) / 4)


# --- Runner ------------------------------------------------------------

def _mode_fps(frame_samples: list[tuple[float, "protocol.SoftwareMode", int]]
              ) -> dict:
    """Observed frames/sec per mode, from consecutive (sim_t, mode,
    cumulative_frames_written) samples taken once per simulated second.
    The mode credited to each interval is the mode at the START of the
    interval, since that is what was actually driving the camera during
    it."""
    frame_time: dict = {}
    frame_count: dict = {}
    for (t0, mode0, f0), (t1, _mode1, f1) in zip(frame_samples, frame_samples[1:]):
        dt = t1 - t0
        df = f1 - f0
        if dt <= 0:
            continue
        frame_time[mode0] = frame_time.get(mode0, 0.0) + dt
        frame_count[mode0] = frame_count.get(mode0, 0) + df
    return {m: frame_count[m] / frame_time[m] for m in frame_time if frame_time[m] > 0}


def _print_summary(computer, out_dir: str, poll_stats: dict, mode_sequence: list,
                    cam_status: protocol.CameraStatus, ssd_free_gb: int,
                    command_stats: dict, all_files_seen: set,
                    frame_samples: list) -> None:
    writer = computer._writer

    print("\n=== Mission simulator summary ===")
    print(f"Polls sent: {poll_stats['sent']}   answered: {poll_stats['answered']}   missed: {poll_stats['misses']}")
    print(f"Frames written: {writer.frames_written()}   Frames dropped: {writer.frames_dropped()}")
    print(f"Mode sequence: {' -> '.join(m.name for m in mode_sequence)}")
    print(f"Final camera status (just before shutdown): {cam_status.name}")
    print(f"Final SSD free (just before shutdown): {ssd_free_gb} GB")

    # --- Observed frame rate per mode ------------------------------------
    fps_by_mode = _mode_fps(frame_samples)
    print("\nObserved frame rate per mode:")
    for mode, fps in sorted(fps_by_mode.items(), key=lambda kv: kv[0].value):
        print(f"  {mode.name:5s} {fps:6.2f} fps")

    # --- Bytes written, split by stream -----------------------------------
    # Video's cumulative total is exact regardless of WIPE, since every
    # synthetic frame is a fixed FakePicamera2.FRAME_BYTES and
    # frames_written() never resets. wipe() reports only one combined
    # CSV+video total (see the WIPE section below), so the CSV figure here
    # is what's currently on disk, not the full-run cumulative total.
    _final_count, _final_total, final_bytes_by_ext, _final_paths = _scan_output(out_dir)
    csv_on_disk = final_bytes_by_ext.get(".csv", 0)
    video_on_disk = final_bytes_by_ext.get(".yuv", 0) + final_bytes_by_ext.get(".txt", 0)
    video_total_written = writer.frames_written() * FakePicamera2.FRAME_BYTES
    print("\nBytes written:")
    print(f"  Video: {video_total_written} bytes cumulative (exact; {video_on_disk} bytes currently on disk)")
    print(f"  CSV:   {csv_on_disk} bytes currently on disk "
          f"(WIPE freed {command_stats.get('wipe_freed_bytes', 0)} bytes across CSV+video combined - see WIPE below)")

    # --- Rollovers per stream ----------------------------------------------
    rollovers = _rollover_counts(all_files_seen)
    print("\nFile rollovers per stream (distinct files ever created, minus the first):")
    for stream, n in rollovers.items():
        print(f"  {stream:16s} {n}")

    # --- WIPE ----------------------------------------------------------------
    print("\nWIPE:")
    print(f"  freed:  {command_stats.get('wipe_freed_bytes', 0)} bytes")
    print(f"  files:  {command_stats.get('wipe_files_before', '?')} -> {command_stats.get('wipe_files_after', '?')}")
    print(f"  result: {command_stats.get('wipe_result')}")
    # StorageManager._newest_group() protects only the single lexically-last
    # filename per directory, not one file per logger prefix. csv/ holds
    # four loggers at once, so wipe() will *attempt* os.remove() on the
    # sensors/system_state/telemetry_mirror files those loggers still have
    # open (uplink_commands is separately exempted by name). This count is
    # how many of those attempts os.remove() itself refused (caught as
    # OSError, e.g. Windows' file-locking) rather than the walk skipping
    # them deliberately - see the note printed below if it is nonzero.
    wipe_errors = computer._storage._wipe_errors
    print(f"  os.remove() failures during WIPE (see note below if nonzero): {wipe_errors}")
    print(f"TEST result: {command_stats.get('test_result')}")

    # --- Poll round-trip time -----------------------------------------------
    if poll_stats["reply_ms_count"] > 0:
        mean_ms = poll_stats["reply_ms_sum"] / poll_stats["reply_ms_count"]
        print(f"\nPoll round-trip time: min={poll_stats['min_reply_ms']:.1f}ms "
              f"max={poll_stats['max_reply_ms']:.1f}ms mean={mean_ms:.1f}ms")

    # --- PASS/FAIL ------------------------------------------------------------
    expected_modes = [protocol.SoftwareMode.MODE_SU, protocol.SoftwareMode.MODE_NG,
                       protocol.SoftwareMode.MODE_NF, protocol.SoftwareMode.MODE_NE,
                       protocol.SoftwareMode.MODE_NF]
    mode_ok = mode_sequence == expected_modes
    zero_dropped = writer.frames_dropped() == 0
    wipe_freed_ok = command_stats.get("wipe_freed_bytes", 0) > 0
    ne_fps = fps_by_mode.get(protocol.SoftwareMode.MODE_NE, 0.0)
    ne_fps_ok = ne_fps > 30.0

    ne_window_lo = int(SOE_ON_S - LO_S)        # 20 s
    ne_window_hi = int(SOE_OFF_S - LO_S)       # 50 s
    new_video_at_ne = any(
        (m := _VIDEO_RE.match(os.path.basename(p))) and m.group(1) == "TPLUS"
        and ne_window_lo <= int(m.group(2)) < ne_window_hi
        for p in all_files_seen
    )

    all_closed = writer._file is None and all(
        log._file is None for log in (
            computer._sensor_log, computer._state_log,
            computer._telemetry_log, computer._command_log))

    print("\n=== PASS/FAIL ===")
    print(f"[{'PASS' if mode_ok else 'FAIL'}] mode sequence correct "
          f"(got {[m.name for m in mode_sequence]}, expected {[m.name for m in expected_modes]})")
    print(f"[{'PASS' if zero_dropped else 'FAIL'}] zero frames dropped (dropped={writer.frames_dropped()})")
    print(f"[{'PASS' if wipe_freed_ok else 'FAIL'}] WIPE freed space (freed={command_stats.get('wipe_freed_bytes', 0)} bytes)")
    print(f"[{'PASS' if ne_fps_ok else 'FAIL'}] NE frame rate above 30 fps (observed {ne_fps:.2f} fps)")
    print(f"[{'PASS' if new_video_at_ne else 'FAIL'}] a new video file opened at the NE transition")
    print(f"[{'PASS' if all_closed else 'FAIL'}] all files closed at shutdown")

    print("\nGenerated files:")
    for root, _dirs, files in sorted(os.walk(out_dir)):
        for name in sorted(files):
            path = os.path.join(root, name)
            size = os.path.getsize(path)
            rel = os.path.relpath(path, out_dir)
            print(f"  {rel:55s} {size:>10d} bytes")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Standalone STAR-D mission simulator")
    p.add_argument("--speed", type=float, default=1.0,
                    help="time acceleration factor (default 1.0 = full 160 s)")
    p.add_argument("--out", type=str, default="./sim_output",
                    help="scratch output directory (default ./sim_output)")
    p.add_argument("--keep", action="store_true",
                    help="do not clear --out on start")
    p.add_argument("--min-free-gb", type=float, default=None,
                    help="override storage_manager.LOW_SSD_FOR_FLIGHT_GB at runtime")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    out_dir = os.path.abspath(args.out)
    if os.path.exists(out_dir) and not args.keep:
        shutil.rmtree(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    if args.min_free_gb is not None:
        # Patch the module attribute rather than the constant in source:
        # the flight threshold of 100 GB correctly fails self_test() on a
        # bench disk with less free space, holding the Pi in SU forever.
        storage_manager.LOW_SSD_FOR_FLIGHT_GB = args.min_free_gb

    # Scale the loop tick so the 10 Hz / 1 Hz CSV cadences (counted in
    # loop ticks, not wall-clock Hz) stay correct relative to simulated
    # time, which itself advances `speed` times faster than real time.
    stard_main.LOOP_PERIOD_S = 0.01 / args.speed

    loop = LoopbackSerial()

    def picamera_factory():
        return FakePicamera2(speed=args.speed)

    computer = stard_main.SoftwarePiComputer(
        port=loop, data_root=out_dir, picamera_factory=picamera_factory)

    state = {
        "mode": protocol.SoftwareMode.MODE_SU,
        "time_ref": protocol.TimeRef.TIME_UNSYNCED,
        "sods_active": False,
        "degraded": False,
    }
    poll_stats = {
        "answered": 0, "misses": 0, "sent": 0,
        "last_reply_ms": None, "last_ssd_free_gb": None,
        "reply_ms_sum": 0.0, "reply_ms_count": 0,
        "min_reply_ms": float("inf"), "max_reply_ms": 0.0,
    }
    mode_sequence: list = []
    command_stats: dict = {}
    all_files_seen: set = set()
    stop_event = threading.Event()

    # Sampled once per simulated second by the frame-sample watcher below,
    # feeding the per-mode fps computation in the summary.
    frame_samples: list = []

    def _sample_frames(stop_event: threading.Event, speed: float) -> None:
        start = time.monotonic()
        while not stop_event.is_set():
            t = (time.monotonic() - start) * speed
            frame_samples.append((t, computer.effective_mode(), computer._writer.frames_written()))
            time.sleep(1.0 / speed)

    esp_thread = threading.Thread(
        target=esp32_driver,
        args=(loop, computer, out_dir, args.speed, stop_event, state, poll_stats,
              mode_sequence, command_stats, all_files_seen),
        name="esp32-sim", daemon=True)
    sample_thread = threading.Thread(
        target=_sample_frames, args=(stop_event, args.speed),
        name="frame-sampler", daemon=True)

    duration = END_S / args.speed
    print(f"Starting mission simulator: speed={args.speed}x, "
          f"duration={duration:.2f}s real ({END_S:.0f}s simulated), out={out_dir}")

    computer.start()
    esp_thread.start()
    sample_thread.start()

    try:
        computer.run(until=duration)
    finally:
        # Snapshot camera/storage state before shutdown stops the camera -
        # shutdown() would otherwise make every run report CAM_UNKNOWN.
        cam_status = computer._camera_status()
        ssd_free_gb = computer._storage.free_gb()
        stop_event.set()
        esp_thread.join(timeout=5.0)
        sample_thread.join(timeout=2.0)
        computer.shutdown()

    _print_summary(computer, out_dir, poll_stats, mode_sequence, cam_status, ssd_free_gb,
                   command_stats, all_files_seen, frame_samples)


if __name__ == "__main__":
    main()
