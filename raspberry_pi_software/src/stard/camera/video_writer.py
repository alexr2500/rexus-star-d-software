"""Video writer thread.

Consumes frames from a bounded queue and writes raw YUV420 to disk on its
own thread, so that a slow fsync cannot stall capture or the UART link.

When the queue is full, frames are dropped and counted rather than
blocking: blocking would stall the sensor pipeline, losing more frames
than it saves and masquerading as a camera fault.
"""

import os
import queue
import threading
from typing import IO

from stard.links import protocol

from stard.camera.camera_control import Frame, SENSOR_SIZE, PIXEL_FORMAT

QUEUE_MAXSIZE = 30
_SENTINEL = None    # pushed to wake the writer thread at shutdown
JOIN_TIMEOUT_S = 7.0

_TIME_REF_TAG = {
    protocol.TimeRef.TIME_UNSYNCED: "UNSYNCED",
    protocol.TimeRef.TIME_COUNTDOWN: "TMINUS",
    protocol.TimeRef.TIME_MISSION: "TPLUS",
}


class VideoWriter:
    def __init__(self, directory: str) -> None:
        self._directory = directory
        os.makedirs(directory, exist_ok=True)

        # The two objects shared across threads. Everything else below is
        # touched by exactly one thread.
        self._queue: queue.Queue[Frame | None] = queue.Queue(maxsize=QUEUE_MAXSIZE)
        self._stop_event = threading.Event()

        # Owned by the main thread.
        self._thread: threading.Thread | None = None
        self._frames_dropped = 0
        self._shutdown_timed_out = False

        # Owned by the writer thread, once it starts.
        self._file: IO[bytes] | None = None
        self._frames_in_file = 0
        self._frames_written = 0


    def submit(self, frame: Frame) -> bool:
        """Hand a frame to the writer thread.

        Returns False if the queue was full and the frame was dropped.
        Never blocks: blocking here would stall the sensor pipeline.
        """
        try:
            self._queue.put_nowait(frame)
            return True
        except queue.Full:
            self._frames_dropped += 1
            return False
        

    def frames_dropped(self) -> int:
        return self._frames_dropped


    def frames_written(self) -> int:
        return self._frames_written


    def start(self) -> None:
        if (self._thread is not None and self._thread.is_alive()):
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="imaging-writer")
        self._thread.start()

    def stop(self) -> None:
        if (self._thread is None):
            return
        self._stop_event.set()
        try:
            self._queue.put(_SENTINEL, timeout=1.0)
        except queue.Full:
            pass
        self._thread.join(timeout=JOIN_TIMEOUT_S)
        if self._thread.is_alive():
            self._shutdown_timed_out = True
        self._thread = None


    def _run(self) -> None:
        """The thread body. Loops until stopped or the sentinel arrives."""
        try:
            while not self._stop_event.is_set():
                try:
                    item = self._queue.get(timeout=0.5)
                except queue.Empty:
                    continue          # nothing arrived; re-check the stop flag

                if item is _SENTINEL:
                    break

                self._write_frame(item)
        finally:
            self._close_file()


    def shutdown_timed_out(self) -> bool:
        """True if the writer thread did not finish within the join timeout."""
        return self._shutdown_timed_out

    # TODO: add this flag as `VIDEO_SHUTDOWN_INCOMPLETE` in the status/error flags

    def _write_frame(self, frame: Frame) -> None:
        if self._file is None or self._frames_in_file >= frame.rotation_limit:
            self._open_new_file(frame)

        if self._file is None:
            raise RuntimeError("video file not open after rollover")

        self._file.write(frame.buffer)
        self._frames_in_file += 1
        self._frames_written += 1

    # TODO: write a try/except for CAM_WRITE_FAIL

    def _filename(self, frame: Frame) -> str:
        """Encode the time reference so a post-flight reader can tell a
        scrubbed countdown from the real flight at a glance.

        File is named from when the capture began (at file creation), not at frame capture
        """
        tag = _TIME_REF_TAG[frame.time_ref]
        seconds = abs(frame.mission_time_ms) // 1000
        return os.path.join(
            self._directory, f"video_{tag}_{seconds:07d}.yuv"
        )
    

    def _open_new_file(self, frame: Frame) -> None:
        """Close the current file and start a new one.

        A mode change mid-file causes an immediate rollover, because the
        new frame carries a smaller rotation_limit. That is intentional:
        it gives a clean file boundary at the NE transition.
        """
        self._close_file()
        self._file = open(self._filename(frame), "wb")
        self._frames_in_file = 0
        self._write_sidecar(frame)


    def _close_file(self) -> None:
        """Flush to the physical device and close. Safe to call twice."""
        if self._file is None:
            return
        self._file.flush()
        os.fsync(self._file.fileno())
        self._file.close()
        self._file = None


    def _write_sidecar(self, frame: Frame) -> None:
        """Raw YUV has no header. Without these values the file cannot
        be decoded, so write them alongside it."""
        path = self._filename(frame).replace(".yuv", ".txt")
        with open(path, "w") as f:
            f.write(f"width={SENSOR_SIZE[0]}\n")
            f.write(f"height={SENSOR_SIZE[1]}\n")
            f.write(f"format={PIXEL_FORMAT}\n")
            f.write(f"first_frame_number={frame.frame_count}\n")
            f.write(f"first_mission_time_ms={frame.mission_time_ms}\n")
            f.write(f"time_ref={frame.time_ref.name}\n")
            f.write(f"rotation_limit={frame.rotation_limit}\n")