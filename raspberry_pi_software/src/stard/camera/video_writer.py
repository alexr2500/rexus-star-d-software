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
from typing import IO, Optional

from stard.camera.camera_control import Frame
from stard.links import protocol

QUEUE_MAXSIZE = 30
ROTATION_S_NE = 30
ROTATION_S_IDLE = 300
_SENTINEL = None    # pushed to wake the writer thread at shutdown
JOIN_TIMEOUT_S = 7.0


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
        self._thread.join(timeout=JOIN_TIMEOUT_S)
        if (self._thread.is_alive()):
            print("Thread stalled")
        self._thread = None

    def _run(self) -> None:
        """The thread body. Loops until stopped."""