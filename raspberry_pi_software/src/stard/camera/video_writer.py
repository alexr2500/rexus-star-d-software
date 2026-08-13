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
from typing import Optional

from stard.camera.camera_control import Frame
from stard.links import protocol

QUEUE_MAXSIZE = 30
ROTATION_S_NE = 30
ROTATION_S_IDLE = 300


class VideoWriter:
    def __init__(self, directory: str) -> None:
        ...

    def submit(self, frame: Frame) -> bool:
        """Called from the capture thread. False if dropped."""
        

    def start(self) -> None:
        ...

    def stop(self) -> None:
        ...

    def _run(self) -> None:
        """The thread body. Loops until stopped."""
        ...