"""Camera control for imaging

Raw YUV420 capture, no encoding possible"""

from dataclasses import dataclass
from typing import Optional
from typing import Callable

from picamera2 import Picamera2

from stard.links import protocol

SENSOR_SIZE = (1640, 1232)  # allows the full sensor size. Frame rate is 41.85fps
PIXEL_FORMAT = "YUV420"

# FrameDurationLimits is microseconds per frame. Setting min == max pins
# the rate rather than letting the ISP vary it with exposure.
FRAME_DURATION_NE_US = 23895      # 41.85 fps
FRAME_DURATION_IDLE_US = 250000    # 4 fps

# Buffers in the pool. Each 1640x1232 YUV420 frame is ~3 MB, so this is
# ~18 MB of DMA memory. Too few and a slow write stalls capture; too many
# wastes memory and hides the stall rather than surfacing it.
BUFFER_COUNT = 6

# Frames without SensorTimestamp advancing before the camera is declared
# stalled. At 41.85 fps this is ~0.5 s.
STALL_THRESHOLD_FRAMES = 20


@dataclass
class Frame:
    """One capture frame handed to the writer thread"""
    buffer: bytes
    mission_time_ms: int            # timed at capture because write might take time afterwards
    time_ref: protocol.TimeRef
    sensor_timestamp_ns: int
    frame_count: int               # counter since start()


class CameraControl:
    """Owns the camera. Configures at construction, captures on demand."""

    def __init__(
        self,
        time_source: Callable[[], int],                     # takes no argument and returns an int
        time_ref_source: Callable[[], protocol.TimeRef]
    ) -> None:
        self._current_mode = protocol.SoftwareMode.MODE_SU
        self._time_source = time_source
        self._time_ref_source = time_ref_source

        self._picam = Picamera2()           # accesses the Pi camera constructor
        self._config = self._picam.create_video_configuration(
            main={"size": SENSOR_SIZE, "format": PIXEL_FORMAT},
            controls={"FrameDurationLimits": (FRAME_DURATION_IDLE_US,
                                              FRAME_DURATION_IDLE_US)},
            buffer_count=BUFFER_COUNT,
        )
        self._picam.configure(self._config)     # applies the settings written above; configure the cam based on the defined constants

        self._running = False                   # initialize the variables
        self._frame_number = 0
        self._last_sensor_timestamp_ns = 0
        self._frames_without_advance = 0


    def start(self) -> None:
        """Begin capturing. Idempotent."""      # means that calling it twice does nothing more than the first time
        if self._running:
            return
        self._picam.start()
        self._running = True

    def stop(self) -> None:
        """Stop capturing. Idempotent, and safe during shutdown."""
        if not self._running:
            return
        self._picam.stop()
        self._running = False


    def set_frame_rate(self, duration_us: int) -> None:
        """Change the frame interval. Safe to call while running."""
        self._picam.set_controls({"FrameDurationLimits": (duration_us,
                                                          duration_us)})


    def capture(self) -> Optional[Frame]:
        if self._running == False:
            return None
        request = self._picam.capture_request()         # blocks until a frame is available. Can take up to 250ms, hence the separate camera thread. This takes one buffer count frome the constant
        try:
            buf = request.make_buffer("main")           # pixel bytes
            metadata = request.get_metadata()
            sensor_ts = metadata.get("SensorTimestamp", 0)
            if (sensor_ts == self._last_sensor_timestamp_ns):
                self._frames_without_advance += 1               # increments if frames stalling
            else:
                self._frames_without_advance = 0
            self._last_sensor_timestamp_ns = sensor_ts
            self._frame_number += 1
            return Frame(buffer=buf, mission_time_ms=self._time_source(), time_ref=self._time_ref_source(), sensor_timestamp_ns=sensor_ts, frame_count=self._frame_number)   # returns a constructed frame
        finally:
            request.release()   # releases the buffer count block


    def status(self) -> protocol.CameraStatus:
        if (self._frame_number == 0 or self._running == False):
            return protocol.CameraStatus.CAM_UNKNOWN
        elif (self._frames_without_advance >= STALL_THRESHOLD_FRAMES):
            return protocol.CameraStatus.CAM_STALL
        else:
            return protocol.CameraStatus.CAM_OK


    def set_mode(self, mode: protocol.SoftwareMode) -> None:
        """Full frame rate during the experiment, idle rate otherwise."""
        if self._current_mode == mode:
            return
        self._current_mode = mode
        if mode == protocol.SoftwareMode.MODE_NE:
            self.set_frame_rate(FRAME_DURATION_NE_US)
        else:
            self.set_frame_rate(FRAME_DURATION_IDLE_US)