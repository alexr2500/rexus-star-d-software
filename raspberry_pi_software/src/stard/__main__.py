from stard.links import protocol
from stard.camera import camera_control, video_writer

def main() -> None:
    # --- SU: local startup, independent of the link ---------------
    def __init__(self) -> None:
        self._pi_init_complete = False  # remains false even if ESP init is complete

     
    def effective_mode(self) -> protocol.SoftwareMode:
        """SU until local init is done, regardless of what the ESP32 reports."""
        if not self._pi_init_complete:
            return protocol.SoftwareMode.MODE_SU
        return self._link.mode()
    #   create output directories
    #   construct camera (may raise if absent -> set fault, continue)

    cam = 

    #   construct writer, loggers
    #   self-test: SSD writable? free space adequate?

    if (protocol.SoftwareMode == protocol.SoftwareMode.MODE_T):


    # --- start the threads ----------------------------------------
    #   writer.start()
    #   camera.start()
    #   capture thread with the shared stop_event

    # --- main loop -------------------------------------------------
    #   link.service()                    <- protocol first
    #   camera.set_mode(link.mode())      <- react to mode changes
    #   periodic: system state CSV at 10 Hz
    #   periodic: telemetry CSV at 1 Hz
    #   sleep to next tick

    # --- shutdown --------------------------------------------------
    #   stop capture, camera, writer, loggers