import time
import logging
from core.mission_manager import MissionManager
from storage.ssd_handler import SSDHandler

MODE_SU = "STARTUP"
MODE_T = "TEST"
MODE_NG = "NORMAL GROUND"
MODE_NF = "NORMAL FLIGHT"
MODE_NE = "NORMAL EXPERIMENT"
MODE_FB = "FALLBACK"

current_mode = MODE_SU

"""
STEP 2:
Setup logging properly.
In aerospace projects, logging is everything.
"""

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)


def system_loop():
    global current_mode

    while True:
        if current_mode == MODE_SU:
            logging.info("In STARTUP mode")
            # Later:
            # - check disk
            # - check camera
            # - check SPI link
            current_mode = MODE_NG

        elif current_mode == MODE_NG:
            logging.info("In NORMAL mode")
            # Later:
            # - read SPI state
            # - record metadata
            # - manage video recording

        elif current_mode == MODE_FB:
            logging.warning("In SAFE mode")
            # Later:
            # - stop camera
            # - close files safely

        time.sleep(1)


if __name__ == "__main__":
    logging.info("Raspberry Pi Software Boot")

    # Later:
    # - initialize SPI
    # - initialize camera
    # - initialize storage manager

    system_loop()