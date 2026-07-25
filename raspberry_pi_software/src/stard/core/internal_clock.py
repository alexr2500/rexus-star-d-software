import time

class ModeHandler:
    def __init__(self):
        self.start_time = None                                          #INITIALIZE CLOCK, T = 0

    def start_mission_clock(self):
        self.start_time = time.monotonic()                              #LO RECEIVED, BEGIN CLOCK

    def get_mission_time(self):                                         #TIME SINCE LO
        if self.start_time is None:
            return 0.0
        return time.monotonic() - self.start_time

#monotic is forward-only and time cannot be adjusted forwards or backwards in any way