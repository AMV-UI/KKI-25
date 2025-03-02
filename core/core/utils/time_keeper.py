import time 

class TimeKeeper():
    """
    TimeKeeper is a class that keeps track of the time.

    Args:
        delay (float): The delay time.
        finish (float): The finish time.
    """
    def __init__(self, delay, finish):
        self.delay = delay
        self.finish = finish
        self.time_start = None

    def debug(self):
        print("diff:", time.time() - self.time_start)
        print(f"{self.time_start= }")
        print("bool:", self.check_finish())

    def set_time(self):
        self.time_start = time.time()

    def reset(self):
        self.time_start = None

    def is_started(self):
        if self.time_start == None:
            return False
        return True

    def get_time(self):
        return self.time_start
        
    def check_delay(self):
        if self.time_start == None:
            return False

        return time.time() - self.time_start > self.delay

    def check_finish(self):
        if self.time_start == None:
            return False

        return (time.time() - self.time_start) > self.finish
