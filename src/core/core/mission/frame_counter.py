import time

class FrameCounter():
    """
    Checking against YOLO Object Detection False Positive
    """
    def __init__(self, frame_amount):
        self.amount = frame_amount
        self.flag = 0
        self.counter = 0

    def reset(self):
        self.flag = 0
        self.counter = 0

    def is_enough(self):
        if self.flag == 1:
            if time.time() - self.counter >= self.amount:
                self.reset()
                return True
        return False

    def is_started(self):
        if self.flag == 1:
            return
        self.flag = 1
        self.counter = time.time()

    def get(self):
        return self.counter
