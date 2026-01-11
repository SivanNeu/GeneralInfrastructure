import time


class MPS:
    def __init__(self, topic, monitorTime = 60):
        self.tic = time.monotonic()
        self.cnt = 0.0
        self.topic = topic
        self.monitorTime = monitorTime

    def calcMPS(self):
        self.cnt += 1
        if time.monotonic() - self.tic >= self.monitorTime:
            mps = self.cnt/(time.monotonic() - self.tic)
            print('%s message MPS: %0.2f'%(self.topic, mps))
            self.cnt = 0.0
            self.tic = time.monotonic()
