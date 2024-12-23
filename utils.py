import mujoco
import numpy as np

class RingBuffer:
    def __init__(self,capacity):
        self.capacity = capacity
        self.buffer = [None] * capacity
        self.head = 0
        self.size = 0
    
    def append(self,val):
        self.buffer[self.head] = val
        self.head = (self.head + 1) % self.capacity
        if self.size < self.capacity:
            self.size += 1
    
    def get_mean(self):
        sum = 0
        for i in range(self.size):
            sum += self.buffer[i]
        return sum / self.size
    
def get_compensation(model,data,kt):
    buffer = RingBuffer(10)
    orig_qacc = data.qacc.copy()
    data.qacc[:] = 0
    mujoco.mj_inverse(model,data)
    buffer.append(data.qfrc_inverse)
    data.qacc = orig_qacc.copy()
    return buffer.get_mean() / kt