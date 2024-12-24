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
    
    def is_full(self):
        if self.capacity == self.size:
            return True
        else:
            return False

buffer = RingBuffer(10) 

def get_compensation(model,data,kt):
    orig_qacc = data.qacc.copy()
    data.qacc[:] = 0
    mujoco.mj_inverse(model,data)
    buffer.append(data.qfrc_inverse.copy())
    data.qacc = orig_qacc.copy()
    compensation = buffer.get_mean()
    
    if buffer.is_full():
        compensation = compensation / kt
    else:
        compensation = np.zeros(compensation.size)

    return compensation


    