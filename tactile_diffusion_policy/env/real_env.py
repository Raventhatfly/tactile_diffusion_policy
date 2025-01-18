import os
import sys
import time
from threading import Thread, Event, Lock
import numpy as np
import torch
import pyrealsense2 as rs

curr_dir = os.getcwd()
ROOT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "arx5-sdk")
ROOT_DIR = os.path.join(ROOT_DIR, "python")
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
from arx5_interface import Arx5CartesianController, EEFState, Gain

os.chdir(curr_dir)

# class DoubleBuffer():
#     def __init__(self):
#         self.frame = list([None,None])
#         self.curr_frame = 0
#         self.lock = Lock()
    
#     def write_buffer(self,frame):
#         self.frame[self.curr_frame] = frame
#         # Do not switch frame during read operation
#         self.lock.acquire()
#         self.curr_frame = 1 - self.curr_frame
#         self.lock.release()

#     def read_buffer(self):
#         self.lock.acquire()
#         ret_frame = self.frame[1 - self.curr_frame]
#         self.lock.release()
#         return ret_frame

class DoubleBufferedRingBuffer:
    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer1 = [None] * capacity
        self.buffer2 = [None] * capacity
        self.read_buffer = self.buffer1
        self.write_buffer = self.buffer2
        self.head = 0  # Index for the next write
        self.tail = 0  # Index for the next read
        self.size = 0  # Current number of elements in the buffer
        self.lock = Lock()  # Lock for buffer swapping

    def write(self, item):
        with self.lock:
            if self.head == self.capacity:
                # Swap buffers
                self.read_buffer, self.write_buffer = self.write_buffer, self.read_buffer
                self.head = 0
            self.write_buffer[self.head] = item
            self.head += 1
            if self.size < self.capacity:
                self.size += 1

    def read(self):
        with self.lock:
            if self.tail == self.capacity:
                # Swap buffers
                self.read_buffer, self.write_buffer = self.write_buffer, self.read_buffer
                self.tail = 0
            item = self.read_buffer[self.tail]
            self.tail += 1
            return item
        
    def get_latest_k(self, k):
        with self.lock:
            if k <= 0:
                return []
            if k > self.size:
                k = self.size  # Adjust k if it's larger than the buffer size

            # Start from the most recent element (head - 1) and work backward
            latest_values = []
            current_index = (self.head - 1) % self.capacity  # Start at the last written element
            for _ in range(k):
                latest_values.append(self.read_buffer[current_index])
                current_index = (current_index - 1) % self.capacity  # Move backward
            return latest_values[::-1]  # Reverse to return in chronological order

    def __str__(self):
        with self.lock:
            return str(self.read_buffer)

    
class RealEnv:
    def __init__(self, nobs_step = 2):
        np.set_printoptions(precision=3, suppress=True)
        buffer_capacity = 10
        self.freq = 10

        self.nobs_step = nobs_step
        interface = "can1"

        urdf = os.path.join(ROOT_DIR, "../models/arx5.urdf")
        self.controller = Arx5CartesianController("L5", interface, urdf)
        self.robot_config = self.controller.get_robot_config()
        self.controller_config = self.controller.get_controller_config()
    
        # Camera 1 Setup
        cam1_serial_number = '230322277180'
        self.pipeline_1 = rs.pipeline()
        self.config_1 = rs.config()
        self.config_1.enable_device(cam1_serial_number)
        self.config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config_1.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

        # self.buffer1 = DoubleBuffer()
        self.buffer1 = DoubleBufferedRingBuffer(buffer_capacity)

        # Camera 2 Setup
        cam2_serial_number = '230322271473'
        self.pipeline_2 = rs.pipeline()
        self.config_2 = rs.config()
        self.config_2.enable_device(cam2_serial_number)
        self.config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config_2.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

        # self.buffer2 = DoubleBuffer()
        self.buffer2 = DoubleBufferedRingBuffer(buffer_capacity)

        # Arm Setup
        # self.buffer_arm = DoubleBuffer()
        self.buffer_arm = DoubleBufferedRingBuffer(buffer_capacity)

        # Finish Event Setup
        self.finish_event = Event()
    
    def transform(data):
        pass

    def vis_transform(data):
        pass

    # ======== start-stop API =============
    @property
    def is_ready(self):
        pass
    
    def start(self, wait=True):
        self.controller.reset_to_home()
        self.pipeline_1.start(self.config_1)
        self.pipeline_2.start(self.config_2)
        self.t1 = Thread(target=self.camera1_thread, args=(self.finish_event,))
        self.t2 = Thread(target=self.camera2_thread, args=(self.finish_event,))
        self.t3 = Thread(target=self.arm_thread, args=(self.finish_event,))
        self.t1.start()
        self.t2.start()
        self.t3.start()
        time.sleep(2)   # Waiting for the camera to start streaming


    def stop(self, wait=True):
        self.finish_event.set()
        self.t1.join()
        self.t2.join()
        self.t3.join()
        self.controller.reset_to_home()  

    # def start_wait(self):
    #     pass

    # def stop_wait(self):
    #     pass

    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= async env API ===========
    def get_obs(self) -> dict:
        obs = dict()
        obs["img1"] = np.array(self.buffer1.get_latest_k(self.nobs_step))
        obs["img2"] = np.array(self.buffer2.get_latest_k(self.nobs_step))
        # TODO: more modality to be added
        # obs["torque"] = ???
        joint_state = self.buffer_arm.get_latest_k(self.nobs_step)
        joint_pos_obs = []
        for single_obs in joint_state:
            joint_pos_obs.append(np.hstack((single_obs.pos(),single_obs.gripper_pos)))
        obs["joint_pos"] = torch.Tensor(np.array(joint_pos_obs))
        return obs
    
    def exec_actions(self, pose: np.ndarray, dt:int):
        # eef_state_init = self.controller.get_eef_state()
        # Do I need to implement linear interplocation here?
        eef_state = EEFState()
        eef_state.pose6d = pose[:6]
        eef_state.gripper_pos = pose[7]
        eef_state.gripper_vel = 0.0
        eef_state.gripper_torque = 0.0
        eef_state.timestamp = time.time()
        self.controller.set_eef_cmd(eef_state)

    # def get_robot_state(self):
    #     return self.robot.get_state()

    # recording API
    def start_episode(self, start_time=None):
        pass
    
    def end_episode(self):
        pass

    def drop_episode(self):
        pass

    # ========= Recording Thread ===========
    def camera1_thread(self, finish_event):
        while not finish_event.is_set():
            start_time = time.perf_counter()
            frames_1 = self.pipeline_1.wait_for_frames()
            depth_frame_1 = frames_1.get_depth_frame()
            color_frame_1 = frames_1.get_color_frame()
            # self.buffer1.write(np.asanyarray(color_frame_1.get_data()))
            self.buffer1.write(np.asanyarray(color_frame_1.get_data()))
            # print("Hello")
            # print(self.buffer1)
            end_time = time.perf_counter()
            # if(end_time - start_time < 1/self.freq):
            #     time.sleep(1/self.freq - (end_time-start_time))

    def camera2_thread(self, finish_event):
        while not finish_event.is_set():  
            start_time = time.perf_counter()
            frames_2 = self.pipeline_2.wait_for_frames()
            self.depth_frame_2 = frames_2.get_depth_frame()
            self.color_frame_2 = frames_2.get_color_frame()
            self.buffer2.write(np.asanyarray(self.color_frame_2.get_data()))
            end_time = time.perf_counter()
            # if(end_time - start_time < 1/self.freq):
            #     time.sleep(1/self.freq - (end_time-start_time))

    def arm_thread(self, finish_event):
        while not finish_event.is_set():
            start_time = time.perf_counter()
            eef_state = self.controller.get_eef_state()
            joint_state = self.controller.get_joint_state()
            self.buffer_arm.write(joint_state)
            end_time = time.perf_counter()
            # if(end_time - start_time < 1/self.freq):
            #     time.sleep(1/self.freq - (end_time-start_time))

    