import os
import sys
import time
from threading import Thread, Event, Lock
import numpy as np
import pyrealsense2 as rs

ROOT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "arx5-sdk")
ROOT_DIR = os.path.join(ROOT_DIR, "python")
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
print(ROOT_DIR)
from arx5_interface import Arx5CartesianController, EEFState, Gain

class DoubleBuffer():
    def __init__(self):
        self.frame = dict(2)
        self.curr_frame = 0
        self.lock = Lock()
    
    def write_buffer(self,frame):
        self.frame[self.curr_frame] = frame
        # Do not switch frame during read operation
        self.lock.acquire()
        self.curr_frame = 1 - self.curr_frame
        self.lock.release()

    def read_buffer(self):
        self.lock.acquire()
        ret_frame = self.frame(1 - self.curr_frame)
        self.lock.release()
        return ret_frame

class RealEnv:
    def __init__(self):
        np.set_printoptions(precision=3, suppress=True)
        interface = "can0"
        urdf = "../models/arx5.urdf"
        self.controller = arx5.Arx5CartesianController("L5", interface, urdf)
        
        self.robot_config = self.controller.get_robot_config()
        self.controller_config = self.controller.get_controller_config()
    
        # Camera 1 Setup
        cam1_serial_number = '230322277180'
        self.pipeline_1 = rs.pipeline()
        self.config_1 = rs.config()
        self.config_1.enable_device(cam1_serial_number)
        self.config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config_1.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

        # Camera 2 Setup
        cam2_serial_number = '230322271473'
        self.pipeline_2 = rs.pipeline()
        self.config_2 = rs.config()
        self.config_2.enable_device(cam2_serial_number)
        self.config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config_2.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

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
        self.pipeline_1.start(self.config_1)
        self.pipeline_2.start(self.config_2)
        self.t1 = Thread(target=self.camera1_thread, args=(self.finish_event,))
        self.t2 = Thread(target=self.camera2_thread, args=(self.finish_event,))
        self.t1.start()
        self.t2.start()


    def stop(self, wait=True):
        self.finish_event.set()
        self.t1.join()
        self.t2.join()

    def start_wait(self):
        pass

    def stop_wait(self):
        pass

    # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= async env API ===========
    def get_obs(self) -> dict:
        return dict()
    
    def exec_actions(self, pose: np.ndarray):
        eef_state = EEFState
        eef_state.pose6d = pose[:6]
        eef_state.gripper_pos = pose[7]
        eef_state.gripper_vel = 0.0
        eef_state.gripper_torque = 0.0
        eef_state.timestamp = time.time()
        self.controller.set_eef_cmd(eef_state)

    def get_robot_state(self):
        return self.robot.get_state()

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
            self.frames_1 = self.pipeline_1.wait_for_frames()
            self.depth_frame_1 = self.frames_1.get_depth_frame()
            self.color_frame_1 = self.frames_1.get_color_frame()

    def camera2_thread(self, finish_event):
        while not finish_event.is_set():  
            self.frames_2 = self.pipeline_2.wait_for_frames()
            self.depth_frame_2 = self.frames_2.get_depth_frame()
            self.color_frame_2 = self.frames_2.get_color_frame()
    