import os
import sys

import numpy as np
import pyrealsense2 as rs

ROOT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "arx5-sdk")
ROOT_DIR = os.path.join(ROOT_DIR, "python")
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)
print(ROOT_DIR)
from arx5_interface import Arx5CartesianController, EEFState, Gain

DEFAULT_OBS_KEY_MAP = {
    # robot
    'ActualTCPPose': 'robot_eef_pose',
    'ActualTCPSpeed': 'robot_eef_pose_vel',
    'ActualQ': 'robot_joint',
    'ActualQd': 'robot_joint_vel',
    # timestamps
    'step_idx': 'step_idx',
    'timestamp': 'timestamp'
}
class RealEnv:
    def __init__(self):
        pass
    
    def transform(data):
        pass

    def vis_transform(data):
        pass

    # ======== start-stop API =============
    @property
    def is_ready(self):
        pass
    
    def start(self, wait=True):
        pass

    def stop(self, wait=True):
        pass

    def start_wait(self):
        pass

    def stop_wait(self):
        pass

#     # ========= context manager ===========
    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    # ========= async env API ===========
    def get_obs(self) -> dict:
        return dict()
    
    def exec_actions(self):
        pass

    def get_robot_state(self):
        return self.robot.get_state()

    # recording API
    def start_episode(self, start_time=None):
        pass
    
    def end_episode(self):
        pass

    def drop_episode(self):
        pass