from typing import Optional
import pathlib
import numpy as np
import time
import shutil
import math
from multiprocessing.managers import SharedMemoryManager
from tactile_diffusion_policy.real_world.rtde_interpolation_controller import RTDEInterpolationController
from tactile_diffusion_policy.real_world.multi_realsense import MultiRealsense, SingleRealsense
from tactile_diffusion_policy.real_world.video_recorder import VideoRecorder
from tactile_diffusion_policy.common.timestamp_accumulator import (
    TimestampObsAccumulator, 
    TimestampActionAccumulator,
    align_timestamps
)
from tactile_diffusion_policy.real_world.multi_camera_visualizer import MultiCameraVisualizer
from tactile_diffusion_policy.common.replay_buffer import ReplayBuffer
from tactile_diffusion_policy.common.cv2_util import (
    get_image_transform, optimal_row_cols)

def main():

    if shm_manager is None:
            shm_manager = SharedMemoryManager()
            shm_manager.start()
    if camera_serial_numbers is None:
        camera_serial_numbers = SingleRealsense.get_connected_devices_serial()
    
    video_capture_resolution = (480,640)
    video_capture_fps = 30
    recording_fps = 30
    thread_per_video = 1
    # I don't what's all of these parameters are for?

    recording_pix_fmt = 'bgr24'
    record_raw_video = True
    frequency = 10
    transform = None
    video_crf=21,
    if not record_raw_video:
        recording_transfrom = transform
        recording_fps = frequency
        recording_pix_fmt = 'rgb24'
    max_obs_buffer_size = 30

    video_recorder = VideoRecorder.create_h264(
            fps=recording_fps, 
            codec='h264',
            input_pix_fmt=recording_pix_fmt, 
            crf=video_crf,
            thread_type='FRAME',
            thread_count=thread_per_video)
    realsense = MultiRealsense(
                serial_numbers=camera_serial_numbers,
                shm_manager=shm_manager,
                resolution=video_capture_resolution,
                capture_fps=video_capture_fps,
                put_fps=video_capture_fps,
                # send every frame immediately after arrival
                # ignores put_fps
                put_downsample=False,
                record_fps=recording_fps,
                enable_color=True,
                enable_depth=False,
                enable_infrared=False,
                get_max_k=max_obs_buffer_size,
                transform=transform,
                vis_transform=vis_transform,
                recording_transform=recording_transfrom,
                video_recorder=video_recorder,
                verbose=False
                )
    
    realsense.start()


if __name__ == "__main__":
    main()