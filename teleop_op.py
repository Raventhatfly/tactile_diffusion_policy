# Credit to: https://github.com/unitreerobotics/unitree_mujoco/blob/main/simulate_python/unitree_mujoco.py
import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading
import numpy as np

locker = threading.Lock()


class ElasticBand:

    def __init__(self):
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0, 0, 3])
        self.length = 0
        self.enable = True

    def Advance(self, x, dx):
        """
        Args:
          δx: desired position - current position
          dx: current velocity
        """
        δx = self.point - x
        distance = np.linalg.norm(δx)
        direction = δx / distance
        v = np.dot(dx, direction)
        f = (self.stiffness * (distance - self.length) - self.damping * v) * direction
        return f

    def MujuocoKeyCallback(self, key):
        glfw = mujoco.glfw.glfw
        if key == glfw.KEY_7:
            self.length -= 0.1
        if key == glfw.KEY_8:
            self.length += 0.1
        if key == glfw.KEY_9:
            self.enable = not self.enable

elastic_band = ElasticBand()

def SimulationThread():
    global mj_data, mj_model
    mj_model = mujoco.MjModel.from_xml_path('./models/arx5.xml')
    mj_data = mujoco.MjData(mj_model)

    

    # if config.USE_JOYSTICK:
    #     unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    # if config.PRINT_SCENE_INFORMATION:
    #     unitree.PrintSceneInformation()
    with mujoco.viewer.launch_passive(mj_model, mj_data) as viewer:
        while viewer.is_running():
            step_start = time.perf_counter()

            locker.acquire()
            
            print(mj_data.xfrc_applied)
            print(mj_data.qpos[1])
            print(mj_data.qvel[1])
            mj_data.xfrc_applied[1] = elastic_band.Advance(
                        mj_data.qpos[1], mj_data.qvel[1]
            )
            mujoco.mj_step(mj_model, mj_data)

            locker.release()

            

            time_until_next_step = mj_model.opt.timestep - (
                time.perf_counter() - step_start
            )
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)



# def PhysicsViewerThread():
#     while viewer.is_running():
#         locker.acquire()
#         viewer.sync()
#         locker.release()
#         time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    # viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    # viewer_thread.start()
    sim_thread.start()