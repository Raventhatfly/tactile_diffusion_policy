import time

import mujoco
import mujoco.viewer
from gymnasium.envs.mujoco.mujoco_env import MujocoEnv
from motor_controller import MotorControlConfig, MotorController
from utils import get_compensation
import matplotlib.pyplot as plt

from threading import Thread
import threading

from pynput import keyboard

dest = [0,0,0,0,0,0,0.01,0.01]
dest = [0.5,0.5,0.5,0.5,0.5,0.5, 0.01, 0.01]

def simulationThread():
  m = mujoco.MjModel.from_xml_path('./models/R5a/urdf/R5a.xml')
  d = mujoco.MjData(m)
  # d.qfrc_applied[6] = 0
  # d.qfrc_applied[7] = 0

  print(m.nv)

  i = 0
  with mujoco.viewer.launch_passive(m, d) as viewer:
    # Close the viewer automatically after 30 wall-seconds.

    cfg = MotorControlConfig()
    controller = MotorController(cfg)
    controllers = []
    
    for i in range(8):
      cfg = MotorControlConfig()
      controller = MotorController(cfg)
      cfg.kt = 0.2
      match i:
        case 0:
          cfg.kp = 10000
          cfg.kd = 500
        case 1:
          cfg.kp = 200
          cfg.kd = 50
        case 2:
          cfg.kp = 200
          cfg.kd = 30
        case 3:
          cfg.kp = 100
          cfg.kd = 10
        case 4:
          cfg.kp = 30
          cfg.kd = 5
        case 5:
          cfg.kp = 20
          cfg.kd = 2
        case 6:
          cfg.kp = 500
          cfg.kd = 50
        case 7:
          cfg.kp = 500
          cfg.kd = 50
      controllers.append(controller)

    data = []
    t = []
    torque = []
    cnt = 0


    num_step = 1
    step_cnt = 0

    while viewer.is_running():
      step_start = time.time()

      # mj_step can be replaced with code that also evaluates
      # a policy and applies a control signal before stepping the physics.
      mujoco.mj_step(m, d)
      # print(d.qpos[6])
      # data.append(d.qpos[6])
      # t.append(cnt)
      # cnt += 1

      viewer.sync()

      compensation = get_compensation(m,d,controllers[0].param.kt)
        
      # compensation = controllers[0].get_compensation(d.qfrc_applied[0])
      d.qfrc_applied[0] = controllers[0].output(d.qpos[0],d.qvel[0], dest[0] * step_cnt / num_step,0.0,compensation[0])
      
      # compensation = controllers[1].get_compensation(d.qfrc_applied[1])
      d.qfrc_applied[1] = controllers[1].output(d.qpos[1],d.qvel[1], dest[1] * step_cnt / num_step,0.0,compensation[1])
      # print(d.qpos[1])
      # data.append(d.qpos[1])
      # t.append(cnt)
      # cnt += 1

      # d.qfrc_applied[6] = 0
      # d.qfrc_applied[7] = 0
      # compensation = controllers[2].get_compensation(d.qfrc_applied[2])
      d.qfrc_applied[2] = controllers[2].output(d.qpos[2],d.qvel[2], dest[2] * step_cnt / num_step,0,compensation[2])

      d.qfrc_applied[3] = controllers[3].output(d.qpos[3],d.qvel[3], dest[3] * step_cnt / num_step,0,compensation[3])

      d.qfrc_applied[4] = controllers[4].output(d.qpos[4],d.qvel[4], dest[4] * step_cnt / num_step,0,compensation[4])

      d.qfrc_applied[5] = controllers[5].output(d.qpos[5],d.qvel[5], dest[5] * step_cnt / num_step,0,compensation[5])

      d.qfrc_applied[6] = controllers[6].output(d.qpos[6],d.qvel[6], dest[6] * step_cnt / num_step,0,compensation[6])

      # print(str(compensation[6]) + " " + str(d.qfrc_applied[6]))

      d.qfrc_applied[7] = controllers[7].output(d.qpos[7],d.qvel[7], dest[7] * step_cnt / num_step,0,compensation[7])

      # d.qfrc_applied[6] = 0
      # d.qfrc_applied[7] = 0

      # print(d.qpos[7])
      # data.append(d.qpos[7])
      # print(d.qfrc_applied[6])
      print(d.qpos[6])
      data.append(d.qpos[6])
      t.append(cnt)
      cnt += 1

      time_until_next_step = m.opt.timestep - (time.time() - step_start)
      if time_until_next_step > 0:
        time.sleep(time_until_next_step)
      
      if step_cnt < num_step:
        step_cnt += 1
    plt.plot(t,data)
    plt.show()
    
    



def keyThread():
  key_pressed = {
        keyboard.Key.up: False,  # +x
        keyboard.Key.down: False,  # -x
        keyboard.Key.left: False,  # +y
        keyboard.Key.right: False,  # -y
        keyboard.Key.page_up: False,  # +z
        keyboard.Key.page_down: False,  # -z
        keyboard.KeyCode.from_char("q"): False,  # +roll
        keyboard.KeyCode.from_char("a"): False,  # -roll
        keyboard.KeyCode.from_char("w"): False,  # +pitch
        keyboard.KeyCode.from_char("s"): False,  # -pitch
        keyboard.KeyCode.from_char("e"): False,  # +yaw
        keyboard.KeyCode.from_char("d"): False,  # -yaw
        keyboard.KeyCode.from_char("r"): False,  # open gripper
        keyboard.KeyCode.from_char("f"): False,  # close gripper
        keyboard.Key.space: False,  # reset to home
  }
  pass


if __name__ == "__main__":
  # sim_thread = Thread(targsim_thread.start()
  # key_thread.start()et=keyThread)

  # sim_thread.start()
  # key_thread.start()
  simulationThread()
