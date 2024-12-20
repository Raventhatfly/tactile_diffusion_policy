import time

import mujoco
import mujoco.viewer
from gymnasium.envs.mujoco.mujoco_env import MujocoEnv

# m = mujoco.MjModel.from_xml_path('./models/assets/quad_insert.xml')
m = mujoco.MjModel.from_xml_path('./models/arx5.xml')
d = mujoco.MjData(m)

print(m.nv)


with mujoco.viewer.launch_passive(m, d) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  while viewer.is_running():
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(m, d)

    # Example modification of a viewer option: toggle contact points every two seconds.
    # with viewer.lock():
    #   viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()


    d.qpos[0] = 0.5
    d.qpos[1] = 0.5
    d.qpos[2] = 0.5
    print(d.xfrc_applied)
    # d.qpos[4] = 0.3
    # d.qpos[3] = 0.4
    # d.qpos[5] = 0.15
    # print(d.sensor)
    

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)