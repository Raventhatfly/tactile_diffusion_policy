from tactile_diffusion_policy.env.real_env import RealEnv
import torch
import hydra
from tactile_diffusion_policy.workspace.base_workspace import BaseWorkspace
from tactile_diffusion_policy.common.pytorch_util import dict_apply
import numpy as np
import os
import time

def main():
    env = RealEnv()
    # load checkpoint
    ckpt_path = "latest.ckpt"
    payload = torch.load(open(ckpt_path, 'rb'))
    cfg = payload['cfg']
    # print(cfg)
    # print(os.getcwd())
    # from tactile_diffusion_policy.workspace.train_workspace import TrainDiffusionWorkspace
    cls = hydra.utils.get_class(cfg._target_)
    # print(cls)
    workspace = cls(cfg)
    workspace: BaseWorkspace
    workspace.load_payload(payload, exclude_keys=None, include_keys=None)


    policy = workspace.model
    if cfg.training.use_ema:
        policy = workspace.ema_model

    device = torch.device('cuda')
    # device = torch.device('cpu')
    policy.eval().to(device)

    # # set inference params
    # policy.num_inference_steps = 16 # DDIM inference iterations
    policy.num_inference_steps = 16
    # policy.n_action_steps = policy.horizon - policy.n_obs_steps + 1
    policy.n_action_steps = 1

    # delta_action = cfg.task.dataset.get('delta_action', False)
    # dt = 0.1
    # with env:
    #     while True:
    #         obs = env.get_obs()
    #         with torch.no_grad():
    #             policy.reset()
    #             result = policy.predict_action(obs)
    #             action = result['action'][0].detach().to('cpu').numpy()
    #             assert action.shape[-1] == 2
    #             del result
    #         if input("Stop Inference? (y/n)") == "y":
    #             break
    #         env.exec_actions(action)
    with env:
        # act = np.array([-0.025, 0.245, 0.201, -0.135, 0.341, 1.745, 0.001])
        # act = np.array([-0.09, 0.302, 0.226, 0.121, 0.819, 2.004, 0.07204837078602602])
        # env.exec_actions(act,1)
        # time.sleep(10)
        # env.exec_actions_slow(act,0.01)
        obs = env.get_obs()
        print(obs["joint_pos"])
        print(type(obs["joint_pos"]))
        print(type(obs["img1"][0][0][0][0]))
        # print(obs["img1"])
        with torch.no_grad():
            policy.reset()

            obs = dict_apply(obs,lambda x: torch.Tensor(x).unsqueeze(0).to(device))
            # obs = dict_apply(obs,lambda x: torch.Tensor(x).unsqueeze(0).to('cpu'))
            # print(obs["joint_pos"].shape)
            # print(obs["img1"].shape)


            start_time = time.perf_counter()
            result = policy.predict_action(obs)
            end_time = time.perf_counter()
            print(end_time-start_time)

        actions = result['action'][0].detach().to('cpu').numpy()
        print(actions)
            
    # with torch.no_grad():
    #     policy.reset()

    #     obs = dict_apply(obs,lambda x: torch.Tensor(x).unsqueeze(0).to(device))
    #     # obs = dict_apply(obs,lambda x: torch.Tensor(x).unsqueeze(0).to('cpu'))
    #     # print(obs["joint_pos"].shape)
    #     # print(obs["img1"].shape)


    #     start_time = time.perf_counter()
    #     result = policy.predict_action(obs)
    #     end_time = time.perf_counter()
    #     print(end_time-start_time)
                


if __name__ == "__main__":
    main()