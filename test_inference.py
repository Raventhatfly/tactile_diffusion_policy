import torch
import hydra
from tactile_diffusion_policy.workspace.base_workspace import BaseWorkspace
from tactile_diffusion_policy.common.pytorch_util import dict_apply
import numpy as np
import os
import time

def main():
    # load checkpoint
    # ckpt_path = "./data/outputs/2025.01.15/11.16.02_train_diffusion_pick_and_place/checkpoints/latest.ckpt"
    ckpt_path = "./latest.ckpt"
    payload = torch.load(open(ckpt_path, 'rb'))
    cfg = payload['cfg']
    cls = hydra.utils.get_class(cfg._target_)
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

    obs = dict()
    obs["img1"] = np.random.rand(2,3,480,640)
    obs["img2"] = np.random.rand(2,3,480,640)
    # obs["joint_pos"] = np.random.rand(2,7)
    obs["joint_pos"] = np.array([[-0.002, -0.002,  0.009, -0.001, -0.003, -0.004,  0.001],[-0.002, -0.002,  0.009, -0.001, -0.003, -0.004, 0.001]])
    # print(obs["img1"])
    # print(obs["joint_pos"])
    print(type(obs["img1"][0][0][0][0]))
    print(type(obs["joint_pos"][0][0]))

    with torch.no_grad():
        policy.reset()
        obs = dict_apply(obs,lambda x: torch.Tensor(x).unsqueeze(0).to(device))
        start_time = time.perf_counter()
        result = policy.predict_action(obs)
        end_time = time.perf_counter()
        print(end_time-start_time)
    actions = result['action'][0].detach().to('cpu').numpy()
    print(actions)

if __name__ == "__main__":
    main()