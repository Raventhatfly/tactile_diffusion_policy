from tactile_diffusion_policy.env.real_env import RealEnv
import torch
import hydra
from tactile_diffusion_policy.workspace.base_workspace import BaseWorkspace
from tactile_diffusion_policy.common.pytorch_util import dict_apply

def main():
    env = RealEnv()
    # load checkpoint
    ckpt_path = "latest.ckpt"
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
    policy.eval().to(device)

    # set inference params
    policy.num_inference_steps = 16 # DDIM inference iterations
    policy.n_action_steps = policy.horizon - policy.n_obs_steps + 1

    # delta_action = cfg.task.dataset.get('delta_action', False)
    # dt = 0.1
    with env:
        while True:
            obs = env.get_obs()
            with torch.no_grad():
                policy.reset()
                result = policy.predict_action(obs)
                action = result['action'][0].detach().to('cpu').numpy()
                assert action.shape[-1] == 2
                del result
            if input("Stop Inference? (y/n)") == "y":
                break
            env.exec_actions(action)

if __name__ == "__main__":
    main()