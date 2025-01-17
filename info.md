# Information on how to run diffusion policy inference on ARX R5/L5 Arm

## Merging two environments together

After cloning the repo make sure that arx5-sdk submodule is added successfully; 
Go into the arx5-sdk folder `conda_environments/py39_environment.yaml`, modify the environment name to `robodiff`; Intall the environment for diffusion policy in the root folder of this project which is called: `conda_environment.yaml`. Activate environment `robodiff`.

And then cd to the `conda_environments/py39_environment.yaml` folder and enter:
```shell
conda env update --name robodiff --file py39_environment.yaml
```