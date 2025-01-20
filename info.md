# Information on how to run diffusion policy inference on ARX R5/L5 Arm

## HuggingFace Hub Problem

While running diffusion policy, an error relating to hugging face hub might be encountered.
Please first remove it by:
```shell
mamba remove huggingface_hub
```
Then install it by:
```shell
mamba install huggingface-hub==0.20.2 -c conda-forge
```
Downgrading it to 0.20.2 version fix the problem.

## Merging two environments together

After cloning the repo make sure that arx5-sdk submodule is added successfully; 
Go into the arx5-sdk folder `conda_environments/py39_environment.yaml`, modify the environment name to `robodiff`; Intall the environment for diffusion policy in the root folder of this project which is called: `conda_environment.yaml`. Activate environment `robodiff`.

And then cd to the `conda_environments/py39_environment.yaml` folder and enter:
```shell
conda env update --name robodiff --file py39_environment.yaml
```

## Start SLCAN
```shell
sudo slcand -o -f -s8 /dev/arxcan0 can0 && sudo ifconfig can0 up
sudo slcand -o -f -s8 /dev/arxcan1 can1 && sudo ifconfig can1 up
```

## Running the Training Script

## Runing the Inference Script
```shell
python inference.py --config-dir=./tactile_diffusion_policy/config --config-name=train_diffusion_workspace.yaml
```