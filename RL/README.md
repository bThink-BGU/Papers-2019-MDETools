# Papers-2019-MDETools
Code appendix for the MDETools 2019 paper
https://mdetools.github.io/mdetools19/challengeproblem.html

## What's here

* **README.md**: This file
* **train_agent.py**: training script for agent
* **run_agent.py**: running the train agent script

## What to run

This project contains two executable:
* `train_agent.py`: Runs red player training
* `run_agent.py`: Runs red player implementation of the challenge after training

You need to download the [Performance Improved Simulation Package](https://drive.google.com/open?id=1CgJ8B4Mst4Z8MiJVNoRPcbrZanLdV2jb) and extract it. 

## How to Run
Assuming you extracted [Performance Improved Simulation Package](https://drive.google.com/open?id=1CgJ8B4Mst4Z8MiJVNoRPcbrZanLdV2jb) to <path>, then:
* Training: ```python train_agent.py "<path>\Windows\windows_x86_64_server\SimGen Unity Framework.exe"```
* Running: execute "<path>\Windows\windows_x86_64\SimGen Unity Framework.exe" and then ```python run_agent.py```.
Check <path> for other OS/architectures. 

## Infrastructure
* This project uses the following packages:
    - [z3-solver](https://github.com/Z3Prover/z3) 
    - [gym](https://github.com/openai/gym)
    - [numpy](https://github.com/numpy/numpy)
    - [matplotlib](https://github.com/matplotlib/matplotlib)
    - [stable_baselines](https://github.com/hill-a/stable-baselines)
    - [tensorflow](https://github.com/tensorflow/tensorflow)
