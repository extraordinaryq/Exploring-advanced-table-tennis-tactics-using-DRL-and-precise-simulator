#  TableTennis-DRL-Simulator 🏓
**Explore advanced table tennis tactics using deep reinforcement learning and precise physics simulator**


<div align="left">
    <a href="./">
        <img src="./pong.gif" width="80%"/>
    </a>

High-resolution demo video is available on [YouTube](https://www.youtube.com/playlist?list=PL2UD_JyvqMzfo0Z4aaf3IONPuUPtUIifV)  

---

## Overview 🌟  

**TableTennis-DRL-Simulator** is a high-fidelity virtual training platform for table tennis, integrating **Deep Reinforcement Learning (DRL)** and **biomechanically grounded physics simulation**.  
Our goal is to enable virtual agents to autonomously learn advanced tactics while ensuring consistency with **real-world dynamics**.  

- 🎮 **VR + DRL**: building on our earlier VR-based DRL system for autonomous skill learning  
- ⚙️ **High-fidelity physics**: precise modeling of aerodynamics, Magnus effects, and realistic collisions  
- 🧍 **Human-representative agents**: joint-level control with biomechanical constraints, calibrated by IMU data  
- 🎯 **Tactical insights**: interpretable three-stroke strategies validated by experts  

This project bridges the gap between **simulation and reality**, offering both a **research testbed** and **training guidance for athletes**.  

---

## Key Features 🔑  

- **Physics-accurate ball dynamics** (gravity, drag, Magnus force)  
- **Realistic collisions** (ball–table, ball–racket with elastic & frictional models)  
- **Human model** with shoulder–elbow–wrist joints and inertia-based motion control  
- **Two-stage DRL training**:  
  - Stage 1: Soft Actor-Critic (SAC) for consistent ball return  
  - Stage 2: Self-play for tactical refinement  
- **Strategy analysis**: data-driven insights into tempo, spin variation, and spatial manipulation  

---

## TODO List 📅  

- [x] **Physics-based Simulator**  
  - High-fidelity ball dynamics (gravity, drag, Magnus effect)  
  - Realistic collision models (ball–table, ball–racket)  
  - Human-representative agent with biomechanical constraints  
  - ✅ **Already open-sourced**  

- [ ] **Pretrained Models**  
  - DRL agents trained with SAC + self-play  
  - To be released after paper acceptance  

- [ ] **Datasets**  
  - Simulation trajectories and tactical analysis data  
  - To be released after paper acceptance  
---

## About 📌
**Pingpong-human Table Tennis Game** is a multi-agent reinforcement learning environment built on [Unity ML-Agents](https://unity.com/products/machine-learning-agents).

> **Version:** Up-to-date with ML-Agents Release 21
## Contents 📂
1. [Getting started](#getting-started)
2. [Training](#training)
3. [SAC](#SAC)
4. [Self-play](#self-play)
5. [Environment description](#environment-description)
6. [Human model setup](#human-model-setup)
6. [Baselines](#baselines)

## Getting Started 🚀
1. Install the Unity ML-Agents toolkit (Release 21+) by following the [installation instructions](https://github.com/Unity-Technologies/ml-agents/tree/release_21).
2. Download or clone this repo containing the `Pingpong-human` Unity project.
3. Open the `Pingpong-human` project in Unity (Unity Hub → Projects → Add → Select root folder for this repo).
4. Load the `TableTennis human inference` scene (Project panel → Assets → `TableTennis human inference.unity`).
5. Click the ▶ button at the top of the window. This will run the agent in inference mode using the provided baseline model.

## Training 🎓

1. Load the `TableTennis human train` scene (Project panel → Assets → `TableTennis human train.unity`).
2. If you previously changed Behavior Type to `Heuristic Only`, ensure that the Behavior Type is set back to `Default` .
2. Activate the virtual environment containing your installation of `ml-agents`.
3. Make a copy of the [provided training config file](Config/) in a convenient working directory.
4. Run from the command line `mlagents-learn <path to config file> --run-id=<some_id> --time-scale=1`
    - Replace `<path to config file>` with the actual path to the file in Step 4
5. When you see the message "Start training by pressing the Play button in the Unity Editor", click ▶ within the Unity GUI.
6. From another terminal window, navigate to the same directory you ran Step 5 from, and run `tensorboard --logdir results` to observe the training process. 

For more detailed instructions, check the [ML-Agents getting started guide](https://github.com/Unity-Technologies/ml-agents/blob/release_21/docs/Getting-Started.md).

## SAC 🤝
To enable PPO algorithm:
1. Set both Left and Right Agent Team ID to 0.
2. Include the SAC hyperparameter hierarchy in your trainer config file, or use the provided file in `Config/pingpong_human.yaml` ([ML-Agents Documentation](https://github.com/Unity-Technologies/ml-agents/blob/main/docs/Learning-Environment-Design-Agents.md#teams-for-adversarial-scenarios))
3. Set the reward function by switching `Rewardfunction` to `PPO` in `TableTennisEnvControl.cs`.

## Self-Play ⚡
To enable self-play algorithm:
1. Set either Left or Right Agent Team ID to 1.
2. Include the self-play hyperparameter hierarchy in your trainer config file, or use the provided file in `Config/pingpong_human_Self.yaml` ([ML-Agents Documentation](https://github.com/Unity-Technologies/ml-agents/blob/main/docs/Learning-Environment-Design-Agents.md#teams-for-adversarial-scenarios))
3. Set the reward function by switching `Rewardfunction` to `SelfPlay` in `TableTennisEnvControl.cs`.

## Environment Description 🎮
**Rule:** Exactly same as table tennis in real life.

**Action space:**

12 continuous action branches:
- translational acceleration (3D) of the human model
- angular velocity (3D) of the shoulder joint
- angular velocity (1D) of the elbow joint
- angular velocity (2D) of the wrist joint

**Observation space:**

Total size: 22
- Angular velocity (3D), speed (3D) and position (3D) of the ball
- position (3D) and speed (3D) of the human model
- rotation angles (3D) of the shoulder joint
- rotation angle (1D) of the elbow joint
- rotation angles (2D) of the wrist joint

**Workflow:**
![Table Tennis Game](https://github.com/extraordinaryq/extraordinaryq/Exploring-advanced-table-tennis-tactics-using-DRL-and-precise-simulator/blob/main/workflow.svg)
<br>
**Reward function:**

The project contains some examples of how the reward function can be defined.
The SAC algorithm gives a +0.25 reward each time the agent receives the ball
and +1 reward each time the agent hits the ball to opponent side. 
The self-play algorithm gives a +1 reward for winner and -1 penalty for loser.

## Human model setup 🧍

Switch different human model by changing `Humanmodel` value in `HumanAi.cs`.
<br>Specific parameters of different parameter:

| Model     | Character | 
|-----------|-------|
| Model   1 | 21-year-old adult male | 
| Model   2 | 20-year-old adult female | 
| Model   3 | 12-year-old boy   | 
| Model   4 | 12-year-old girl  |

## Baselines (coming soon) 📊
The following baselines are included:
- `Pingpong-human_SAC_Model1.onnx` - Model1 trained using PPO in 10M steps
- `Pingpong-human_SelfPlay_Model4.onnx` - Model4 trained using Self-Play in 20M steps

## Acknowledgement 🙏

The code base is built with [Ultimate Volleyball](https://www.gocoder.one/blog/competitive-self-play-unity-ml-agents/).

Thanks for the great implementations! 

## Citation 📄

If our code or models help your work, please cite our paper:
```BibTeX
@InProceedings{Seah_2024_ICVR,
    author    = {Jiang, Daqi and Seah, Hock Soon and Tandianus, Budianto and Sui, Yiliang and Wang, Hong},
    title     = {Deep Reinforcement Learning-based Training for Virtual Table Tennis Agent},
    booktitle = {International Conference on Virtual Reality (ICVR)},
    month     = {July},
    year      = {2024},
    pages     = {146-152},
}
@article{jiang2025tabletennis,
  title={Exploring Advanced Table Tennis Tactics using Deep Reinforcement Learning and Precise Physics Simulator},
  author={Jiang, Daqi and Wang, Tianyu and Wang, Hong and Seah, Hock Soon and Xiang, Zihao and Zhang, Bowen},
  year={2025}
}
```
