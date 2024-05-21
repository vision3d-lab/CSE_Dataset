# CSE-Dataset
**A Benchmark Dataset for Collaborative SLAM in Service Environments** (On-going)
<h2>
<a href="https://arxiv.org/abs/2403.05005">Paper (arxiv)</a> |
<a href="https://vision3d-lab.github.io/ditto">Project Page</a> 
</h2>

## Overview (Have to convert the video!)
<p align="center">
  <a href="https://youtu.be/EKeVWzePS5M">
    <img src="https://img.youtube.com/vi/EKeVWzePS5M/0.jpg" alt="CSE-Dataset" width="600" height="auto">
  </a>
</p>

As service environments have become diverse, they have started to demand complicated tasks that are difficult for a single robot to complete. This change has led to an interest in multiple robots instead of a single robot. C-SLAM, as a fundamental technique for multiple service robots, needs to handle diverse challenges such as homogeneous scenes and dynamic objects to ensure that robots operate smoothly and perform their task safely. However, existing C-SLAM datasets do not cover the various indoor service environments with the aforementioned challenges. To close this gap, we introduce a new multi-modal C-SLAM dataset for multiple service robots in various indoor service environments, called C-SLAM dataset in Service Environments (CSE). We use the NVIDIA Isaac Sim to build data in various indoor service environments with the challenges that may occur in real-world service environments. By using simulation, we can provide accurate and precisely time-synchronized sensor data, such as stereo RGB, stereo depth, IMU, and GT poses. We configure three common indoor service environments (Hospital, Office, and Warehouse), each of which includes various dynamic objects that perform motions suitable to each environment. In addition, we drive the robots to mimic the actions of real service robots. Through these factors, we build a more realistic C-SLAM dataset for multiple service robots. We demonstrate our dataset by evaluating diverse state-of-the-art single-robot SLAM and multi-robot SLAM methods.
<br/><br/>

## Dataset characteristics
### 1. Multi-Agent for C-SLAM
We utilize a total of 3 ground robots as a dataset for C-SLAM. All robots drive simultaneously, observing and avoiding each other. Each robot provides diverse sensor data (stereo RGB, stereo depth, IMU, ground truth poses), and all sensor data between robots is precisely time-synchronized.

### 2. Service Environments

- **Realistic Service Scenes** \
The CSE dataset is built in common 3 indoor service environments (Hospital, Office, Warehouse) where real service robots may operate.

- **Challenging Cases** \
The CSE dataset includes various components and challenging cases that can occur in real-world service environments. It incorporates characteristics such as homogeneous textures, visual redundancy, repetitive patterns, and reflective materials, as well as numerous dynamic objects like humans. Particularly, these dynamic objects cause severe occlusions in the robot's view by moving around and avoiding the robot at close range.

### 3. Static/dynamic sequences
Each environment is organized into static and dynamic versions. The dynamic version is the sequences acquired in an environment where only dynamic objects like humans are added to the static environment. Additionally, each robot is driven with nearly identical trajectories in static and dynamic environments. We expect these categorizations to provide an opportunity to evaluate the effectiveness of SLAM algorithms handling dynamic objects.

https://github.com/vision3d-lab/CSE_Dataset/blob/main/assets/service_scene.png

<br/><br/>

## Sequences
<p align="center">
  <img src="https://github.com/vision3d-lab/CSE_Dataset/blob/main/assets/Sequence_Table.png" width="80%" height="auto"/>
</p>

### hospital / Office / Warehouse

| ![Hospital](https://img.youtube.com/vi/rqxZ9Uby1z8/0.jpg) | ![Office](https://img.youtube.com/vi/U79whrA5fwE/0.jpg) | ![Warehouse](https://img.youtube.com/vi/_ViSRllbTAg/0.jpg) |
|:---:|:---:|:---:|
| [Hospital](https://youtu.be/rqxZ9Uby1z8) | [Office](https://youtu.be/U79whrA5fwE) | [Warehouse](https://youtu.be/_ViSRllbTAg) |


### Lifelong_SLAM 

<br/><br/>

## Download
- dataset structure
```bash
.
├── hospital
│   ├── dynamic_hospital_robot1.bag
│   ├── dynamic_hospital_robot2.bag
│   ├── dynamic_hospital_robot3.bag
│   ├── static_hospital_robot1.bag
│   ├── static_hospital_robot2.bag
│   └── static_hospital_robot3.bag
├── lifelong_sequence
│   ├── lifelong_static_office_origTraj_robot1.bag
│   ├── lifelong_static_office_origTraj_robot2.bag
│   ├── lifelong_static_office_origTraj_robot3.bag
│   ├── lifelong_static_office_reverseTraj_robot1.bag
│   ├── lifelong_static_office_reverseTraj_robot2.bag
│   └── lifelong_static_office_reverseTraj_robot3.bag
├── office
│   ├── dynamic_office_robot1.bag
│   ├── dynamic_office_robot2.bag
│   ├── dynamic_office_robot3.bag
│   ├── static_office_robot1.bag
│   ├── static_office_robot2.bag
│   └── static_office_robot3.bag
└── warehouse
    ├── dynamic_warehouse_robot1.bag
    ├── dynamic_warehouse_robot2.bag
    ├── dynamic_warehouse_robot3.bag
    ├── static_warehouse_robot1.bag
    ├── static_warehouse_robot2.bag
    └── static_warehouse_robot3.bag
```
- Download
우선 구글 폼(link)를 작성해주세요, 작성이 완료되면 당신의 이메일로 데이터셋을 다운받을 수 있는 링크가 제공됩니다.
