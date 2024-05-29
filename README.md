# CSE-Dataset
**A Benchmark Dataset for Collaborative SLAM in Service Environments** (On-going)

## 1. Overview 
<p align="center">
  <a href="https://youtu.be/Z_YZCeCAz9I">
    <img src="https://img.youtube.com/vi/Z_YZCeCAz9I/0.jpg" alt="CSE-Dataset" width="600">
  </a>
  <br>
  <span style="font-size:small;">Click the image to view an introduction to the CSE dataset !</span>
</p>

We introduce a new multi-modal C-SLAM dataset for multiple service robots in various indoor service environments, called C-SLAM dataset in Service Environments (CSE). We use the NVIDIA Isaac Sim to build data in various indoor service environments with the challenges that may occur in real-world service environments. By using simulation, we can provide accurate and precisely time-synchronized sensor data, such as stereo RGB, stereo depth, IMU, and GT poses. We configure three common indoor service environments (Hospital, Office, and Warehouse), each of which includes various dynamic objects that perform motions suitable to each environment. In addition, we navigate the three robots to mimic the actions of real service robots. Through these factors, we build a more realistic C-SLAM dataset for multiple service robots. We demonstrate our dataset by evaluating diverse state-of-the-art single-robot SLAM and multi-robot SLAM methods.
<br/><br/>

## 2. Dataset characteristics
### 2.1. Multi-Agent for C-SLAM
We utilize a total of 3 ground robots as a dataset for C-SLAM. All robots drive simultaneously, observing and avoiding each other. Each robot provides diverse sensor data (stereo RGB, stereo depth, IMU, ground truth poses), and all sensor data between robots is precisely time-synchronized.

### 2.2. Service Environments

- **Realistic Service Scenes** \
The CSE dataset is built in common 3 indoor service environments (Hospital, Office, Warehouse) where real service robots may operate.

<p align="center">
  <img src="https://github.com/vision3d-lab/CSE_Dataset/blob/main/assets/service_scene.png" width="80%" height="auto">
</p>

- **Challenging Cases** \
The CSE dataset includes various components and challenging cases that can occur in real-world service environments. It incorporates characteristics such as homogeneous textures, visual redundancy, repetitive patterns, and reflective materials, as well as numerous dynamic objects like humans. Particularly, these dynamic objects cause severe occlusions in the robot's view by moving around and avoiding the robot at close range.

<p align="center">
  <img src="https://github.com/vision3d-lab/CSE_Dataset/blob/main/assets/challenges.png" width="80%" height="auto">
</p>

### 2.3. Static/dynamic sequences
Each environment is organized into static and dynamic versions. The dynamic version is the sequences acquired in an environment where only dynamic objects like humans are added to the static environment. Additionally, each robot is driven with nearly identical trajectories in static and dynamic environments. We expect these categorizations to provide an opportunity to evaluate the effectiveness of SLAM algorithms handling dynamic objects.

<p align="center">
  <img src="https://github.com/vision3d-lab/CSE_Dataset/blob/main/assets/static_dynamic_seqs.gif" width="80%" height="auto">
</p>

## 3. Sequences
We acquired sequences for a total of 3 indoor service environments (Hospital, Office, Warehouse), divided into Static / Dynamic versions. Details of each sequence can be found in the main paper.

### Hospital / Office / Warehouse
We provide a total of 6 sequences for C-SLAM divided into static and dynamic for each environment. Below is the full sequence videos of the static / dynamic version of each environment. If you want to watch them, click on each scene name.

| ![Hospital](https://img.youtube.com/vi/rqxZ9Uby1z8/0.jpg) | ![Office](https://img.youtube.com/vi/U79whrA5fwE/0.jpg) | ![Warehouse](https://img.youtube.com/vi/_ViSRllbTAg/0.jpg) |
|:---:|:---:|:---:|
| [Hospital](https://youtu.be/rqxZ9Uby1z8) | [Office](https://youtu.be/U79whrA5fwE) | [Warehouse](https://youtu.be/_ViSRllbTAg) |

<br/>

## 4. Data format
We provide the sequence as the rosbag files(ROS1), separate for each robot. The rosbag file for each robot contains the topics shown in the table below. 

| Data                     | Resolution | Rate (Hz) | Topic Name                  | Message Type                |
|--------------------------|------------|-----------|-----------------------------|-----------------------------|
| Stereo Left RGB          | 1280x720   | 30        | /carter/rgb_left/compressed | sensor_msgs/CompressedImage |
| Stereo Right RGB         | 1280x720   | 30        | /carter/rgb_right/compressed| sensor_msgs/CompressedImage |
| Stereo Left Depth        | 1280x720   | 30        | /carter/depth_left          | sensor_msgs/Image           |
| Stereo Right Depth       | 1280x720   | 30        | /carter/depth_right         | sensor_msgs/Image           |
| IMU                      | -          | 120       | /carter/imu                 | sensor_msgs/Imu             |
| Ground Truth Pose        | -          | 120       | /carter/gt_pose             | nav_msgs/Odometry           |
| Left Camera Intrinsics   | -          | 120       | /carter/camera_info_left    | sensor_msgs/CameraInfo      |
| Right Camera Intrinsics  | -          | 120       | /carter/camera_info_right   | sensor_msgs/CameraInfo      |



## 5. Download
Coming soon!

