## Welcome to Multi-Sensor Defense Analysis Platform (MSDAP)
MSDAP is a platform designed to analyze threats of LiDAR attacks in autonomous driving security, developed as an open-source tool. It operates on the AWSIM and Autoware Universe foundation, developed by Autoware Foundation (https://autoware.org/).
![framework2](https://github.com/user-attachments/assets/aa70a2b6-96fc-499e-9782-a34f68f61bba)


## Features
This extension provides the following features:
- Simulate HFR attacks (特定の範囲のLiDAR点群を消去する)
- Simulate CPI attacks (任意形状のLiDAR点群を注入する)
unattacked point cloud<br>
![benign](https://github.com/user-attachments/assets/013ca6bf-cf5a-4cac-950b-368c9e6e9264)

attacked point cloud<br>
![attacked](https://github.com/user-attachments/assets/730c1716-7615-4a40-a606-fbde33c9ab89)




## Prerequirements
- AWSIM
- Autoware Universe 

## Install
- Install AWSIM (Driving Simulator). https://tier4.github.io/AWSIM/GettingStarted/SetupUnityProject/
- Install Autoware Universe (Open-soruced autonomous driving system). https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/
- Install this package 
```
cd ~/ros2_ws/src && git clone git@github.com:Ofurosuki/lidar_attack_simulator.git
```
## Usage
- Source ROS2 underlay
```
source /opt/ros/humble/setup.bash
```

- Source the ROS2 package
```
cd ~/ros2_ws/src/lidar_attack_simulator && source install/setup.bash
```
- Configure parameters in a launch file, hfr_simulator_launch.py or cpi_simulator_launch.py

- Launch the file.
```
ros2 launch attack_simulator hfr_simulator
```



