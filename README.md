## Welcome to Multi-Sensor Defense Analysis Platform (MSDAP)
MSDAPとはLiDAR攻撃の脅威について、オープンソースで解析するために作られた自動運転車セキュリティ解析用プラットフォームです。
Tier4 が開発したAWSIMとAutoware Universeを基盤として動作します。

## Features
This extension provides the following features:
- Simulate HFR attacks (特定の範囲のLiDAR点群を消去する)
- Simulate CPI attacks (任意形状のLiDAR点群を注入する)

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



