# Face Tracking with Raspberry Pi

This project sets up a Raspberry Pi to perform face tracking using a pan-tilt mechanism controlled by servos. The project utilizes ROS2 for managing the nodes, OpenCV for face detection, and `pigpio` for controlling the servos.

## Prerequisites

- Raspberry Pi (with Ubuntu 22.04.4 LTS installed)
- Internet connection
- Camera module for the Raspberry Pi
- Pan-tilt mechanism with 2 servos
- Breadboard and jumper wires

## Installation Steps

1. Update and Upgrade the System
```
sudo apt update
sudo apt upgrade 
```

2. Install ROS2
Follow the instructions on the ROS2 Humble installation page for the Raspberry Pi OS. 
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#install-ros-2-packages

3. Install Development Tools and Dependencies
```
sudo apt install -y python3-pip python3-colcon-common-extensions
sudo apt install -y libopencv-dev python3-opencv
sudo apt install python-setuptools python3-setuptools
```

4. Install Additional Python Packages
```
pip3 install rclpy
pip3 install geometry_msgs
pip3 install image_transport
```

5. Create and Build the ROS2 Workspace
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

6. Clone the Project Repository
```
cd ~/ros2_ws/src
git clone https://github.com/pieroVG/robo_P
cd ~/ros2_ws
colcon build
source install/setup.bash
```

7. Install and Configure pigpio
```
cd ~/ros2_ws/src/face_reco/face_reco/pigpio-master
make
sudo make install

sudo pigpiod
```

8. Setting Up the Camera
This will depend on the camera you are using
```
sudo raspi-config
```

9. Running the Project
```
ros2 launch robO_bringup tracking.launch.py
```
