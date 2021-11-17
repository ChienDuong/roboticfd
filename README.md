# Roboticfd

`Robotic fundamental tutorials.`
# Title

| **Author(s)** | Chien Dunog |
| :------------ | :-------------------------------------------------------------------------------------------- |
| **Topic(s)** | Robotic / Ros / AI |
| **Status**       | **In progress** |
# Pre-requirement:
1. Ubuntu 16.04 or 18.04
3. Ros

# Table of Content



0. [x] Config pre-requirements
1. [x] Manual control robot in simulation
2. [ ] Autonomous in Simulation: build map, run map
3. [ ] Create custom message. And use_case example.
4. [ ] Tips & trick: ros param, ros launch, switch topic tmux, roslog, ros bag, public topic one time.
5. [ ] Build an Hardware
## Config pre-requirement 
Install Ros, gazebo, rviz 
ROS: 18.04: melodic, 16.04: kinetic

Check ubuntu version
```BASH
cat /etc/os-release
```

Install Ros
```BASH
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-melodic-desktop-full
```

Basic package: should be install together to system. 

```BASH
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update

sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control

sudo apt-get install ros-melodic-unique-id

sudo apt-get install ros-melodic-uuid-msgs

sudo apt-get install ros-melodic-navigation
```

Note: If you dont want to install to the system `/opt/ros`, you can download the package from resource (example github) -> put in src folder -> catkin_make again

Example: slam-gmapping package I dont want to install to the system by using apt install. Then I put it in `education_robot_ws/src` folder gmapping. I can use this package after catkin_make the workspace again

If you want to install to the system

` sudo apt-get install ros-kinetic-slam-gmapping`


Setup Ros environment

```BASH
cd ~/education_robot_ws/
catkin_make
```
## Manual control robot in simulation

- Keyboard control
you only need 3 packages
 ![tree](wiki/images/manual_control_robot_simulation.png)
```BASH
cd  education_robot_ws
source devel/setup.bash
roslaunch edu_teleop edu_teleop_key.launch 
```
- Load gazebo and robot in environment

```BASH
cd  education_robot_ws
source devel/setup.bash
roslaunch edu_gazebo edu_empty_world.launch
```

Results:

![results](wiki/images/1_manual_control_rs.gif)


Good References: 

- https://github.com/ROBOTIS-GIT/turtlebot3_simulations