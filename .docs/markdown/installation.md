# Installation Instructions

Robowflex is supported on many ROS platforms, from Kinetic on Ubuntu 16.04 to Noetic on 20.04.

Here are some bare-bones installation instructions to get up and running on a new Ubuntu 18.04 machine without ROS already installed.
This will only enable the core modules to be built, as some modules require special packages to be installed.

First, [install ROS](http://wiki.ros.org/melodic/Installation/Ubuntu) following the directions copied below:

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo rosdep init
rosdep update
source /opt/ros/melodic/setup.bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

We recommend `catkin-tools` to build your ROS workspace:
```sh
sudo apt install python-catkin-tools
```

Install _MoveIt_:
```sh
sudo apt install ros-melodic-moveit
```

Finally, create a workspace with Robowflex inside:
```sh
cd ~
mkdir -p rb_ws/src
cd rb_ws
source /opt/ros/melodic/setup.bash # if you haven't already
catkin config --init
cd src
git clone https://github.com/KavrakiLab/robowflex.git
catkin build
```

To try out a demo script, you first need a robot description.
The easiest to try is the _Fetch_ robot, either by debian or source:
```sh
# Debian
sudo apt install ros-melodic-fetch-ros

# Or, Source
cd ~/rb_ws/src
git clone https://github.com/fetchrobotics/fetch_ros
catkin build
```

After the workspace is built, source and run a demo:
```sh
cd ~/rb_ws
source ./devel/setup.bash
rosrun robowflex_library fetch_test
```
