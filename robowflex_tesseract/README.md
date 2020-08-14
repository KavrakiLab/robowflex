# Robowflex Tesseract

Robowflex Tesseract allows easy integration between robowflex robots/scenes and [tesseract](https://github.com/ros-industrial-consortium/tesseract) planners. In particular, the [trajOpt](https://github.com/ros-industrial-consortium/trajopt_ros) planner can be used for high-level motion planning in ROS, i.e., it searches for feasible and locally optimal trajectories given a scene (`robowflex::scene`) and start/goal states as (`moveit::core::RobotState`) or  a start state and a goal pose (`robowflex::RobotPose`) for a robot link (e.g., the end-effector).

# Scripts

There is an example script (`scripts` directory) that shows how the module can be used.
- `fetch_tabletop.cpp`: Planning requests (`moveit_msgs::MotionPlanRequest`) and scenes (`robowflex::scene`) are loaded from pre-recorded yaml files for a fetch robot (in the `scenes` directory) and the planning problem is solved using TrajOpt. The user can easily change the optimizer parameters and initialization.

# Installation Instructions

Both Tesseract and TrajOpt can be added as catkin packages in your workspace. 
Currently, Robowflex Tesseract uses the `kinetic-devel` branch of both [tesseract](https://github.com/ros-industrial-consortium/tesseract/tree/kinetic-devel) and [trajopt](https://github.com/ros-industrial-consortium/trajopt_ros/tree/kinetic-devel).
