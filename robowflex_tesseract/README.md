# Robowflex Tesseract

Robowflex Tesseract allows easy integration between robowflex robots/scenes and [Tesseract](https://github.com/ros-industrial-consortium/tesseract) planners. In particular, Tesseract [TrajOpt](https://github.com/ros-industrial-consortium/trajopt_ros) planner can be used for high-level motion planning in ROS, i.e., it searches for feasible trajectories given a scene (`robowflex::scene`) and initia/goal states as (`moveit::core::RobotState`) or start state/goal pose as a (`robowflex::RobotPose`) for a robot link (e.g., the end-effector).

# Scripts

There is an example script (`scripts` directory) that shows how the module can be used.
- `fetch_tabletop.cpp`: Planning requests (`moveit_msgs::MotionPlanRequest`) and scenes (`robowflex::scene`) are loaded from pre-recorded yaml files (in the `scenes` directory) and the planning problem is solved using TrajOpt. The user can easily change the optimizer parameters and initialization.

# Installation Instructions

Both Tesseract and TrajOpt can be added as catkin packages in your workspace. Currently, Robowflex Tesseract uses the `kinetic-devel` branch of both [tesseract](https://github.com/ros-industrial-consortium/tesseract/tree/kinetic-devel) and [trajopt](https://github.com/ros-industrial-consortium/trajopt_ros/tree/kinetic-devel).
