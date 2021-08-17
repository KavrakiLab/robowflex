# Robowflex Tesseract

Robowflex Tesseract allows easy integration between Robowflexs' robots/scenes and [tesseract](https://github.com/ros-industrial-consortium/tesseract) planners. In particular, the [trajOpt](https://github.com/ros-industrial-consortium/trajopt_ros) planner can be used for high-level motion planning using a `robowflex::scene` and a `planning_interface::MotionPlanRequest` as other planners.

## Planning pipeline
The TrajOptPlanner class supports the standard planning pipeline from Robowflex, i.e., given a `robowflex::scene` and a `planning_interface::MotionPlanRequest`, first create a planner, initialize it, set the planner's parameters and plan. The result is given as a `planning_interface::MotionPlanResponse`.

Some of these steps have unique features or alternative uses in the TrajOptPlanner class compared to other Robowflex planners:

- **Initialize**: The simplest way to initialize the planner is by providing the name of the chain group on the robot's srdf that makes the manipulator. If the robot does not have a chain group, the user can manually add it to the srdf.
If the user can not or do not want to modify the robot's srdf, an overloaded version of the initialize() method is available that takes the name of the `base_link` and `tip_link` and creates the manipulator on the fly.
- **Parameters**: The planner has a public structure ``options`` that allow the user to set the planner's parameters. This structure contains parameters that determine the planner behavior, as well as the specific parameters used in the optimizer. Some important parameters are:
    - backend_optimizer: Choose the backend optimizer used by TrajOpt. See [trajopt](https://github.com/ros-industrial-consortium/trajopt_ros/tree/kinetic-devel) for details.
    - perturb_init_traj: Whether the initial trajectory given to the optimizer should be randomly perturbed or not. This can be useful to let the planner explore different solutions when the initial trajectories are stationary or straight lines in C-Space.
    - return_first_sol: When set, the planner runs only once and returns. When false, the planner is allowed to run more than once (see return_after_timeout). If this flag is set, perturb_init_traj is set too.
    - return_after_timeout: When set, the planner will run many times until reaching max_planning_time. When false, the planner will run until it either finds a feasible solution or it reaches the max_planning_time. This flag is only valid if return_first_sol is false.
    - use_cont_col_avoid: Whether the planner should use continuous collision avoidance costs/constraints or discrete.
    - num_waypoints: Number of waypoints used by the planner.

- **Plan**: In addition to the standard plan() method from robowflex::Planner, TrajOptPlanner offers additional planning interfaces. One that stands out receives the scene, the start state and a goal pose for a given link. This interface allows the user to perform planning without a full goal state specification (i.e., no need to perform IK queries). Instead, it creates cartesian pose constraints on the last waypoint of the trajectory for the given link (e.g., the end effector). 
All the additional versions of the plan() method return a PlannerResult structure with information about the planner convergence status. In particular, PlannerResult is a pair of booleans. The first field specifies whether the optimization algorithm converged or not. The second field specifies whether the resulting trajectory is collision-free or not. Note that it is possible that the optimization converges to a trajectory that is in collision (i.e., PlannerResult->first = true, PlannerResult->second = false) because the collision avoidance terms can be defined as costs instead of constraints.

### Custom terms in TrajOpt
The original implementation of [trajopt_ros](https://github.com/ros-industrial-consortium/trajopt_ros/tree/kinetic-devel) allows the user to incorporate custom terms to the trajectory optimization formulation. These custom terms are usually out of the scope of the standard motion planning problem but they might come handy for specific robotic tasks. The standard supported terms are usually joint/cartesian cost/constraints for positions, velocities and accelerations.

`robowflex::TrajOptPlanner` exposes this feature by letting the user inherit from the planner class and implement their own `robowflex::TrajOptPlanner::plan()` method. Here, the user is responsible for adding all the required terms to construct the trajectory optimization problem. The method expects as arguments a `robowflex::Scene` and a `robot_state:RobotState` start state, as this is the minimum information required to create a trajectory optimization problem. Notice that a goal state can be defined using custom terms (e.g., position cartesian terms for the robot's end-effector). 

## Scripts
6 different scripts (`scripts` directory) show how the planner can be used in different ways. Some of the scripts load the planning request and scene from yaml files (`scenes` folder) and require to run RVIZ to visualize the scene, states and output trajectory:

- fetch_trajopt.cpp: Shows the simplest setup for planning using an empty scene and given start and goal states.
- fetch_tabletop_goalstate.cpp: Shows how to plan for a given scene, start and goal state.
- fetch_tabletop_goalpose.cpp: Shows how to plan for a given scene, start state and goal pose for the end effector.
- fetch_tabletop_inits.cpp: Shows how to plan for a given scene, start and goal state using different initial trajectories, namely stationary and joint_interpolated (straight-line).
- fetch_tabletop_planning_time.cpp: Shows how to plan for a given scene, start and goal with different behaviors for the planner. The first behavior runs the planner just once and returns right away. The second behavior allows the planner to run more than once and runs until a feasible solution (collision-free) is found or the time limit is reached. The final behavior forces the planner to take all the available time and returns the best found solution at the end. In the two latter cases, the planner is initialized with a perturbed version of the given initial trajectory every time is run.
- ur5_custom_planning.cpp: Shows how to build a custom planner by defining cartesian constraints for the robot's end-effector at multiple timesteps in the trajectory.

## Installation Instructions
Both Tesseract and TrajOpt can be added as catkin packages in your workspace. 
Currently, Robowflex Tesseract uses the `kinetic-devel` branch of both [tesseract](https://github.com/ros-industrial-consortium/tesseract/tree/kinetic-devel) and [trajopt](https://github.com/ros-industrial-consortium/trajopt_ros/tree/kinetic-devel).
The Tesseract module can be difficult to compile and integrate in a complex ROS workspace.
See the provided Docker image for an example of installation.
