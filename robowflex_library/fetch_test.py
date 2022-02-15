from sys import argv

import robowflex_library as rf

GROUP = "arm_with_torso"

DEFAULT_ADAPTERS = [
    "default_planner_request_adapters/AddTimeParameterization",
    "default_planner_request_adapters/FixWorkspaceBounds",
    "default_planner_request_adapters/FixStartStateBounds",
    "default_planner_request_adapters/FixStartStateCollision",
    "default_planner_request_adapters/FixStartStatePathConstraints"
]

# Startup ROS
print('Starting ROS...')
ros = rf.ROS(len(argv), argv, "robowflex", 1)
print('ROS running!')

# Create the default Fetch robot
print('Creating Fetch robot...')
fetch = rf.FetchRobot()
fetch.initialize(True)

# Create an empty scene
print('Creating scene...')
scene = rf.Scene(fetch)

# Create the default planner for the Fetch
print('Creating planner...')
planner = rf.FetchOMPLPipelinePlanner(fetch, "default")
planner.initialize(rf.Settings(), DEFAULT_ADAPTERS)

# Set the Fetch's base pose
print('Posing Fetch...')
fetch.set_base_pose(1, 1, 0.5)

# Set the Fetch's head pose to look at a point
fetch.point_head([2, 1, 1.5])

# Create a motion planning request with a pose goal
print('Creating planning request...')
request = rf.MotionRequestBuilder(planner, GROUP, '')
fetch.set_group_state(GROUP, [0.05, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0])
request.set_start_configuration(fetch.get_scratch_state())

# Unfurl
fetch.set_group_state(GROUP, [0.265, 0.501, 1.281, -2.272, 2.243, -2.774, 0.976, -2.007])
request.set_goal_configuration(fetch.get_scratch_state())
request.set_config("RRTConnect")

# Do motion planning!
print('Motion planning...')
result = planner.plan(scene, request.get_request())
# NOTE: We don't export the MoveIt messages library; could probably get the below constant from the
# Python bindings but I'm lazy
if result.error_code_.val != 1:
  raise RuntimeError(f'Motion planning failed with code: {result.error_code_.val}')

# Create a trajectory object for better manipulation
print('Exporting trajectory...')
trajectory = rf.Trajectory(result.trajectory_)

# Output path to a file for visualization
trajectory.to_yaml_file("fetch_path.yml")
