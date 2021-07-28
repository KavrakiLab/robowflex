import importlib
import robowflex_visualization as rv
import math

# reload modules, as Blender likes to keep things around.
importlib.reload(rv)
importlib.reload(rv.robot)
importlib.reload(rv.scene)
importlib.reload(rv.primitives)
importlib.reload(rv.utils)

# Load a Fetch robot.
fetch = rv.robot.Robot("Fetch", "package://robowflex_resources/fetch/robots/fetch.urdf")

# Open the Gripper
fetch.set_joint("l_gripper_finger_joint", 0.05)
fetch.set_joint("r_gripper_finger_joint", 0.05)

# Keyframe the head at neutral
fetch.set_joint("head_pan_joint", 0)
fetch.set_joint("head_tilt_joint", 0)
fetch.add_keyframe("head_pan_joint", 0)
fetch.add_keyframe("head_tilt_joint", 0)

# Keyframe the head looking over at the block
fetch.set_joint("head_pan_joint", math.radians(50))
fetch.set_joint("head_tilt_joint", math.radians(30))
fetch.add_keyframe("head_pan_joint", 60)
fetch.add_keyframe("head_tilt_joint", 60)

# Load a planning scene.
scene = rv.scene.Scene("Scene",
                       "package://robowflex_library/yaml/test_fetch.yml")

# Animate a simple motion plan.
frame = fetch.animate_path(
    "package://robowflex_visualization/yaml/fetch_pick.yml",
    15,    # FPS
    30,    # Start Frame
)

# Keyframe the gripper at opening
fetch.add_keyframe("l_gripper_finger_joint", frame)
fetch.add_keyframe("r_gripper_finger_joint", frame)

# Keyframe head still looking over
fetch.add_keyframe("head_pan_joint", frame)
fetch.add_keyframe("head_tilt_joint", frame)

# Attach the cube to the gripper
cube = scene.get_object("Cube3")
fetch.attach_object("wrist_roll_link", cube, frame)

# Close the Gripper
fetch.set_joint("l_gripper_finger_joint", 0.04)
fetch.set_joint("r_gripper_finger_joint", 0.04)
fetch.add_keyframe("l_gripper_finger_joint", frame + 30)
fetch.add_keyframe("r_gripper_finger_joint", frame + 30)

# Animate the plan in reverse
frame = fetch.animate_path(
    "package://robowflex_visualization/yaml/fetch_pick.yml",
    15,    # FPS
    frame + 30,    # Start Frame
    True,    # Reverse the Motion
)

# Keyframe head returning to neutral
fetch.set_joint("head_pan_joint", 0)
fetch.set_joint("head_tilt_joint", 0)
fetch.add_keyframe("head_pan_joint", frame)
fetch.add_keyframe("head_tilt_joint", frame)
