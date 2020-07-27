import importlib
import robowflex_visualization as rv

# reload modules, as Blender likes to keep things around.
importlib.reload(rv)
importlib.reload(rv.robot)
importlib.reload(rv.scene)
importlib.reload(rv.primitives)
importlib.reload(rv.utils)

# Load a Fetch robot.
fetch = rv.robot.Robot("Fetch", "package://fetch_description/robots/fetch.urdf")
fetch.prettify()    # Make it look nice.

# Open the Gripper
fetch.set_joint("l_gripper_finger_joint", 0.05)
fetch.set_joint("r_gripper_finger_joint", 0.05)

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

cube = scene.get_object("Cube3")
fetch.attach_object("wrist_roll_link", cube, frame)

# Close the Gripper
fetch.set_joint("l_gripper_finger_joint", 0.06)
fetch.set_joint("r_gripper_finger_joint", 0.06)
fetch.add_keyframe("l_gripper_finger_joint", frame + 30)
fetch.add_keyframe("r_gripper_finger_joint", frame + 30)

# Animate the plan in reverse
fetch.animate_path(
    "package://robowflex_visualization/yaml/fetch_pick.yml",
    15,    # FPS
    frame + 30,    # Start Frame
    True,    # Reverse the Motion
)
