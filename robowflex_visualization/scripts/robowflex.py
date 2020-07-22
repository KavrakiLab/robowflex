import importlib
import robowflex_visualization as rv

# reload modules, as Blender likes to keep things around.
importlib.reload(rv)
importlib.reload(rv.robot)
importlib.reload(rv.scene)
importlib.reload(rv.utils)

fetch = rv.robot.load("Fetch", "package://fetch_description/robots/fetch.urdf")
fetch.set_joint("torso_lift_joint", 0.2)
fetch.set_joint("shoulder_pan_joint", 1.2)
fetch.set_joint("upperarm_roll_joint", 1.2)

rv.scene.add_planning_scene("Scene", "package://robowflex_library/yaml/test_fetch.yml")
