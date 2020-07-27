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
fetch.prettify() # Make it look nice.

# Animate a simple motion plan.
fetch.animate_path("package://robowflex_visualization/yaml/fetch_path.yml")

# Load a planning scene.
scene = rv.scene.Scene("Scene", "package://robowflex_library/yaml/test_fetch.yml")
