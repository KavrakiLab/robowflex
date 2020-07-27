# Robowflex Visualization
Visualize motion plans and robots in tandem with `robowflex_library` in [Blender](https://www.blender.org/).

## Requirements
1. Blender must be installed with COLLADA loading support (`*.dae`).
2. `python3-yaml` for loading YAML files.
3. The ROS package `collada_urdf` for translating URDFs into COLLADA files.

`package://` URIs are supported by calling the `rospack` command-line tool.
Blender should be run in an environment with your ROS workspace sourced.

## Design
This package provides the `robowflex_visualization` Python library intended for use within Blender.
All code for this library is within the `robowflex_visualization` folder.

## Usage

1. Dump a trajectory to a YAML file via the `robowflex::path::toYAMLFile()` function.
An example of this can be found in `robowflex_library/scripts/fetch_test.cpp`:
```cpp
# fetch_test.cpp
...
// Generate a plan
planning_interface::MotionPlanResponse res = planner->plan(scene, request.getRequest());

// Output path to a file for visualization.
path::toYAMLFile("fetch_path.yml", *res.trajectory_);
...
```

Planning scenes can be edited manually by writing the YAML file (an example is in `robowflex_library/yaml/test.yml`), or by calling `robowflex::Scene::toYAMLFile()` on a loaded planning scene.

2. Edit the provided file, `scripts/robowflex.py` to use your desired files.
```py
# robowflex.py

import robowflex_visualization as rv
...

# Load a robot under the name `Fetch`
fetch = rv.robot.Robot("Fetch", "package://fetch_description/robots/fetch.urdf")

# Animate a trajectory
fetch.animate_path("package://robowflex_visualization/yaml/fetch_path.yml")

# Add a planning scene.
scene = rv.scene.Scene("Scene", "package://robowflex_library/yaml/test_fetch.yml")
```

3. Open blender from the current terminal and open the provided `robowflex.blend` Blender scene.
   To the left should be a baked in Python script (if there is no script, it is also available at `scripts/blender.py`).
   This script sets up the system path for the built-in Blender Python to look at your Python3 system path as well as load the `robowflex_visualization` Python module.
   Run this script to execute whatever code you have in `scripts/robowflex.py`.
   You can change what script is called.
   
4. Lather, rinse, repeat!
   You can edit code outside the Blender editor, everything should be reload appropriately.

## Examples

### `robowflex_library/scripts/fetch_test.cpp`
You can render still images of robots.
<img style="width:100%;max-width:720px;" src="https://s3.amazonaws.com/zk-bucket/robowflex/fetch.png" />

### `robowflex_library/scripts/ur5_test.cpp`
As well as full animations of motion plans.
<img style="width:100%;max-width:720px;" src="https://s3.amazonaws.com/zk-bucket/robowflex/ur5.gif" />
