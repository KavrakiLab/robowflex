import bpy
import sys
import os
import importlib.util
import subprocess
"""Run this script in order to execute:
        `robowflex_visualization/src/robowflex.py`

   This script contains some utility functions for loading system Python paths
   and finding the ROS package. You can change which script is ran by changing
   the arguments to `load_ROS_module` at the bottom of the file.

   Moreover, the `robowflex_visualization/src` directory is added to the path,
   so that the various helper modules can be found.

   You should put all of your visualization code in whatever file is loaded at
   the bottom.
"""


def add_path(path):
    """Add a path to the system search path.
    """
    if not path in sys.path:
        sys.path.append(path)
        print("Adding path {} to system path.".format(path))


def initialize_path():
    """Initialize Blender Python's system path with the system Python's paths.
    """
    try:
        output = subprocess.check_output(
            ["python3", "-c", "import sys; print('\\n'.join(sys.path))"])
        paths = output.decode().strip().split('\n')

        for path in paths:
            add_path(path)

    except subprocess.CalledProcessError:
        print("Unable to call system Python3")
        return ""


def find_package(package):
    """Find a ROS package path.
    """
    try:
        output = subprocess.check_output(["rospack", "find", package])
        return output.decode().strip()

    except subprocess.CalledProcessError:
        print("Unable to find package: `{}`".format(package))
        return ""


def initialize_robowflex_path():
    """Adds the robowflex_visualization/src folder to search path.
    """
    directory = find_package("robowflex_visualization")
    add_path(directory)


def load_ROS_module(module_name,
                    module_file,
                    package = "robowflex_visualization"):
    """Load a robowflex visualization module.
    """
    directory = find_package(package)

    spec = importlib.util.spec_from_file_location(    #
        module_name, os.path.join(directory, module_file))

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)


if __name__ == '__main__':
    bpy.ops.outliner.orphans_purge()
    initialize_path()
    initialize_robowflex_path()
    load_ROS_module("robowflex", "scripts/robowflex.py")
