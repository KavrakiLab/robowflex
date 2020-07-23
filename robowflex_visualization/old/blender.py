import sys
import os
import imp
import subprocess

def initialize_path():
    '''Initialize Blender Python's system path with the system Python's paths.
    '''
    try:
        output = subprocess.check_output(
            ["python3", "-c", "import sys; print('\\n'.join(sys.path))"])
        paths = output.decode().strip().split('\n')

        for path in paths:
            if not path in sys.path:
                sys.path.append(path)
                print("Adding path {} to system path.".format(path))

    except subprocess.CalledProcessError:
        print("Unable to call system Python3")
        return ""


def find_package(package):
    '''Find a ROS package path.
    '''
    try:
        output = subprocess.check_output(["rospack", "find", package])
        return output.decode().strip()

    except subprocess.CalledProcessError:
        print("Unable to find package: `{}`".format(package))
        return ""


def load_robowflex_module(module):
    '''Load a robowflex visualization module.
    '''
    package = "robowflex_visualization"
    directory = find_package(package)

    fp, pathname, description = imp.find_module(
        module, [os.path.join(directory, "src")])
    imp.load_module("robowflex_" + module, fp, pathname, description)


if __name__ == '__main__':
    initialize_path()
    load_robowflex_module("utils")
