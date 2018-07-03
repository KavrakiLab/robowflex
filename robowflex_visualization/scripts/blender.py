#!/usr/bin/env python

import sys
import os
import imp

# pylint: disable=import-error
import bpy

# Blender is stupid and won't load modules from the current directory,
# so in order to load, say, blender_utils.py, we have to specify the directory.
CURRENT_DIRECTORY = os.getcwd()

# Let python find the blender_utils directory.
if not CURRENT_DIRECTORY in sys.path:
    sys.path.append(CURRENT_DIRECTORY)
    print(sys.path)

# pylint: disable=wrong-import-position
import blender_utils
import utils
import blender_load_scene as blender_scene
import blender_animate_robot as blender_robot
import blender_render_scene as blender_render

if __name__ == '__main__':
    imp.reload(utils)
    imp.reload(blender_scene)
    imp.reload(blender_utils)
    imp.reload(blender_robot)
    imp.reload(blender_render)
    blender_robot.animate_robot('../yaml/ur5.yaml', '../yaml/ur5_path.yaml')
    blender_scene.add_planning_scene('package://robowflex_library/yaml/test.yml')
    blender_render.add_blender_scene('../yaml/render_settings.yaml')
