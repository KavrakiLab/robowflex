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
import utils
import blender_utils
import blender_load_scene as blender_scene
import blender_animate_robot as blender_robot
import blender_render_scene as blender_render
import blender_convex_hulls

if __name__ == '__main__':
    imp.reload(utils)
    imp.reload(blender_scene)
    imp.reload(blender_utils)
    imp.reload(blender_robot)
    imp.reload(blender_render)
    imp.reload(blender_convex_hulls)

    blender_robot.animate_robot(
        'r2.yml',    # Robot geometry
        ['r2_path_{:d}.yml'.format(i) for i in range(1, 7)]    # Robot path
    )

    blender_scene.add_planning_scene('package://robowflex_library/yaml/bench/world.yml')
    blender_render.add_blender_scene('package://robowflex_visualization/yaml/r2_settings.yml')

    box = blender_scene.add_box({'dimensions' : [8, 4, 4]})
    box.draw_type = 'WIRE'
    box.location = (1.76, 0.97, -0.83)
    box.hide_render = True

    mesh = bpy.data.objects["Mesh"]
    bpy.context.scene.objects.active = mesh

    bpy.ops.object.modifier_add(type = 'BOOLEAN')
    mesh.modifiers["Boolean"].object = box
