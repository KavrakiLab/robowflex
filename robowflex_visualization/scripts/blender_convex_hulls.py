#!/usr/bin/env python
'''Reads in a static file of transforms of different parts of the robot and makes a convex hull between
each link in order to visually illustrate TrajOpt's collision checking process.

'''
import json
import sys
import os
import time
import imp

# pyline: disable=import-error
import bpy
import bmesh

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
from blender_animate_robot import RobotFrames

class RobotHulls(RobotFrames):
    def load_hulls(self, idx1, idx2):
        '''Loads all of the robot's meshes into the scene.

        '''
        for link in self.link_list:
            link_name = link['name']
            # Mark all objects as imported
            old = set([obj.name for obj in bpy.data.objects])

            if 'visual' not in link:
                self.link_to_parts[link_name] = []
                continue

            imported_name_sets = []
            for link_element in link['visual']['elements']:
                blender_scene.add_shape(link_element)
                new = set([obj.name for obj in bpy.data.objects])
                imported_name_sets.append(new - old)
                old = new

            remaining = []
            for names, elem in zip(imported_name_sets, link['visual']['elements']):
                sublist = []
                for name in names:
                    bpy.ops.object.select_all(action = 'DESELECT')

                    # For some dumb reason, loading robotiq's meshes loads in extra
                    # cameras and lamps. Delete those.
                    i_obj = bpy.data.objects[name]

                    if 'Camera' in name or 'Lamp' in name:
                        i_obj.select = True
                        bpy.ops.object.delete()
                        continue

                    if 'origin' in elem:
                        blender_utils.pose_add(i_obj, self.points[idx1]['point'][link_name], elem['origin'])
                    else:
                        blender_utils.set_pose(i_obj, self.points[idx1]['point'][link_name])
                    i_obj.keyframe_insert(data_path = "location", index = -1)
                    i_obj.name = link_name
                    sublist.append(i_obj.name)
                remaining.append(sublist)

            # Do it again.
            old2 = set([obj.name for obj in bpy.data.objects])

            if 'visual' not in link:
                self.link_to_parts[link_name] = []
                continue

            imported_name_sets2 = []
            for link_element in link['visual']['elements']:
                blender_scene.add_shape(link_element)
                new2 = set([obj.name for obj in bpy.data.objects])
                imported_name_sets2.append(new2 - old2)
                old2 = new2

            imported_names2 = new2 - old2

            for names, elem in zip(imported_name_sets2, link['visual']['elements']):
                sublist = []
                for name in names:
                    bpy.ops.object.select_all(action = 'DESELECT')

                    # For some dumb reason, loading robotiq's meshes loads in extra
                    # cameras and lamps. Delete those.
                    i_obj = bpy.data.objects[name]

                    if 'Camera' in name or 'Lamp' in name:
                        i_obj.select = True
                        bpy.ops.object.delete()
                        continue

                    if 'origin' in elem:
                        blender_utils.pose_add(i_obj, self.points[idx2]['point'][link_name], elem['origin'])
                    else:
                        blender_utils.set_pose(i_obj, self.points[idx2]['point'][link_name])
                    i_obj.keyframe_insert(data_path = "location", index = -1)
                    i_obj.name = link_name
                    sublist.append(i_obj.name)
                remaining.append(sublist)

            # Join all of the objects for this link.
            bpy.ops.object.select_all(action = 'DESELECT')
            for names in remaining:
                for name in names:
                    i_obj = bpy.data.objects[name]
                    i_obj.select = True
            bpy.ops.object.join()
            last_name = name # the new object is last name

            # Make a convex hull out of all of the vertices
            bpy.ops.object.mode_set(mode = 'EDIT')
            mesh = bmesh.from_edit_mesh(bpy.context.object.data)
            for v in mesh.verts:
                v.select = True

            bpy.ops.mesh.convex_hull()
            bpy.ops.object.mode_set(mode = 'OBJECT')

            self.link_to_parts[link_name] = remaining


def convex_hull_robot(mesh_map_file, path_file, first_configuration=0, second_configuration=20):
    '''Given the data dump from robowflex::Robot::dumpGeometry and dumpPathTransforms, load the robot into blender
    twice and make a convex hull between each of it's links.

    WARNING: will delete all existing objects in the scene.
    '''
    blender_utils.delete_all()

    points = utils.read_yaml_data(path_file)
    link_map = utils.read_yaml_data(mesh_map_file)

    robot_frames = RobotHulls(points, link_map)
    robot_frames.load_hulls(first_configuration, second_configuration)
