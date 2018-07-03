#!/usr/bin/env python
'''Reads in a static file of transforms of different parts of the ur5 robot and animates them in blender.

'''
import json
import sys
import os
import time
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


class RobotFrames(object):
    def __init__(self, points, link_list, distance_threshold = 0.07, frame_extra_count = 10):
        '''Initialize RobotFrames.

        @param points: a list of dictionaries that contain a point (TF
               locations of each link) and a duration.
        @param link_list: a list of link elements (primitive or mesh) 
        @param distance_threshold: the minimum distance required to consider a
               frame as 'moving'
        @param frame_extra_count: the number of frames to render before the
               robot starts/stops moving.

        '''
        if not points:
            raise ValueError('Points should not be empty')
        self.points = points['transforms']
        for link in link_list:
            for idx, point in enumerate(self.points):
                if not link['name'] in point['point']:
                    raise ValueError('Link ' + link['name'] + 'is not present in frame ' + str(idx))
        self.link_list = link_list
        self.distance_threshold = distance_threshold
        self.frame_extra_count = frame_extra_count
        self.link_to_parts = {}

    def load_meshes(self):
        '''
        Loads all of the robot's meshes into the scene. 
        '''
        for link in self.link_list:
            link_name = link['name']
            # Mark all objects as imported
            old = set([obj.name for obj in bpy.data.objects])

            if 'visual' not in link:
                self.link_to_parts[link_name] = []
                continue
            for link_element in link['visual']['elements']:
                blender_scene.add_shape(link_element)

            new = set([obj.name for obj in bpy.data.objects])
            imported_names = new - old
            remaining = []
            for name in imported_names:
                # For some dumb reason, loading robotiq's meshes loads in extra
                # cameras and lamps. Delete those.
                i_obj = bpy.data.objects[name]
                if 'Camera' in name or 'Lamp' in name:
                    bpy.ops.object.select_all(action = 'DESELECT')
                    i_obj.select = True
                    bpy.ops.object.delete()
                    continue
                blender_utils.set_pose(i_obj, self.points[0]['point'][link_name])
                i_obj.keyframe_insert(data_path = "location", index = -1)
                i_obj.name = link_name
                remaining.append(i_obj.name)
            self.link_to_parts[link_name] = remaining

    def animate(self, fps = 30):
        '''Adds key frames for each of the robot's links according to point data.

        '''
        for idx, point in enumerate(self.points):
            bpy.context.scene.frame_set(idx)
            for link in self.link_list:
                link_name = link['name']
                for name in self.link_to_parts[link_name]:
                    i_obj = bpy.data.objects[name]
                    blender_utils.set_pose(i_obj, point['point'][link_name])
                    i_obj.keyframe_insert(data_path = "location", index = -1)
                    i_obj.keyframe_insert(data_path = "rotation_quaternion", index = -1)
        bpy.context.scene.render.fps = fps
        bpy.context.scene.frame_start = -self.frame_extra_count
        bpy.context.scene.frame_end = len(self.points) - 1 + self.frame_extra_count


def animate_robot(mesh_map_file, path_file):
    '''Given the data dump from robowflex::Robot::dumpGeometry and dumpPathTransforms, load the robot into blender and
    animate its path.

    WARNING: well delete all existing objects in the scene.

    '''
    blender_utils.delete_all()

    points = utils.read_yaml_data(path_file)
    link_map = utils.read_yaml_data(mesh_map_file)

    robot_frames = RobotFrames(points, link_map)
    robot_frames.load_meshes()
    robot_frames.animate(fps = points['fps'])

    # TODO: auto-adjust the camera position until the full motion lies within
    # the frame? Will need to get bounding box of the entire motion, then
    # project back to the active camera.
    #
    # E = cam_location
    # n = norm(1/4 * (sum(frame_vectors[i])))
    # Q = 1/4 * (sum(frame_vectors[i])) + cam_location
    # frame_radius = min dist(1/2 * (cam_frame[i] + cam_frame[j]) - Q)
    # Iterate over all frames and calculate an AABB for all links.
    # Minimize sum(abs(dist(project_point(v) - Q) - frame_radius)) for v in AABB
