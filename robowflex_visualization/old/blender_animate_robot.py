#!/usr/bin/env python
'''Reads in a static file of transforms of different parts of the robot and animates them in blender.

'''
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
    def __init__(self, points, link_list, start_frame = 30, frame_extra_count = 30):
        '''Initialize RobotFrames.

        @param points: a list of dictionaries that contain a point (TF locations of each link) and a duration.
        @param link_list: a list of link elements (primitive or mesh)
        @param start_frame: the starting frame of the animation.
        @param frame_extra_count: the number of frames to render before the robot starts/stops moving.

        '''
        if not points:
            raise ValueError('Points should not be empty')

        self.points = points['transforms']

        for link in link_list:
            if 'visual' not in link:
                continue

            for idx, point in enumerate(self.points):
                if not link['name'] in point['point']:
                    raise ValueError('Link {} is not present in frame {:d}'.format(link['name'], idx))

        self.link_list = link_list
        self.start_frame = start_frame
        self.frame_extra_count = frame_extra_count
        self.link_to_parts = {}

    def load_meshes(self):
        '''Loads all of the robot's meshes into the scene.

        '''
        for link in self.link_list:
            link_name = link['name']
            if 'visual' not in link:
                self.link_to_parts[link_name] = []
                continue

            link_part_names = []

            # URDFs can have multiple visual elements per link.
            # Handles loading each element individually.
            for link_element in link['visual']['elements']:
                new = blender_scene.add_shape(link_element)
                element_names = []
                for i_obj in new:
                    bpy.ops.object.select_all(action = 'DESELECT')

                    # Set the first keyframe to be the initial location.
                    if 'origin' in link_element:
                        blender_utils.pose_add(i_obj, self.points[0]['point'][link_name], link_element['origin'])
                    else:
                        blender_utils.set_pose(i_obj, self.points[0]['point'][link_name])

                    i_obj.keyframe_insert(data_path = "location", index = -1)
                    i_obj.name = link_name
                    element_names.append(i_obj.name)
                link_part_names.append(element_names)

            self.link_to_parts[link_name] = link_part_names

        # Apply smooth shading and edge split to each object for aesthetics
        for obj in bpy.data.objects:
            bpy.context.scene.objects.active = obj

            blender_utils.apply_smooth()
            blender_utils.apply_edge_split()

    def animate(self, fps = 30):
        '''Adds key frames for each of the robot's links according to point data.

        '''
        # Sometimes, weird keyframes are being added way after finish. Delete those.
        for link in self.link_list:
            link_name = link['name']
            for names in self.link_to_parts[link_name]:
                for name in names:
                    i_obj = bpy.data.objects[name]
                    i_obj.animation_data_clear()
        current_frame = self.start_frame
        for point in self.points:
            bpy.context.scene.frame_set(current_frame)
            for link in self.link_list:
                if 'visual' not in link:
                    continue
                link_name = link['name']
                for names, elem in zip(self.link_to_parts[link_name], link['visual']['elements']):
                    for name in names:
                        i_obj = bpy.data.objects[name]
                        if 'origin' in elem:
                            blender_utils.pose_add(i_obj, point['point'][link_name], elem['origin'])
                        else:
                            blender_utils.set_pose(i_obj, point['point'][link_name])
                        i_obj.keyframe_insert(data_path = "location", index = -1)
                        i_obj.keyframe_insert(data_path = "rotation_quaternion", index = -1)
            current_frame += fps * point['duration']

        bpy.context.scene.render.fps = fps
        bpy.context.scene.frame_start = 0
        bpy.context.scene.frame_end = current_frame + self.frame_extra_count


def load_points(files):
    points = [utils.read_yaml_data(path_file) for path_file in files]
    combined = {'transforms': [], 'fps': points[0]['fps']}

    for point in points:
        combined['transforms'].extend(point['transforms'])

    return combined

def animate_robot(mesh_map_file, path_files):
    '''Given the data dump from robowflex::Robot::dumpGeometry and dumpPathTransforms, load the robot into blender and
    animate its path.

    WARNING: will delete all existing objects in the scene.
    '''
    blender_utils.delete_all()

    points = load_points(path_files)
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
