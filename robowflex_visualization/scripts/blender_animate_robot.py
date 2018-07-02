#!/usr/bin/env python
'''
Reads in a static file of transforms of different parts of the ur5
robot and animates them in blender.
'''
import json
import yaml
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader
import sys
import os
import time

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

def read_yaml_data(file_name):
    ''' Returns the yaml data structure of the data stored. '''
    with open(file_name) as input_file:
        if '.yaml' in file_name:
            return yaml.load(input_file.read(), Loader=Loader)


class RobotFrames(object):
    def __init__(self, points, link_map, 
                 distance_threshold=0.07, frame_extra_count=10):
        '''
        @param points: a list of dictionaries that contain a point (TF 
               locations of each link) and a duration.
        @param link_map: a map between the link name and its mesh file.
        @param distance_threshold: the minimum distance required to consider a
               frame as 'moving'
        @param frame_extra_count: the number of frames to render before the
               robot starts/stops moving.
        '''
        if not points:
            raise ValueError('Points should not be empty')
        self.points = points['transforms']
        for link_name in link_map.keys():
            for idx, point in enumerate(self.points):
                if not link_name in point['point']:
                    raise ValueError('Link ' + link_name + 'is not ' +
                                     'present in frame ' + str(idx))
        self.link_map = link_map
        self.distance_threshold = distance_threshold
        self.frame_extra_count = frame_extra_count
        self.link_to_parts = {}
        # Will be overwritten by the call to animate.
        self.step_size = 1

    def load_meshes(self):
        ''' Loads all of the robot's meshes into the scene. '''
        for link_name, mesh_file in self.link_map.items():
            # Mark all objects as imported
            old = set([obj.name for obj in bpy.data.objects])

            if '.dae' in mesh_file:
                bpy.ops.wm.collada_import(filepath=mesh_file)
            elif '.stl' in mesh_file:
                bpy.ops.import_mesh.stl(filepath=mesh_file)
            new = set([obj.name for obj in bpy.data.objects])
            imported_names = new - old
            remaining = []
            for name in imported_names:
                # For some dumb reason, loading robotiq's meshes loads in extra
                # cameras and lamps. Delete those.
                i_obj = bpy.data.objects[name]
                if 'Camera' in name or 'Lamp' in name:
                    bpy.ops.object.select_all(action='DESELECT')
                    i_obj.select = True
                    bpy.ops.object.delete()
                    continue
                i_obj.location = self.point_to_vec(self.points[0]['point'][link_name])
                i_obj.rotation_mode = 'QUATERNION'
                i_obj.rotation_quaternion = self.point_to_quat(self.points[0]['point'][link_name])
                i_obj.keyframe_insert(data_path="location", index=-1)
                i_obj.name = link_name
                remaining.append(i_obj.name)
            self.link_to_parts[link_name] = remaining

    def animate(self, fps=30):
        ''' Adds key frames for each of the robot's links according to frame data. '''
        for idx, point in enumerate(self.points):
            bpy.context.scene.frame_set(idx)
            for link_name in self.link_map:
                for name in self.link_to_parts[link_name]:
                    i_obj = bpy.data.objects[name]
                    i_obj.location = self.point_to_vec(point['point'][link_name])
                    i_obj.rotation_mode = 'QUATERNION'
                    i_obj.rotation_quaternion = self.point_to_quat(point['point'][link_name])
                    i_obj.keyframe_insert(data_path="location", index=-1)
                    i_obj.keyframe_insert(data_path="rotation_quaternion", index=-1)
        # TODO set the fps param in blender.

    @staticmethod
    def __distance_moved(frame1, frame2):
        '''
        Returns the difference between locations and quaternions between 2 states.
        '''
        dist_moved = 0.0
        for link_name in LINK_MAP:
            dist_moved += sum([abs(p[1] - p[0])
                               for p in zip(frame2[link_name]['position'] + frame2[link_name]['orientation'],
                                            frame1[link_name]['position'] + frame1[link_name]['orientation'])])
        return dist_moved

    @staticmethod
    def point_to_quat(point):
        ''' 
        Takes a yaml map of a pose and extracts the vector.
        ROS quaternions or XYZW, but Blender's are WXYZ, so reorder them.
        '''
        l = point['orientation'][3:] + point['orientation'][:3]
        return l

    @staticmethod
    def point_to_vec(point):
        ''' Takes a yaml map of a pose and extracts the quaternion. '''
        return point['position']

    def dist_from_previous(self, idx):
        ''' Convenience method for getting distance from previous frame. '''
        return self.__distance_moved(self.points[idx-1], self.points[idx])

    def get_first_moving_frame_idx(self):
        '''
        Locates the first frame where the robot moves more than DISTANCE_THRESHOLD.
        Returns the last frame if the robot never moves.
        '''
        return 0

    def get_last_moving_frame_idx(self):
        '''
        Locates the last frame that the robot moves more than DISTANCE_THRESHOLD.
        Return the first frame if the robot never moves.
        '''
        # Iterate through frames from end to start
        return len(self.points) - 1

def animate_robot(mesh_map_file, path_file):
    '''
    Given the data dump from robowflex::Robot::dumpGeometry and dumpPathTransforms,
    load the robot into blender and animate its path.
    WARNING: well delete all existing objects in the scene.
    '''

    blender_utils.delete_all()
    # Find a better way of passing this in.

    points = read_yaml_data(utils.resolvePackage(path_file))
    link_map = read_yaml_data(utils.resolvePackage(mesh_map_file))

    robot_frames = RobotFrames(points, link_map)
    robot_frames.load_meshes()
    robot_frames.animate(fps=30)

    # TODO: Load scene from yaml.
    
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

    # Find start and stop points based on frame data.
    bpy.context.scene.frame_start = robot_frames.get_first_moving_frame_idx()
    bpy.context.scene.frame_end = robot_frames.get_last_moving_frame_idx()

    # Set the output to be MP4 H264 video.
    bpy.context.scene.render.image_settings.file_format = 'FFMPEG'
    bpy.context.scene.render.ffmpeg.format = 'MPEG4'
    bpy.context.scene.render.ffmpeg.codec = 'H264'
    bpy.context.scene.render.ffmpeg.constant_rate_factor = 'HIGH'

    # Make the animation!
    #bpy.ops.render.render(animation=True)

if __name__ == '__main__':
    animate_robot('ur5.yaml', 'ur5_path.yaml')
