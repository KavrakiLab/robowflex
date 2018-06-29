#!/usr/bin/env python
'''
Reads in a static file of transforms of different parts of the ur5
robot and animates them in blender.
'''
import json
import yaml
import sys
import os

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
            return yaml.loads(input_file.read())

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
        self.points = points
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

    def smooth_frames(self):
        ''' Removed duplicate frames, most likely there due to tf issues. '''
        self.frames = [frame for idx, frame in enumerate(self.frames)
                       if idx == 0 or self.dist_from_previous(idx) > 0.00001]

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
                               for p in zip(frame2[link_name], frame1[link_name])])
        return dist_moved

    @staticmethod
    def point_to_quat(point):
        ''' Takes a yaml map of a pose and extracts the vector. '''
        return point['orientation']

    @staticmethod
    def point_to_vec(point):
        ''' Takes a yaml map of a pose and extracts the quaternion. '''
        return point['position']

    def dist_from_previous(self, idx):
        ''' Convience method for getting distance from previous frame. '''
        return self.__distance_moved(self.frames[idx-1], self.frames[idx])

    def get_first_moving_frame_idx(self):
        '''
        Locates the first frame where the robot moves more than DISTANCE_THRESHOLD.
        Returns the last frame if the robot never moves.
        '''
        for idx in range(0, len(self.frames) - 2):
            dist_moved = self.dist_from_previous(idx + 1)
            if dist_moved > self.distance_threshold:
                return max(0, self.step_size * (idx - self.frame_extra_count))
        return (len(self.frames) - 1) * self.step_size

    def get_last_moving_frame_idx(self):
        '''
        Locates the last frame that the robot moves more than DISTANCE_THRESHOLD.
        Return the first frame if the robot never moves.
        '''
        # Iterate through frames from end to start
        for idx in range(len(self.frames) - 1, 1, -1):
            dist_moved = self.dist_from_previous(idx)
            if dist_moved > self.distance_threshold:
                return min((len(self.frames) - 1) * self.step_size,
                           (idx + self.frame_extra_count) * self.step_size)
        return 0

# Hardcoded link names to mesh files
# TODO: given an un-xacro'd urdf, find all links and their visual_geometries.
# Make the below link_map using this.
WS_DIR = '/home/brycew/ur_wksp/src/'
GRIP_DIR = WS_DIR + 'robotiq_85_gripper/robotiq_85_description/meshes/visual/'
UR_DIR = WS_DIR + 'universal_robot/ur_description/meshes/ur5/visual/'
FTS_DIR = WS_DIR + 'universal_robot/robotiq_force_torque_sensor/meshes/visual/'

LINK_MAP = {
    'robotiq_85_left_finger_tip_link' : GRIP_DIR + 'robotiq_85_finger_tip_link.dae',
    'robotiq_85_right_finger_tip_link' : GRIP_DIR + 'robotiq_85_finger_tip_link.dae',
    'robotiq_85_left_finger_link' : GRIP_DIR + 'robotiq_85_finger_link.dae',
    'robotiq_85_right_finger_link' : GRIP_DIR + 'robotiq_85_finger_link.dae',
    'robotiq_85_right_knuckle_link' : GRIP_DIR + 'robotiq_85_knuckle_link.dae',
    'robotiq_85_left_knuckle_link' : GRIP_DIR + 'robotiq_85_knuckle_link.dae',
    'robotiq_85_right_inner_knuckle_link' : GRIP_DIR + 'robotiq_85_inner_knuckle_link.dae',
    'robotiq_85_left_inner_knuckle_link' : GRIP_DIR + 'robotiq_85_inner_knuckle_link.dae',
    'robotiq_85_base_link' : GRIP_DIR + 'robotiq_85_base_link.dae',
    'fts_robotside' : FTS_DIR + 'robotiq_fts300.stl',
    'wrist_3_link' : UR_DIR + 'wrist3.dae',
    'wrist_2_link' : UR_DIR + 'wrist2.dae',
    'wrist_1_link' : UR_DIR + 'wrist1.dae',
    'upper_arm_link' : UR_DIR + 'upperarm.dae',
    'shoulder_link' : UR_DIR + 'shoulder.dae',
    'forearm_link' : UR_DIR + 'forearm.dae',
    'base' : UR_DIR + 'base.dae',
    #'box_link' : '',
}

def animate_robot(mesh_map_file, path_file):
    '''
    Given the data dump from robowflex::Robot::dumpGeometry and dumpPathTransforms,
    load the robot into blender and animate its path.
    '''

    blender_utils.delete_all()
    # Find a better way of passing this in.

    points = read_yaml_data(utils.resolvePackage(path_file))
    link_map = read_yaml_data(utils.resolvePackage(mesh_map_file))

    robot_frames = RobotFrames(points, link_map)
    robot_frames.load_meshes()
    robot_frames.smooth_frames()
    robot_frames.animate(fps=30)

    ### Tweak the settings to something that I liked. ###
    blender_utils.add_sun((0.0, 0.0, 2.4), shadow=True)
    blender_utils.add_camera(location=(-1.8739, 0.19844, 1.92992),
                    rotation=(0.605, 0.327, -0.338, -0.643))
    if not bpy.data.worlds:
        bpy.ops.world.new()
    bpy.data.worlds[0].horizon_color = (0.0, 0.283, 0.823) #util.hex_to_rgb('0091ea')

    # Make a plane to show the robot's shadows.
    # Make alpha of plane 0
    bpy.ops.mesh.primitive_plane_add(radius=4.0)
    obj = bpy.context.active_object
    obj.location = (0, 0, 0)
    floor_mat = bpy.data.materials.new(name='floor-material')
    floor_mat.use_only_shadow = True
    blender_utils.add_mat_to_obj(obj, floor_mat)

    # Add environment lighting, and a little ambient occusion
    world = bpy.data.worlds['World']
    world.light_settings.use_environment_light = True
    world.light_settings.environment_energy = 0.63
    world.light_settings.use_ambient_occlusion = True
    world.light_settings.ao_factor = 0.44
    world.light_settings.samples = 15

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

    # TODO for the URs, find a way to subdivide the caps so they don't look super wrinkly
    # With the renaming of objects to links, the sub parts of the link are always the same.
    # Hardcode them somewhere.

    # Set the output to be MP4 H264 video.
    bpy.context.scene.render.image_settings.file_format = 'FFMPEG'
    bpy.context.scene.render.ffmpeg.format = 'MPEG4'
    bpy.context.scene.render.ffmpeg.codec = 'H264'
    bpy.context.scene.render.ffmpeg.constant_rate_factor = 'HIGH'

    # Make the animation!
    #bpy.ops.render.render(animation=True)

if __name__ == '__main__':
    main()
