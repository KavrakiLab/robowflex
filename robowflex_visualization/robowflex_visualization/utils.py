'''Helper functions for the Robowflex Visualization library.
'''

import bpy
import mathutils

import math
import os.path
import subprocess
import logging
import yaml


def resolve_package(path):
    '''Resolves `package://` URLs to their canonical form. The path does not need
    to exist, but the package does. Can be used to write new files in packages.

    Returns "" on failure.
    '''
    if not path:
        return ""

    package_name = ""
    package_path1 = ""
    PREFIX = "package://"
    if PREFIX in path:
        path = path[len(PREFIX):]    # Remove "package://"
        if "/" not in path:
            package_name = path
            path = ""
        else:
            package_name = path[:path.find("/")]
            path = path[path.find("/"):]

        package_path1 = subprocess.check_output(
            ["rospack", "find", package_name]).decode().strip()

    elif "~" in path:
        path = os.path.expanduser(path)

    new_path = os.path.realpath(package_path1 + path)
    return new_path


def resolve_path(path):
    '''Resolves `package://` URLs and relative file paths to their canonical form.
    Returns "" on failure.
    '''
    full_path = resolve_package(path)
    if not os.path.exists(full_path):
        logging.warn("File {} does not exist".format(full_path))
        return ""
    return full_path


def apply_smooth_shade(item):
    '''Applies smooth shading to the provided object.
    '''
    deselect_all()
    set_active(item)
    item.select_set(True)

    if item.type == 'MESH':
        bpy.ops.object.shade_smooth()

    deselect_all()


def apply_edge_split(item, angle = math.pi / 6):
    '''Applies the edge-split modifier to the provided object.
    '''
    deselect_all()
    set_active(item)
    item.select_set(True)
    try:
        bpy.ops.object.modifier_add(type = 'EDGE_SPLIT')
        bpy.context.object.modifiers["EdgeSplit"].split_angle = angle
    except:
        pass

    deselect_all()


def find_object_in_collection(coll_name, item_name):
    '''Retrieves an object by name from a collection.
    '''
    collection = bpy.data.collections.get(coll_name)
    for item in collection.objects:
        if item.name == item_name:
            return item

    return None


def move_selected_to_collection(name):
    '''Moves selected objects to collection.
    '''
    collection = bpy.data.collections.get(name)
    selected = bpy.context.selected_objects
    for select in selected:
        old = find_collection(select)

        collection.objects.link(select)
        old.objects.unlink(select)


def find_collection(item):
    '''Find the current collection of an object.
    '''
    collections = item.users_collection
    if len(collections) > 0:
        return collections[0]
    return bpy.context.scene.collection


def remove_collection(name, remove_objects = True):
    '''Removes a collection and all its contents.
    '''
    collection = bpy.data.collections.get(name)

    if collection:
        if remove_objects:
            obs = [o for o in collection.objects if o.users == 1]
            while obs:
                bpy.data.objects.remove(obs.pop())

        bpy.data.collections.remove(collection)


def make_collection(name):
    '''Creates a new collection, deleting collections with the same name.
    '''

    remove_collection(name)

    collection = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(collection)
    return collection


def deselect_all():
    bpy.ops.object.select_all(action = 'DESELECT')


def set_active(item):
    bpy.context.view_layer.objects.active = item


def add_material(item, material):
    '''Adds a material to an object.
    '''
    if item.data.materials:
        # assign to 1st material slot
        item.data.materials[0] = material

    else:
        # no slots
        item.data.materials.append(material)


def pose_to_quat(pose):
    '''Takes a pose dict and extracts the orientation quaternion. ROS quaternions or XYZW, but Blender's are WXYZ, so
    reorder them.
    '''

    if 'x' in pose['orientation']:
        q = pose['orientation']
        return mathutils.Quaternion([q['w'], q['x'], q['y'], q['z']])

    if isinstance(pose['orientation'][0], str):
        # Means there's a NAN somewhere.
        return mathutils.Quaternion((1.0, 0.0, 0.0, 0.0))
    else:
        return mathutils.Quaternion(
            pose['orientation'][3:] + pose['orientation'][:3])


def pose_to_vec(pose):
    '''Takes a pose dict and extracts the position vector.
    '''
    if 'x' in pose['position']:
        v = pose['position']
        return mathutils.Vector([v['x'], v['y'], v['z']])

    return mathutils.Vector(pose['position'])


def pose_add(obj, pose1, pose2):
    '''Adds the second pose after the first. For something like link origins, the
       joint pose should be passed first, then the link origin
    '''

    # Get the objects for the pose vectors and quaternions.
    p1_vec = pose_to_vec(pose1)
    q1 = pose_to_quat(pose1)
    p2_vec = pose_to_vec(pose2)
    q2 = pose_to_quat(pose2)

    # Rotate pose2's vec by pose1's quaternion.
    p2_vec.rotate(q1)
    obj.location = p1_vec + p2_vec

    # Apply pose2's quaternion to the existing rotation.
    q2.rotate(q1)
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = q2


def set_pose(obj, pose):
    '''Sets the pose of a blender object by passing in a pose dict.
    '''

    obj.location = pose_to_vec(pose)
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = pose_to_quat(pose)


def read_YAML_data(file_name):
    '''Returns the yaml data structure of the data stored.
    '''
    full_name = resolve_path(file_name)
    if not full_name:
        logging.warn('Cannot open {}'.format(file_name))
        return None
    with open(full_name) as input_file:
        return yaml.load(input_file.read())
