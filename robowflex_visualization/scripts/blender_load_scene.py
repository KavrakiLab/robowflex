#!/usr/bin/env python
'''A module for reading in MoveIt/Robowflex Scenes to Blender.

'''

import os
import sys
import random
# pylint: disable=import-error
import bpy

CURRENT_DIRECTORY = os.getcwd()
if not CURRENT_DIRECTORY in sys.path:
    sys.path.append(CURRENT_DIRECTORY)

# pylint: disable=wrong-import-position
import blender_utils
import utils


def set_color(obj, element):
    if 'color' in element:
        # TODO: figure out a better way to make new materials?
        mat = bpy.data.materials.new(name = str(random.randint(1, 100000)))
        mat.diffuse_color = element['color'][:3]
        if len(element['color']) > 3:
            # An alpha value was provided
            mat.alpha = element['color'][3]

        blender_utils.add_mat_to_obj(obj, mat)


def add_box(box):
    '''Creates and adds a dict of shape_msgs::SolidPrimitive box to the blender scene.

    '''
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    obj.scale = [d / 2.0 for d in box['dimensions']]
    set_color(obj, box)
    return obj


def add_sphere(sphere):
    '''Creates and adds a dict of shape_msgs::SolidPrimitive sphere to the blender scene.

    '''
    bpy.ops.mesh.primitive_uv_sphere_add(size = sphere['dimensions'][0])
    obj = bpy.context.active_object
    set_color(obj, sphere)
    return obj


def add_cylinder(cylinder):
    '''Creates and adds a dict of shape_msgs::SolidPrimitive cylinder to the blender scene.

    '''
    height = cylinder['dimensions'][0]
    radius = cylinder['dimensions'][1]
    bpy.ops.mesh.primitive_cylinder_add(radius = radius, depth = height)
    obj = bpy.context.active_object
    set_color(obj, cylinder)
    return obj


def add_cone(cone):
    '''Creates and adds a dict of shape_msgs::SolidPrimitive cone to the blender scene.

    '''
    height = cone['dimensions'][0]
    radius = cone['dimensions'][1]
    bpy.ops.mesh.primitive_cylinder_add(radius = radius, depth = height)
    obj = bpy.context.active_object
    set_color(obj, cone)
    return obj


SHAPE_MAP = {
    'box'     : add_box,
    1         : add_box,
    'sphere'  : add_sphere,
    2         : add_sphere,
    'cylinder': add_cylinder,
    3         : add_cylinder,
    'cone'    : add_cone,
    4         : add_cone
} # yapf: disable


def add_mesh(mesh):
    '''Loads and adds a mesh to the blender scene.

    '''

    # Mark all existing objects as already imported
    old = set([obj.name for obj in bpy.data.objects])

    mesh_file = utils.resolvePackage(mesh['resource'])
    if '.dae' in mesh_file.lower():
        bpy.ops.wm.collada_import(filepath = mesh_file)
    elif '.stl' in mesh_file.lower():
        bpy.ops.import_mesh.stl(filepath = mesh_file)
    elif '.ply' in mesh_file.lower():
        bpy.ops.import_mesh.ply(filepath = mesh_file)
    else:
        return None

    # Compare all imported objects to what we saw before loading the mesh.
    new = set([obj.name for obj in bpy.data.objects])
    imported_names = new - old
    obj_list = []

    for name in imported_names:
        bpy.ops.object.select_all(action = 'DESELECT')
        i_obj = bpy.data.objects[name]

        # For some dumb reason, loading robotiq's meshes loads in extra
        # cameras and lamps. Delete those.
        if 'Camera' in name or 'Lamp' in name:
            i_obj.select = True
            bpy.ops.object.delete()
            continue

        if "materials" in i_obj.data:
            if not i_obj.data.materials:
                set_color(i_obj, mesh)

        if 'dimensions' in mesh:
            i_obj.scale = mesh['dimensions']

        obj_list.append(i_obj)

    return obj_list


def add_shape(shape):
    '''Add a shape_msgs::SolidPrimitive to the scene.

    '''
    if 'resource' in shape:
        return add_mesh(shape)
    else:
        return [SHAPE_MAP[shape['type']](shape)]


def add_collision_objects(collision_objects):
    '''Adds a moveit_msgs::CollisionObject to the blender scene.

    '''
    for coll_obj in collision_objects:
        if 'primitives' in coll_obj:
            shapes = coll_obj['primitives']
            poses = coll_obj['primitive_poses']
        elif 'meshes' in coll_obj:
            shapes = coll_obj['meshes']
            poses = coll_obj['mesh_poses']
        for shape, pose in zip(shapes, poses):
            if not 'color' in shape:
                shape['color'] = (0.0, 0.9, 0.2)    # MoveIt Green.
            obj = add_shape(shape)
            for i_obj in obj:
                blender_utils.set_pose(i_obj, pose)


def add_planning_scene_world(world):
    '''Adds a moveit_msgs::PlanningSceneWorld to the blender scene.

    '''
    # TODO: add Octomap
    add_collision_objects(world['collision_objects'])


def add_planning_scene(scenefile):
    '''Reads in a scene yaml (output by robowflex_library) file and adds all collision objects to the blender scene.

    '''
    scene = utils.read_yaml_data(scenefile)
    add_planning_scene_world(scene['world'])
    # TODO: be able to handle assigning object_colors, which are stored at this level.
