## @package primitives
#  Functions for constructing shapes in Blender.

import os
import sys
import bpy
import robowflex_visualization as rv


def add_box(box):
    '''Creates and adds a dict of shape_msgs::SolidPrimitive box to the blender scene.
    '''
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    obj.scale = [d / 2.0 for d in box['dimensions']]
    rv.utils.set_color(obj, box)
    return obj


def add_sphere(sphere):
    '''Creates and adds a dict of shape_msgs::SolidPrimitive sphere to the blender scene.
    '''
    bpy.ops.mesh.primitive_uv_sphere_add(size = sphere['dimensions'][0])
    obj = bpy.context.active_object
    rv.utils.set_color(obj, sphere)
    return obj


def add_cylinder(cylinder):
    '''Creates and adds a dict of shape_msgs::SolidPrimitive cylinder to the blender scene.
    '''
    height = cylinder['dimensions'][0]
    radius = cylinder['dimensions'][1]
    bpy.ops.mesh.primitive_cylinder_add(radius = radius, depth = height)
    obj = bpy.context.active_object
    rv.utils.set_color(obj, cylinder)
    return obj


def add_cone(cone):
    '''Creates and adds a dict of shape_msgs::SolidPrimitive cone to the blender scene.
    '''
    height = cone['dimensions'][0]
    radius = cone['dimensions'][1]
    bpy.ops.mesh.primitive_cylinder_add(radius = radius, depth = height)
    obj = bpy.context.active_object
    rv.utils.set_color(obj, cone)
    return obj


def add_mesh(mesh):
    '''Loads and adds a mesh to the blender scene.
    '''

    # Mark all existing objects as already imported
    old = set([obj.name for obj in bpy.data.objects])

    mesh_file = rv.utils.resolve_path(mesh['resource'])
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

    top = None
    for name in imported_names:
        rv.utils.deselect_all()
        i_obj = bpy.data.objects[name]

        # If an object comes with extra cameras or lamps, delete those.
        if 'Camera' in name or 'Lamp' in name:
            i_obj.select_set(True)
            bpy.ops.object.delete()
            continue

        if not i_obj.parent:
            top = i_obj

        rv.utils.clear_alpha(i_obj)

        if "materials" in i_obj.data:
            if not i_obj.data.materials:
                rv.utils.set_color(i_obj, mesh)

        obj_list.append(i_obj)

    if 'dimensions' in mesh and top:
        top.scale = mesh['dimensions']

    return obj_list, top


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


def add_shape(shape):
    '''Add a shape_msgs::SolidPrimitive to the scene.
    '''
    if 'resource' in shape:
        meshes, top = add_mesh(shape)
        return top

    else:
        return SHAPE_MAP[shape['type']](shape)
