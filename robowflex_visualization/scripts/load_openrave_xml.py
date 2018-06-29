#!/usr/bin/env python
'''
A modules for reading in openRAVE XML Environments and
(eventually, not currently) robots.
'''

import os
import sys
import random
import xml.etree.ElementTree as ET
import zipfile
#pylint: disable=import-error
import bpy

current_path = os.path.dirname(bpy.data.filepath)
if not current_path in sys.path:
    sys.path.append(current_path)
scripts_path = os.getcwd()
if not scripts_path in sys.path:
    sys.path.append(scripts_path)

import blender_utils as util

DATA_PATH = os.getcwd() + '/../../OptPlanners_OpenRAVE/scripts/data/envs/'

def make_tuple(node, child_elem):
    ''' Makes a double list out of the common xml list format. '''
    return tuple(float(x) for x in node.find(child_elem).text.split(' '))

def make_mat(geom_node):
    ''' Makes a blender material from the geometry node description. '''
    color_tuple = make_tuple(geom_node, 'diffuseColor')
    mat = bpy.data.materials.new(name=str(random.randint(1, 100000)))
    mat.diffuse_color = color_tuple
    return mat

def add_openrave_box(box):
    ''' Creates and adds a box to the blender scene. '''
    extents = make_tuple(box, 'extents')
    mat = make_mat(box)
    bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.active_object
    obj.location = (0, 0, 0)
    obj.scale = extents
    util.add_mat_to_obj(obj, mat)

def add_openrave_cylinder(cylinder):
    ''' Creates and adds a cylinder to the scene. '''
    translation = make_tuple(cylinder, 'translation')
    rotationaxis = make_tuple(cylinder, 'rotationaxis')
    radius = float(cylinder.find('radius').text)
    height = float(cylinder.find('height').text)
    mat = make_mat(cylinder)
    bpy.ops.mesh.primitive_cylinder_add(radius=radius, depth=height, location=translation)
    util.add_mat_to_obj(bpy.context.active_object, mat)

def add_openrave_stl(stl_node):
    mat = bpy.data.materials.new(name=str(random.randint(1, 100000)))
    mat.diffuse_color = (0.058, 0.000304, 0.089)
    bpy.ops.import_mesh.stl(filepath=DATA_PATH + stl_node.find('Data').text)
    obj = bpy.context.active_object
    obj.location = (0, 0, 0)
    util.add_mat_to_obj(obj, mat)

def add_openrave_trimesh(trimesh_node):
    mat = bpy.data.materials.new(name=str(random.randint(1, 100000)))
    mat.diffuse_color = (0.058, 0.000304, 0.089)
    filename = trimesh_node.find('Data').text
    if filename.endswith('stl'):
        add_openrave_stl(trimesh_node)
    else:
        print('Cannot currently handle other trimeshes.')

def add_openrave_kinbody(kinbody_node):
    # TODO: set the newly added object to be at 'transform'
    transform = make_tuple(kinbody_node, 'Translation')
    print('For object {}, transform is {}'.format(kinbody_node.attrib['name'], transform))
    add_openrave_element(kinbody_node.find('Body'))

FN_MAP = {'box' : add_openrave_box,
          'cylinder' : add_openrave_cylinder,
          'trimesh': add_openrave_stl}

def add_openrave_element(root):
    if root.tag == 'KinBody':
        add_openrave_kinbody(root)
    elif root.tag == 'Body':
        for child in root:
            add_openrave_element(child)
    elif root.tag == 'Environment':
        for child in root:
            add_openrave_element(child)
    elif root.tag == 'Geom':
        FN_MAP[root.attrib['type']](root)
    
if __name__ == '__main__':
    util.delete_all()
    tree = ET.parse(DATA_PATH + 'wam7_table_short_column.env.xml')
    add_openrave_element(tree.getroot())

    # Extra stuff: add a plane in at each point and render an image
    '''
    start_y = -0.66
    bpy.ops.mesh.primitive_plane_add(location=(0, start_y, -0.26), rotation=(3.1415926/2, 0, 0))
    mat = bpy.data.materials.new(name=str(random.randint(1, 100000)))
    mat.diffuse_color=(0.101, 0.800, 0.071)
    mat.use_transparency = True
    mat.alpha = 0.388
    util.add_mat_to_obj(bpy.context.active_object, mat)
    bpy.context.active_object.scale = (1.0, 0.61, 1.0)
    bpy.context.active_object.keyframe_insert(data_path='location', index=-1)
    for i in range(66):
        bpy.context.scene.frame_set(i)
        bpy.context.active_object.location = (0, start_y + i * 0.02, -0.26)
        bpy.context.active_object.keyframe_insert(data_path='location', index=-1)
    util.add_camera((2.41642, 2.28693, 0.82967), (0.294, 0.211, 0.532, 0.766))
    '''
