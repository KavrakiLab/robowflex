## @package scene
#  Functions for loading and animating scenes in Blender.

import bpy
import robowflex_visualization as rv


## @brief Container for MoveIt planning scenes in Blender.
#  This class loads mesh and primitive resources described in a planning scene
#  and stores them in a Blender collection.
class Scene:

    ## @brief Constructor. Loads scene and creates a collection.
    #
    #  @param name Name of Blender collection to put scene geometry in.
    #  @param scene_file Scene file YAML package resource URI to load.
    #
    def __init__(self, name, scene_file):
        self.name = name
        self.collection = rv.utils.make_collection(name)

        self.shapes = {}
        self.load_scene(scene_file)

    def load_scene(self, scene_file):
        self.filepath = resolved = rv.utils.resolve_path(scene_file)
        self.yaml = rv.utils.read_YAML_data(scene_file)

        self.add_collision_objects(self.yaml["world"]["collision_objects"])

    def add_collision_objects(self, cos):
        for co in cos:
            if 'primitives' in co:
                shapes = co['primitives']
                poses = co['primitive_poses']
            elif 'meshes' in co:
                shapes = co['meshes']
                poses = co['mesh_poses']

            for shape, pose in zip(shapes, poses):
                self.add_shape(co["id"], shape, pose)

    def add_shape(self, name, shape, pose):
        if not 'color' in shape:
            shape['color'] = (0.0, 0.9, 0.2)    # MoveIt Green.

        obj = rv.primitives.add_shape(shape)
        obj.name = name

        self.shapes[name] = obj

        rv.utils.deselect_all()
        rv.utils.set_pose(obj, pose)
        rv.utils.select_all_children(obj)
        rv.utils.move_selected_to_collection(self.name)
        rv.utils.deselect_all()

    def get_object(self, name):
        return self.shapes[name]
