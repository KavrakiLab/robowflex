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
        self.shapes = {}

        self.collection = rv.utils.get_collection(name)

        if not self.collection:
            # Create scene
            self.collection = rv.utils.make_collection(name)
            self.load_scene(scene_file)

        else:
            # Populate based on existing scene
            for item in self.collection.objects:
                self.shapes[item.name] = item

    ## @brief Loads a YAML moveit_msgs::PlanningScene into Blender.
    #
    #  @param scene_file YAML package URI to load.
    #
    def load_scene(self, scene_file):
        self.filepath = resolved = rv.utils.resolve_path(scene_file)
        self.yaml = rv.utils.read_YAML_data(scene_file)

        if "world" in self.yaml and self.yaml["world"]["collision_objects"]:
            for co in self.yaml["world"]["collision_objects"]:
                self.add_collision_object(co)

        if "robot_state" in self.yaml:
            if "attached_collision_objects" in self.yaml["robot_state"].keys():
               for cao in self.yaml["robot_state"]["attached_collision_objects"]:
                   self.add_collision_object(cao["object"])

    ## @brief Adds a collision object (moveit_msgs::CollisionObject) to the scene.
    #
    #  @param co Collision object.
    #
    def add_collision_object(self, co):
        if 'primitives' in co:
            shapes = co['primitives']
            poses = co['primitive_poses']
        elif 'meshes' in co:
            shapes = co['meshes']
            poses = co['mesh_poses']

        for shape, pose in zip(shapes, poses):
            self.add_shape(co["id"], shape, pose)

    ## @brief Adds a shape (either shape_msgs::SolidPrimitive or shape_msgs::Mesh) to the scene.
    #
    #  @param name Name of the object.
    #  @param shape Shape to add.
    #  @param pose Pose relative to world of the object.
    #
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

    ## @brief Retrieve a named object in the scene.
    #
    #  @param name Name of object to retrieve.
    def get_object(self, name):
        return self.shapes[name]
