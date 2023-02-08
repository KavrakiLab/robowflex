## @package robot
#  Functions for loading and animating robots in Blender.

import bpy
import os
import subprocess
import math
import mathutils
from urdf_parser_py import urdf as URDF

import robowflex_visualization as rv


## @brief Controllable URDF described robot.
#  This class loads a URDF from the package resource URI under a Blender
#  collection.
#
class Robot:
    ## @var name
    #  Name of Robot and Blender collection.
    ## @var collection
    #  Blender collection containing loaded robot geometry.
    ## @var urdf
    #  Absolute path to the URDF file.
    ## @var xml
    #  Raw XML of the URDF file.
    ## @var robot
    #  Parsed URDF file.

    ## @brief Constructor. Loads robot and creates a collection.
    #
    #  @param name Name of Blender collection to put Robot geometry in.
    #  @param urdf URDF package resource URI to load.
    #  @param make_pretty Calls prettify functions after load.
    #
    def __init__(self, name, urdf, make_pretty = True):
        self.name = name

        self.collection = rv.utils.get_collection(name)
        if not self.collection:
            self.collection = rv.utils.make_collection(name)
            self.load_urdf(urdf)
            if make_pretty:
                self.prettify()

        else:
            self.load_urdf(urdf, False)

        self.clear_animation_data()

    ## @brief Loads the URDF as a COLLADA mesh, as well as loading the XML.
    #
    #  @param urdf URDF package resource URI to load.
    #  @param load Load the actual COLLADA mesh, not just URDF info.
    #
    def load_urdf(self, urdf, load = True):
        self.urdf_path = rv.utils.resolve_path(urdf)
        xml_output = os.path.join("/tmp",
                                  os.path.basename(self.urdf_path) + ".xml")
        dae_output = os.path.join("/tmp",
                                  os.path.basename(self.urdf_path) + ".dae")

        subprocess.check_output([
            "rosrun",
            "xacro",
            "xacro",
            self.urdf_path,
            "-o",
            xml_output,
        ])

        self.urdf_xml = open(xml_output, 'r').read()
        self.robot = URDF.Robot.from_xml_string(self.urdf_xml)

        if load:
            # Don't regenerate URDF COLLADA if it already exists.
            if not os.path.exists(dae_output):
                subprocess.check_output([
                    "rosrun",
                    "collada_urdf",
                    "urdf_to_collada",
                    xml_output,
                    dae_output,
                ])

            # Import and move into new collection.
            bpy.ops.wm.collada_import(filepath = dae_output)
            rv.utils.move_selected_to_collection(self.name)
            rv.utils.deselect_all()

    ## @brief Gets a Blender object corresponding to a link on the robot.
    #
    #  @param link_name Name of the link to find in the robot.
    #
    def get_link(self, link_name):
        return rv.utils.find_object_in_collection(self.name, link_name)

    ## @brief Get the XML description in the URDF of a link.
    #
    #  @param link_name Name of the link to find in the robot.
    #
    def get_link_xml(self, link_name):
        for link in self.robot.links:
            if link.name == link_name:
                return link

        return None

    ## @brief Get the XML description in the URDF of a joint.
    #
    #  @param joint_name Name of the joint to find in the robot.
    #
    def get_joint_xml(self, joint_name):
        for joint in self.robot.joints:
            if joint.name == joint_name:
                return joint

        return None

    ## @brief Set the value of a 1-DoF joint in the URDF.
    #
    #  Works by finding the child link of the joint and setting its relative
    #  transformation in Blender according to the axis of movement. Basically
    #  reimplementing forward kinematics.
    #
    #  @param joint_name Name of the joint to set in the robot.
    #  @param value Value of joint to set.
    #  @param interpolate If true, will attempt to make quaternions compatible
    #                     with current pose.
    #
    def set_joint(self, joint_name, value, interpolate = True):
        joint_xml = self.get_joint_xml(joint_name)
        parent_xml = self.get_link_xml(joint_xml.parent)
        link_xml = self.get_link_xml(joint_xml.child)
        link = self.get_link(joint_xml.child)

        link.rotation_mode = "QUATERNION"

        parent_l = mathutils.Matrix.Identity(4)
        if parent_xml.visual:
            parent_l = rv.utils.get_tf_origin_xml(parent_xml.visual)
            parent_l = parent_l.inverted()

        origin_l = mathutils.Matrix.Identity(4)
        if link_xml.visual:
            origin_l = rv.utils.get_tf_origin_xml(link_xml.visual)

        origin_j = rv.utils.get_tf_origin_xml(joint_xml)

        if joint_xml.type == "prismatic":
            rm = mathutils.Matrix.Translation(
                mathutils.Vector(joint_xml.axis) * value)

        else:
            rotation = mathutils.Quaternion(joint_xml.axis, value)
            rm = rotation.to_matrix()
            rm.resize_4x4()

        output = parent_l @ origin_j @ rm @ origin_l

        prior = link.rotation_quaternion.copy()

        link.location = output.to_translation()
        link.rotation_quaternion = output.to_quaternion()

        if interpolate:
            link.rotation_quaternion.make_compatible(prior)

    ## @brief Set the value of a 6-DoF joint in the URDF.
    #
    #  Assumes input transform is of the form:
    #  {
    #    'translation' : [x, y, z],
    #    'rotation' : [x, y, z, w]
    #  }
    #
    #  @param joint_name Name of the joint to set in the robot.
    #  @param tf YAML of the transform.
    #  @param interpolate If true, will attempt to make quaternions compatible
    #                     with current pose.
    #
    def set_joint_tf(self, joint_name, tf, interpolate = True):
        # Check for "virtual_joint", which isn't in the URDF
        if joint_name == "virtual_joint":
            root = self.get_root().name
            link_xml = self.get_link_xml(root)
            link = self.get_link(root)
        else:
            joint_xml = self.get_joint_xml(joint_name)
            link_xml = self.get_link_xml(joint_xml.child)
            link = self.get_link(joint_xml.child)

        link.rotation_mode = "QUATERNION"
        link.location = tf['translation']

        q = mathutils.Quaternion([tf['rotation'][3]] + tf['rotation'][:3])
        if interpolate:
            q.make_compatible(link.rotation_quaternion)

        link.rotation_quaternion = q

    ## @brief Adds a keyframe to a joint in the URDF at a frame in the
    #  animation timeline.
    #
    #  Adds keyframe actually to child link (same as set_joint()). Uses current
    #  transform of the link. Only keyframes important data path for the joint
    #  (e.g., rotation for rotation joints).
    #
    #  @param joint_name Joint to add keyframe to.
    #  @param frame Frame to add keyframe at.
    #
    def add_keyframe(self, joint_name, frame):
        if joint_name == "virtual_joint":
            root = self.get_root().name
            link_xml = self.get_link_xml(root)
            link = self.get_link(root)
        else:
            joint_xml = self.get_joint_xml(joint_name)
            link_xml = self.get_link_xml(joint_xml.child)
            link = self.get_link(joint_xml.child)

        link.keyframe_insert(data_path = "location", frame = frame)
        link.keyframe_insert(data_path = "rotation_quaternion", frame = frame)

    ## @brief Adds keyframes to animate a moveit_msgs::RobotTrajectoryMsg.
    #
    #  @param path_file YAML file for moveit_msgs::RobotTrajectoryMsg.
    #  @param fps Frames-per-second to animate the path at.
    #  @param start Frame to begin the animation at.
    #  @param reverse If true, load path from end-to-start rather than start-to-end.
    #  @param interpolate If true, interpolates quaternion from previous state.
    #
    def animate_path(self,
                     path_file,
                     fps = 60.,
                     start = 30,
                     reverse = False,
                     interpolate = False):
        path = rv.utils.read_YAML_data(path_file)

        trajectory = path["joint_trajectory"]
        names = trajectory["joint_names"]

        if "multi_dof_joint_trajectory" in path:
            mdof_trajectory = path["multi_dof_joint_trajectory"]
            mdof_names = mdof_trajectory["joint_names"]

        last_time = float(trajectory["points"][-1]["time_from_start"])
        last_frame = start

        # Add keyframe at start
        for name in names:
            self.add_keyframe(name, start)

        for i in range(len(trajectory["points"])):
            point = trajectory["points"][i]

            time = float(point["time_from_start"])
            if reverse:
                time = last_time - time

            frame = start + time * fps
            if frame > last_frame:
                last_frame = frame

            for value, name in zip(point["positions"], names):
                self.set_joint(name, value, interpolate or i > 0)
                self.add_keyframe(name, frame)

            if "multi_dof_joint_trajectory" in path:
                mdof_point = mdof_trajectory["points"][i]
                for tf, name in zip(mdof_point["transforms"], mdof_names):
                    self.set_joint_tf(name, tf, interpolate or i > 0)
                    self.add_keyframe(name, frame)

        return math.ceil(last_frame)

    ## @brief Sets the robot's state from a file with a moveit_msgs::RobotState
    #
    #  @param state_file File containing the state.
    def set_state(self, state_file):
        state = rv.utils.read_YAML_data(state_file)
        joint_state = state["joint_state"]

        names = joint_state["name"]
        position = joint_state["position"]

        for name, value in zip(names, position):
            self.set_joint(name, value)

    ## @brief Attaches an object to a link of the robot.
    #
    #  @param link_name Name of link to attach item to.
    #  @param item Blender object to attach to link.
    #  @param frame Animation frame to parent in.
    #
    def attach_object(self, link_name, item, frame):
        rv.utils.parent_object(self.get_link(link_name), item, frame)

    ## @brief Detaches an object to a link of the robot. Object must have been
    #         previously parented.
    #
    #  @param link_name Name of link to attach item to.
    #  @param item Blender object to attach to link.
    #  @param frame Animation frame to parent in.
    #
    def detach_object(self, link_name, item, frame):
        rv.utils.unparent_object(self.get_link(link_name), item, frame)

    ## @brief Cleans up robot mesh geometry and applies modifiers to improve
    #         rendering aesthetics.
    #
    def prettify(self):
        for link_xml in self.robot.links:
            # Loads original mesh.
            self.load_link_mesh(link_xml.name)

            link = self.get_link(link_xml.name)
            rv.utils.remove_doubles(link)
            # rv.utils.remove_inner_faces(link)
            rv.utils.apply_edge_split(link)
            rv.utils.apply_smooth_shade(link)

    ## @brief Loads the mesh data for a specific link on the robot.
    #
    #  If the mesh is originally a COLLADA mesh, loads and replaces current
    #  mesh data. This provides better textures if they exist, as collada_urdf
    #  does not preserve them when converting the robot.
    #
    #  @param link_name Name of the link to the load the mesh for.
    #
    def load_link_mesh(self, link_name):
        link_xml = self.get_link_xml(link_name)

        name = self.name + "_temp"
        collection = rv.utils.make_collection(name)

        if link_xml.visuals:
            visual = link_xml.visuals[0]
            geometry = visual.geometry

            # Only replace geometry if COLLADA
            if hasattr(geometry, 'filename') and ".dae" in geometry.filename:
                meshes, top = rv.primitives.add_mesh(
                    {"resource": geometry.filename})

                rv.utils.deselect_all()
                bpy.context.view_layer.objects.active = meshes[0]
                for mesh in meshes:
                    mesh.select_set(True)
                bpy.ops.object.join()
                rv.utils.move_selected_to_collection(name)
                rv.utils.deselect_all()

                old = self.get_link(link_name)
                old.data = meshes[0].data

        rv.utils.remove_collection(name)

    ## @brief Clear all animation data for this robot.
    def clear_animation_data(self):
        for link_xml in self.robot.links:
            link = self.get_link(link_xml.name)
            if link:
                link.animation_data_clear()

    ## @brief Get the root link of this robot
    def get_root(self):
        children = []
        for joint in self.robot.joints:
            xml = self.get_joint_xml(joint.name)
            children.append(xml.child)

        for link in self.robot.links:
            if link.name not in children:
                return link
