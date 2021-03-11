## @package robot
#  Functions for loading and animating robots in Blender.

import bpy
import os
import subprocess
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
    #
    def __init__(self, name, urdf):
        self.name = name
        self.collection = rv.utils.make_collection(name)

        self.load_urdf(urdf)

    ## @brief Loads the URDF as a COLLADA mesh, as well as loading the XML.
    #
    #  @param urdf URDF package resource URI to load.
    #
    def load_urdf(self, urdf):
        self.urdf = resolved = rv.utils.resolve_path(urdf)
        output = os.path.join("/tmp", os.path.basename(resolved) + ".dae")

        # Don't regenerate URDF COLLADA if it already exists.
        if not os.path.exists(output):
            subprocess.check_output(
                ["rosrun", "collada_urdf", "urdf_to_collada", resolved, output])

        # Import and move into new collection.
        bpy.ops.wm.collada_import(filepath = output)
        rv.utils.move_selected_to_collection(self.name)
        rv.utils.deselect_all()

        # Read in the URDF XML.
        self.xml = open(self.urdf, 'r').read()
        self.robot = URDF.Robot.from_xml_string(self.xml)

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

    ## @brief Set the value of a joint in the URDF.
    #
    #  Currently limited to 1-DoF joints (e.g., prismatic, continuous, or
    #  revolute). Works by finding the child link of the joint and setting its
    #  relative transformation in Blender according to the axis of movement.
    #
    #  TODO: Handle multi-dof joints
    #
    #  @param joint_name Name of the joint to set in the robot.
    #  @param value Value of joint to set.
    #
    def set_joint(self, joint_name, value):
        joint_xml = self.get_joint_xml(joint_name)
        link_xml = self.get_link_xml(joint_xml.child)
        link = self.get_link(joint_xml.child)

        # Assuming Axis is one value...
        # TODO: Handle arbitrary axis
        if (joint_xml.type == "prismatic"):
            link.location = [
                joint_xml.origin.xyz[i] + joint_xml.axis[i] * value
                for i in range(3)
            ]

        else:
            link.rotation_euler = [
                joint_xml.origin.rpy[i] + joint_xml.axis[i] * value
                for i in range(3)
            ]

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
        joint_xml = self.get_joint_xml(joint_name)
        link = self.get_link(joint_xml.child)

        if (joint_xml.type == "prismatic"):
            link.keyframe_insert(data_path = "location", frame = frame)
        else:
            link.keyframe_insert(data_path = "rotation_euler", frame = frame)

    ## @brief Adds keyframes to animate a moveit_msgs::RobotTrajectoryMsg.
    #
    #  @param path_file YAML file for moveit_msgs::RobotTrajectoryMsg.
    #  @param fps Frames-per-second to animate the path at.
    #  @param start Frame to begin the animation at.
    #  @param reverse If true, load path from end-to-start rather than start-to-end.
    #
    def animate_path(self, path_file, fps = 60., start = 30, reverse = False):
        path = rv.utils.read_YAML_data(path_file)

        trajectory = path["joint_trajectory"]
        names = trajectory["joint_names"]

        last_time = float(trajectory["points"][-1]["time_from_start"])
        last_frame = start

        for point in trajectory["points"]:
            time = float(point["time_from_start"])
            if reverse:
                time = last_time - time

            frame = start + time * fps
            if frame > last_frame:
                last_frame = frame

            for value, name in zip(point["positions"], names):
                self.set_joint(name, value)
                self.add_keyframe(name, frame)

        return last_frame

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
            if geometry.filename and ".dae" in geometry.filename:
                mesh = rv.primitives.add_mesh({
                    "resource": geometry.filename
                })

                rv.utils.deselect_all()
                mesh.select_set(True)
                rv.utils.move_selected_to_collection(name)
                rv.utils.deselect_all()

                old = self.get_link(link_name)
                old.data = mesh.data

        rv.utils.remove_collection(name)



