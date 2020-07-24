"""Functions for loading and animating robots in Blender.
"""

import bpy
import os
import subprocess
from urdf_parser_py import urdf as URDF

import robowflex_visualization as rv


class Robot:
    """\brief Controllable URDF described robot.
    This class loads a URDF from the package resource URI \a urdf under the Blender collection \a name.
    """

    def __init__(self, name, urdf):
        """\brief Constructor. Loads robot and creates a collection.

        \param name Name of Blender collection to put Robot geometry in.
        \param urdf URDF package resource URI to load.
        """
        self.name = name
        self.collection = rv.utils.make_collection(name)

        self.load_urdf(urdf)
        self.prettify()

    def load_urdf(self, urdf):
        """\brief Loads the URDF as a COLLADA mesh, as well as loading the XML.

        \param urdf URDF package resource URI to load.
        """
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

    def prettify(self):
        """\brief Makes the Robot's meshes look nice.
        TODO: Make the Meshes look nice.
        """
        pass

    def get_link(self, link_name):
        """\brief Gets a Blender object corresponding to a link on the robot.

        \param link_name Name of the link to find in the robot.
        """
        return rv.utils.find_object_in_collection(self.name, link_name)

    def get_link_xml(self, link_name):
        """\brief Get the XML description in the URDF of a link.

        \param link_name Name of the link to find in the robot.
        """
        for link in self.robot.links:
            if link.name == link_name:
                return link

        return None

    def get_joint_xml(self, joint_name):
        """\brief Get the XML description in the URDF of a joint.

        \param joint_name Name of the joint to find in the robot.
        """
        for joint in self.robot.joints:
            if joint.name == joint_name:
                return joint

        return None

    def set_joint(self, joint_name, value):
        """\brief Set the value of a joint in the URDF.

        Currently limited to 1-DoF joints (e.g., prismatic, continuous, or
        revolute). Works by finding the child link of the joint and setting its
        relative transformation in Blender according to the axis of movement.

        TODO: Handle multi-dof joints

        \param joint_name Name of the joint to set in the robot.
        \param value Value of joint to set.

        """
        joint_xml = self.get_joint_xml(joint_name)
        link_xml = self.get_link_xml(joint_xml.child)
        link = self.get_link(joint_xml.child)

        #Assuming Axis is one value...
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

    def add_keyframe(self, joint_name, frame):
        """\brief Adds a keyframe to a joint in the URDF at a frame in the animation timeline.

        Adds keyframe actually to child link (same as set_joint()). Uses
        current transform of the link. Only keyframes important data path for
        the joint (e.g., rotation for rotation joints).

        \param joint_name Joint to add keyframe to.
        \param frame Frame to add keyframe at.

        """
        joint_xml = self.get_joint_xml(joint_name)
        link = self.get_link(joint_xml.child)

        if (joint_xml.type == "prismatic"):
            link.keyframe_insert(data_path = "location", frame = frame)
        else:
            link.keyframe_insert(data_path = "rotation_euler", frame = frame)

    def animate_path(self, path_file, fps = 60., start = 30):
        """\brief Adds keyframes to animate a moveit_msgs::RobotTrajectoryMsg.

        \param path_file YAML file for moveit_msgs::RobotTrajectoryMsg.
        \param fps Frames-per-second to animate the path at.
        \param start Frame to begin the animation at.
        """
        path = rv.utils.read_YAML_data(path_file)

        trajectory = path["joint_trajectory"]
        names = trajectory["joint_names"]

        for point in trajectory["points"]:
            time = float(point["time_from_start"])
            frame = start + time * fps

            for value, name in zip(point["positions"], names):
                self.set_joint(name, value)
                self.add_keyframe(name, frame)

    def attach_object(self, link_name, item):
        """\brief Attaches an object to a link of the robot.

        Actually sets the parent of the item to the link.

        \param link_name Name of link to attach item to.
        \param item Blender object to attach to link.
        """
        rv.utils.parent_object(self.get_link(link_name), item)


def load(name, urdf):
    """\brief Loads a robot.

    \param name Name of robot to use. Creates scene that contains all links.
    \param urdf URDF URI to load.
    """
    return Robot(name, urdf)
