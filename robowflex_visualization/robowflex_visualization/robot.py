'''Functions for loading and animating robots in Blender.
'''

import bpy
import os
import subprocess
from urdf_parser_py import urdf as URDF

import robowflex_visualization as rv


class Robot:
    def __init__(self, name, urdf):
        self.name = name
        self.collection = rv.utils.make_collection(name)

        self.load_urdf(urdf)
        self.prettify()

    def load_urdf(self, urdf):
        self.urdf = resolved = rv.utils.resolve_path(urdf)
        output = os.path.join("/tmp", os.path.basename(resolved) + ".dae")

        if not os.path.exists(output):
            subprocess.check_output(
                ["rosrun", "collada_urdf", "urdf_to_collada", resolved, output])

        bpy.ops.wm.collada_import(filepath = output)
        rv.utils.move_selected_to_collection(self.name)
        rv.utils.deselect_all()

        self.xml = open(self.urdf, 'r').read()
        self.robot = URDF.Robot.from_xml_string(self.xml)

    def prettify(self):
        for link_xml in self.robot.links:
            link = self.get_link(link_xml.name)
            # TODO: Make the Meshes look nice
            # rv.utils.apply_smooth_shade(link)
            # rv.utils.apply_edge_split(link)

    def get_link(self, link_name):
        return rv.utils.find_object_in_collection(self.name, link_name)

    def get_link_xml(self, link_name):
        for link in self.robot.links:
            if link.name == link_name:
                return link

        return None

    def get_joint_xml(self, joint_name):
        for joint in self.robot.joints:
            if joint.name == joint_name:
                return joint

        return None

    def set_joint(self, joint_name, value):
        joint_xml = self.get_joint_xml(joint_name)
        link_xml = self.get_link_xml(joint_xml.child)
        link = self.get_link(joint_xml.child)

        if (joint_xml.type == "prismatic"):
            link.location = [
                joint_xml.origin.xyz[i] + joint_xml.axis[i] * value
                for i in range(3)
            ]

        else:
            # Assuming Axis is one value...
            link.rotation_euler = [
                joint_xml.origin.rpy[i] + joint_xml.axis[i] * value
                for i in range(3)
            ]

    def add_keyframe(self, joint_name, frame):
        joint_xml = self.get_joint_xml(joint_name)
        link = self.get_link(joint_xml.child)

        if (joint_xml.type == "prismatic"):
            link.keyframe_insert(data_path = "location", frame = frame)
        else:
            link.keyframe_insert(data_path = "rotation_euler", frame = frame)

    def animate_path(self, path_file, fps = 60., start = 30):
        path = rv.utils.read_YAML_data(path_file)

        trajectory = path["joint_trajectory"]
        names = trajectory["joint_names"]

        for point in trajectory["points"]:
            time = float(point["time_from_start"])
            frame = start + time * fps

            for value, name in zip(point["positions"], names):
                self.set_joint(name, value)
                self.add_keyframe(name, frame)

def load(name, urdf):
    return Robot(name, urdf)
