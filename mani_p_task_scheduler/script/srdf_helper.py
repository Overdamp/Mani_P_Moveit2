#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import os
from ament_index_python.packages import get_package_share_directory

class SRDFHelper:
    def __init__(self, package_name="mani_p_moveit_config3", srdf_file="config/Manipulator_station_urdf_2.srdf"):
        self.package_name = package_name
        self.srdf_file = srdf_file
        self.group_states = {} # Dictionary to store parsed states: {'group_name': {'state_name': {'joint_name': value}}}
        self.load_srdf()

    def load_srdf(self):
        try:
            # Find the package directory
            pkg_path = get_package_share_directory(self.package_name)
            full_path = os.path.join(pkg_path, self.srdf_file)
            
            if not os.path.exists(full_path):
                # Fallback: Try to find in source directory if running from workspace without install
                # This is a bit hacky but useful for development
                # Assuming standard workspace structure: src/package_name/...
                # But safer to rely on installed path. If not found, just print warning.
                print(f"⚠️ SRDF file not found at: {full_path}")
                return

            tree = ET.parse(full_path)
            root = tree.getroot()

            # Parse <group_state> tags
            for group_state in root.findall('group_state'):
                group_name = group_state.get('group')
                state_name = group_state.get('name')
                
                if group_name not in self.group_states:
                    self.group_states[group_name] = {}
                
                joints = {}
                for joint in group_state.findall('joint'):
                    name = joint.get('name')
                    value = float(joint.get('value'))
                    joints[name] = value
                
                self.group_states[group_name][state_name] = joints
                
            print(f"✅ Loaded SRDF Poses: {self.get_available_poses()}")

        except Exception as e:
            print(f"❌ Error loading SRDF: {e}")

    def get_pose(self, group_name, state_name):
        """ Returns a dictionary of joint values for the specified group and state name """
        if group_name in self.group_states and state_name in self.group_states[group_name]:
            return self.group_states[group_name][state_name]
        return None

    def get_available_poses(self):
        """ Returns a list of available poses "group:state" """
        poses = []
        for group, states in self.group_states.items():
            for state in states:
                poses.append(f"{group}:{state}")
        return poses
