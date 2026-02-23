#!/usr/bin/env python3
"""
Reorder model.sdf to match the old file structure
"""

import xml.etree.ElementTree as ET

# Parse the current model
tree = ET.parse('/home/rikisu/major_project_ws/src/delta_robot_description/models/model.sdf.backup')
root = tree.getroot()
model = root.find('.//model')

# Extract all links and joints
links = {link.get('name'): link for link in model.findall('link')}
joints = {joint.get('name'): joint for joint in model.findall('joint')}
plugins = list(model.findall('plugin'))

# Define the desired order based on old model
link_order = [
    'Frame',         # was base_link
    'base',          # was base_hex
    'forearm1',
    'forearm2',
    'forearm3',
    'arm1',
    'arm2',
   '3', 
    'arm4',
    'arm5',
    'arm6',
    'EEBase',
    'Bevel1',
    'Bevel2',
    'T',
    'EE'
]

joint_order = [
    'Frame_to_base',  # fixed joint Frame -> base
    'jbf1',
    'jbf2',
    'jbf3',
    'ball1',
    'ball2',
    'ball3',
    'ball4',
    'ball5',
    'ball6',
    'ball7',
    # ball8-12 commented out
    'ball9',
    'ball10',
    'ball11',
    # ball12 commented out
    'Bevelj1',
    'Bevelj2',
    'Tj1',
   'BeveljEE'
]

# Clear the model
for element in list(model):
    model.remove(element)

# Add links in order
for link_name in link_order:
    if link_name in links:
        model.append(links[link_name])

# Add joints in order
for joint_name in joint_order:
    if joint_name in joints:
        model.append(joints[joint_name])

# Add plugins at the end
for plugin in plugins:
    model.append(plugin)

# Write the reordered file
tree.write('/home/rikisu/major_project_ws/src/delta_robot_description/models/model.sdf',
           xml_declaration=True, encoding='utf-8', method='xml')

print("Model reordered successfully!")
