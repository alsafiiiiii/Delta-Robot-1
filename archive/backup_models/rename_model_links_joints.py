import xml.etree.ElementTree as ET
import re
import shutil

# File paths
SRC = "model.sdf.xacro"
DST = "model_renamed.sdf.xacro"

# Mapping for links and joints (partial, extend as needed)
link_map = {
    "world_lnk_1": "world_link",
    "Frame": "frame",
    "base_link": "base_link",
    # Leg 1
    "servo1_mount": "servo_mount_1",
    "servo1": "servo_1",
    "servo1_horn1": "servo_horn_upper_1",
    "servo1_horn2": "servo_horn_lower_1",
    "bicep1": "bicep_1",
    # Leg 2
    "servo2_mount": "servo_mount_2",
    "servo2": "servo_2",
    "servo2_horn1": "servo_horn_upper_2",
    "servo2_horn2": "servo_horn_lower_2",
    "bicep2": "bicep_2",
    # Leg 3
    "servo3_mount": "servo_mount_3",
    "servo3": "servo_3",
    "servo3_horn1": "servo_horn_upper_3",
    "servo3_horn2": "servo_horn_lower_3",
    "bicep3": "bicep_3",
    # Add rod caps, pivots, arms, virtual pivots, end effector, etc.
    "end_effctr1-_1": "end_effector",
    # ...
}
joint_map = {
    "virtual_joint": "joint_world_to_frame",
    "base_frame": "joint_frame_to_base",
    "base_to_servo_mount1": "joint_base_to_servo_mount_2",
    "base_to_servo_mount2": "joint_base_to_servo_mount_1",
    "base_to_servo_mount3": "joint_base_to_servo_mount_3",
    "servo_mount1_to_servo2": "joint_servo_mount_2_to_servo_2",
    "servo_mount2_to_servo1": "joint_servo_mount_1_to_servo_1",
    "servo_mount3_to_servo3": "joint_servo_mount_3_to_servo_3",
    "servo2_to_horn1": "motor_joint_2",
    "servo1_to_horn1": "motor_joint_1",
    "servo3_to_horn1": "motor_joint_3",
    "servo2_to_horn2": "joint_servo_2_to_horn_lower_2",
    "servo1_to_horn2": "joint_servo_1_to_horn_lower_1",
    "servo3_to_horn2": "joint_servo_3_to_horn_lower_3",
    "horn1_to_bicep2": "joint_horn_upper_2_to_bicep_2",
    "horn2_to_bicep1": "joint_horn_upper_1_to_bicep_1",
    "horn3_to_bicep3": "joint_horn_upper_3_to_bicep_3",
    # ...
}

# Helper to replace names in text
pattern = re.compile(r'("|>)([\w\-]+)("|<)')
def replace_name(text, mapping):
    for old, new in mapping.items():
        text = re.sub(rf'(?<=\W){re.escape(old)}(?=\W)', new, text)
    return text

def main():
    # Copy original to new file for backup
    shutil.copyfile(SRC, DST)
    with open(DST, 'r', encoding='utf-8') as f:
        xml = f.read()
    # Replace link and joint names everywhere
    xml = replace_name(xml, link_map)
    xml = replace_name(xml, joint_map)
    with open(DST, 'w', encoding='utf-8') as f:
        f.write(xml)
    print(f"Renamed file written to {DST}")

if __name__ == "__main__":
    main()
