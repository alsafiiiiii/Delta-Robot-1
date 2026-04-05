
import xml.etree.ElementTree as ET
import sys
import os
import re

INPUT_FILE = "model.sdf"
OUTPUT_FILE = "model_renamed.sdf"

def extract_prefix(name):
    # Find v1_, v2_, ... or v1-, v2-, ... or similar at start
    m = re.match(r"(.*?[vV]\d+[_-])", name)
    if m:
        return m.group(1), name[len(m.group(1)):]
    # Otherwise, use up to first dash or underscore as prefix
    m = re.match(r"([^_-]+[_-])", name)
    if m:
        return m.group(1), name[len(m.group(1)):]
    return "", name

def shorten_name(rest, maxlen=8):
    # Remove vowels except first char, cut to maxlen
    if not rest:
        return ""
    s = rest[0] + re.sub(r"[aeiouAEIOU]", "", rest[1:])
    return s[:maxlen]
def fix_topology_errors(root):
    """
    Patches CAD exporter errors:
    1. Connects the virtual_joint to the actual base link.
    2. Reverses the inverted servo mount joints to connect the tree.
    """
    # Fix 1: Attach world to delta_base-v1 instead of non-existent 'base'
    for joint in root.iter('joint'):
        if joint.get('name') == 'virtual_joint':
            child_tag = joint.find('child')
            if child_tag is not None and child_tag.text.strip() == 'base':
                child_tag.text = 'delta_base-v1'
                print("Fixed: Connected world_link -> delta_base-v1")

    # Fix 2: Explicit map of the broken motor joints to their correct Parent -> Child relationships
    # We make the Mount the parent, and the heavy Motor Body (ComponentXXX) the child.
    motor_corrections = {
        "servo_setup_v8_1_ST3215-HS-v5_Rigid-1": {
            "parent": "servo_setup_v8_1_servo_mount-v2",
            "child": "servo_setup_v8_1_ST3215-HS-v5_Component137"
        },
        "servo_setup_v8_2_ST3215-HS-v5-2_Rigid-1": {
            "parent": "servo_setup_v8_2_servo_mount-v2-2",
            "child": "servo_setup_v8_2_ST3215-HS-v5-2_Component135"
        },
        "servo_setup_v8_3_ST3215-HS-v5-1_Rigid-1": {
            "parent": "servo_setup_v8_3_servo_mount-v2-1",
            "child": "servo_setup_v8_3_ST3215-HS-v5-1_Component136"
        }
    }

    fixed_count = 0
    for joint in root.iter('joint'):
        j_name = joint.get('name')
        if j_name in motor_corrections:
            parent_tag = joint.find('parent')
            child_tag = joint.find('child')
            
            if parent_tag is not None and child_tag is not None:
                parent_tag.text = motor_corrections[j_name]["parent"]
                child_tag.text = motor_corrections[j_name]["child"]
                fixed_count += 1
                
    print(f"Fixed: Reconnected {fixed_count} motor roots to the base.")
    
def main():
    if not os.path.exists(INPUT_FILE):
        print(f"{INPUT_FILE} not found in current directory. Please run from the model directory.")
        sys.exit(1)

    tree = ET.parse(INPUT_FILE)
    root = tree.getroot()
    fix_topology_errors(root)
    link_map = {}
    visual_map = {}
    link_counts = {}
    visual_counts = {}

    # Rename <link> and <visual> names
    for link in root.iter('link'):
        old_link_name = link.get('name')
        prefix, rest = extract_prefix(old_link_name)
        short_rest = shorten_name(rest)
        # Ensure uniqueness
        key = prefix + short_rest
        link_counts.setdefault(key, 1)
        new_link_name = f"{key}_{link_counts[key]}"
        link_counts[key] += 1
        link_map[old_link_name] = new_link_name
        link.set('name', new_link_name)
        # Rename visuals inside this link
        for visual in link.findall('visual'):
            old_visual_name = visual.get('name')
            vprefix, vrest = extract_prefix(old_visual_name)
            vshort_rest = shorten_name(vrest)
            vkey = vprefix + vshort_rest
            visual_counts.setdefault(vkey, 1)
            new_visual_name = f"{vkey}_{visual_counts[vkey]}"
            visual_counts[vkey] += 1
            visual_map[old_visual_name] = new_visual_name
            visual.set('name', new_visual_name)

    # Update all references to links and visuals
    for elem in root.iter():
        for attr in elem.attrib:
            if elem.attrib[attr] in link_map:
                elem.attrib[attr] = link_map[elem.attrib[attr]]
            if elem.attrib[attr] in visual_map:
                elem.attrib[attr] = visual_map[elem.attrib[attr]]
        # Some references may be in text
        if elem.text and elem.text.strip() in link_map:
            elem.text = link_map[elem.text.strip()]
        if elem.text and elem.text.strip() in visual_map:
            elem.text = visual_map[elem.text.strip()]

    tree.write(OUTPUT_FILE, encoding='utf-8', xml_declaration=True)
    print(f"Renamed SDF written to {OUTPUT_FILE}")


def visualize_sdf_tree(sdf_file):
    import xml.etree.ElementTree as ET
    from collections import defaultdict, deque

    tree = ET.parse(sdf_file)
    root = tree.getroot()

    # Find all links and joints
    links = set()
    joints = []
    for link in root.iter('link'):
        links.add(link.get('name'))
    for joint in root.iter('joint'):
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is not None and child is not None:
            joints.append((parent.text.strip(), child.text.strip(), joint.get('name')))

    # Build parent->children map
    tree_map = defaultdict(list)
    child_set = set()
    for parent, child, joint_name in joints:
        tree_map[parent].append((child, joint_name))
        child_set.add(child)

    # Find root(s): links that are never a child
    roots = [l for l in links if l not in child_set]
    if not roots:
        print("No root link found. The tree may be cyclic or incomplete.")
        return

    # BFS to print tree
    def print_tree(node, prefix="", visited=None):
        if visited is None:
            visited = set()
        visited.add(node)
        children = tree_map.get(node, [])
        for i, (child, joint_name) in enumerate(children):
            connector = "├─" if i < len(children) - 1 else "└─"
            print(f"{prefix}{connector} {node} --[{joint_name}]→ {child}")
            if child not in visited:
                print_tree(child, prefix + ("│  " if i < len(children) - 1 else "   "), visited)

    print("\nSDF Joint Parent→Child Tree:")
    for root_link in roots:
        print(f"{root_link}")
        print_tree(root_link)

if __name__ == "__main__":
    main()
    # Visualize the renamed SDF tree
    visualize_sdf_tree(OUTPUT_FILE)
