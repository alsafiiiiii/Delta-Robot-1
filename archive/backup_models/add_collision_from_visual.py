import sys
import re
from pathlib import Path

def duplicate_visual_as_collision(file_path):
    with open(file_path, 'r') as f:
        content = f.read()


    # Find all <visual ...>...</visual> blocks (non-greedy)
    visual_blocks = re.findall(r'(<visual[\s\S]*?</visual>)', content, re.MULTILINE)
    collision_blocks = []
    for block in visual_blocks:
        # Remove <material>...</material> and <xacro:material_.../> tags
        # Remove <material>...</material>
        block_wo_material = re.sub(r'<material[\s\S]*?</material>', '', block, flags=re.MULTILINE)
        # Remove <xacro:material_.../>
        block_wo_material = re.sub(r'<xacro:material_[^>]*/>', '', block_wo_material, flags=re.MULTILINE)
        # Change <visual ...> to <collision ...> and </visual> to </collision>
        collision_block = re.sub(r'<visual', '<collision', block_wo_material, count=1)
        collision_block = re.sub(r'</visual>', '</collision>', collision_block, count=1)
        collision_blocks.append(collision_block)

    # Insert collision blocks after each visual block
    new_content = content
    for v, c in zip(visual_blocks, collision_blocks):
        new_content = new_content.replace(v, v + '\n' + c)

    # Write to a new file
    out_path = Path(file_path).with_suffix('.with_collision' + Path(file_path).suffix)
    with open(out_path, 'w') as f:
        f.write(new_content)
    print(f"Collision tags added. Output: {out_path}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python add_collision_from_visual.py <path_to_sdf_or_xacro>")
        sys.exit(1)
    duplicate_visual_as_collision(sys.argv[1])
