import re
import sys

def clean_num(s):
    try:
        v = float(s)
        # round to 5dp first, then kill any -0
        v = round(v, 5)
        if v == 0.0:
            return "0"
        formatted = f"{v:.5f}".rstrip('0').rstrip('.')
        return formatted
    except ValueError:
        return s

def clean_pose(pose_str):
    return "  ".join(clean_num(p) for p in pose_str.strip().split())

def main(filepath):
    with open(filepath, "r") as f:
        content = f.read()

    results = []
    pos = 0
    while True:
        start = content.find('<link', pos)
        if start == -1:
            break

        # depth-aware scan to find matching </link>
        depth = 0
        i = start
        while i < len(content):
            o = content.find('<link',  i)
            c = content.find('</link>', i)
            if c == -1:
                break
            if o != -1 and o < c:
                depth += 1
                i = o + 5
            else:
                depth -= 1
                i = c + 7
                if depth == 0:
                    break

        block = content[start:i]
        pos = i

        name_m = re.match(r'<link\s+name="([^"]+)"', block)
        if not name_m:
            continue
        name = name_m.group(1)

        # strip sub-elements so only the link-level <pose> is visible
        stripped = re.sub(r'<(inertial|visual|collision)\b.*?</\1>', '', block, flags=re.DOTALL)
        pose_m = re.search(r'<pose(?:\s[^>]*)?>(.*?)</pose>', stripped, re.DOTALL)
        pose_str = clean_pose(pose_m.group(1)) if pose_m else "(no pose)"

        results.append((name, pose_str))

    if not results:
        print("No links found.")
        return

    max_name = max(len(r[0]) for r in results)
    print(f"\n{'LINK':<{max_name}}  POSE (x  y  z  roll  pitch  yaw)")
    print("-" * (max_name + 65))
    for name, pose in results:
        print(f"{name:<{max_name}}  {pose}")
    print(f"\n{len(results)} links found.")

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "Delta_robot.sdf"
    main(path)