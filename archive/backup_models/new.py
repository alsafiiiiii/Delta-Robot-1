#!/usr/bin/env python3
"""
copy_sdf_poses.py
Usage:
    python3 copy_sdf_poses.py <new_sdf> <original_sdf> [output_sdf]

Copies link poses from <new_sdf> into <original_sdf>, matching links by name
(with fuzzy stripping of common prefixes/suffixes).  Writes result to
[output_sdf] (default: original_sdf with _updated suffix).
"""

import re
import sys
from pathlib import Path


# ── helpers ──────────────────────────────────────────────────────────────────

def extract_poses(text: str) -> dict[str, str]:
    """Return {link_name: pose_string} for every <link> that has a <pose>."""
    poses = {}
    for m in re.finditer(
        r'<link\s+name="([^"]+)"[^>]*>(.*?)</link>', text, re.DOTALL
    ):
        name, body = m.group(1), m.group(2)
        pm = re.search(r'<pose[^>]*>(.*?)</pose>', body, re.DOTALL)
        if pm:
            poses[name] = pm.group(1).strip()
    return poses


def simplify(name: str) -> str:
    """
    Strip common CAD export noise so names can be matched across files.
    e.g. "servo_setup_v8_1_servo_mount2"  -> "servo_mount2"
         "pivot_arm_1_pivot_joint-v1-4"   -> "pivot_arm_1"
         "rod_cap_7_threaded_female-v1-11" -> "rod_cap_7"
         "delta2_differential_manipulator-v1_bevel_gear_ee" -> "bevel_gear_ee"
    """
    # drop long assembly prefixes like "servo_setup_v8_N_", "delta2_..._"
    name = re.sub(r'^servo_setup_v\d+_\d+_', '', name)
    name = re.sub(r'^delta\d+_differential_manipulator-v\d+_', '', name)
    # drop trailing joint/version suffixes like "_pivot_joint-v1-4", "-v1-11", "-v1"
    name = re.sub(r'_pivot_joint-v\d+-?\d*$', '', name)
    name = re.sub(r'_threaded_female-v\d+-?\d*$', '', name)
    name = re.sub(r'-v\d+-?\d*$', '', name)
    name = name.lower().strip('_')
    # normalise: letter immediately followed by trailing digits -> insert underscore
    # e.g. "servo_mount2" -> "servo_mount_2",  "rod_cap12" -> "rod_cap_12"
    name = re.sub(r'([a-z])(\d+)$', r'\1_\2', name)
    return name


def build_match_map(src_names: list[str], dst_names: list[str]) -> dict[str, str]:
    """
    Return {src_name: dst_name} for every src link that can be matched to a
    dst link.  Tries exact match first, then simplified-name match.
    """
    mapping = {}
    dst_set = set(dst_names)
    dst_simple = {simplify(n): n for n in dst_names}   # simple -> original

    # track which dst names are already claimed (for duplicate handling)
    claimed = set()

    for src in src_names:
        if src in dst_set and src not in claimed:
            mapping[src] = src
            claimed.add(src)
            continue
        s = simplify(src)
        if s in dst_simple and dst_simple[s] not in claimed:
            mapping[src] = dst_simple[s]
            claimed.add(dst_simple[s])

    return mapping


def replace_pose(text: str, link_name: str, new_pose: str) -> tuple[str, bool]:
    """Replace the first <pose> inside the named <link> block."""
    # find the link opening tag
    link_pat = re.compile(
        r'(<link\s+name="' + re.escape(link_name) + r'"[^>]*>)',
        re.DOTALL
    )
    m = link_pat.search(text)
    if not m:
        return text, False

    start = m.end()   # position right after <link name="...">

    # find the matching </link>
    depth = 1
    pos = start
    while pos < len(text) and depth > 0:
        open_m  = re.search(r'<link\b',  text[pos:])
        close_m = re.search(r'</link\s*>', text[pos:])
        if close_m and (not open_m or close_m.start() < open_m.start()):
            pos += close_m.end()
            depth -= 1
        elif open_m:
            pos += open_m.end()
            depth += 1
        else:
            break

    block = text[start:pos]

    # replace first <pose ...>...</pose> inside this block
    new_block, n = re.subn(
        r'<pose(\s[^>]*)?>.*?</pose>',
        lambda mo: f'<pose{mo.group(1) or ""}>{new_pose}</pose>',
        block, count=1, flags=re.DOTALL
    )
    if n == 0:
        return text, False

    return text[:start] + new_block + text[pos:], True


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    new_path  = Path(sys.argv[1])
    orig_path = Path(sys.argv[2])
    out_path  = Path(sys.argv[3]) if len(sys.argv) > 3 else \
                orig_path.with_stem(orig_path.stem + "_updated")

    new_text  = new_path.read_text(encoding="utf-8")
    orig_text = orig_path.read_text(encoding="utf-8")

    src_poses = extract_poses(new_text)   # name -> pose
    dst_poses = extract_poses(orig_text)  # name -> pose (just to get names)

    print(f"Source links with poses : {len(src_poses)}")
    print(f"Target links with poses : {len(dst_poses)}")

    match_map = build_match_map(list(src_poses), list(dst_poses))
    print(f"Matched pairs           : {len(match_map)}\n")

    updated = 0
    skipped = []
    result  = orig_text

    for src_name, dst_name in match_map.items():
        new_pose = src_poses[src_name]
        result, ok = replace_pose(result, dst_name, new_pose)
        if ok:
            updated += 1
            print(f"  ✓  {src_name!r}  ->  {dst_name!r}")
        else:
            skipped.append(dst_name)
            print(f"  ✗  could not replace pose for {dst_name!r}")

    if skipped:
        print(f"\nFailed to update: {skipped}")

    out_path.write_text(result, encoding="utf-8")
    print(f"\nUpdated {updated} poses.  Saved to: {out_path}")


if __name__ == "__main__":
    main()