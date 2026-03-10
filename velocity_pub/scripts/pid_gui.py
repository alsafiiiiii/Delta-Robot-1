import re

def parse_val(s):
    # Snap scientific notation or very small numbers to 0.0
    val = float(s)
    return 0.0 if abs(val) < 1e-9 else val

def get_precision(s):
    # Finds the max decimal places used in the input string
    matches = re.findall(r'\.(\d+)', s)
    return len(max(matches, key=len)) if matches else 2

def process():
    l_raw = input("Link Pose: ").strip()
    j_raw = input("Joint Pose: ").strip()

    # Determine output precision based on input
    prec = max(get_precision(l_raw), get_precision(j_raw))

    L = [parse_val(x) for x in re.findall(r'-?\d+\.?\d*(?:[eE][-+]?\d+)?', l_raw)]
    J = [parse_val(x) for x in re.findall(r'-?\d+\.?\d*(?:[eE][-+]?\d+)?', j_raw)]

    # Pad to 6 elements
    L, J = [(v + [0.0]*6)[:6] for v in (L, J)]

    # Calculations
    inv_J = [-x for x in J]
    L_new = [val + j for val, j in zip(L, J)] # L - (-J)

    def fmt(nums): 
        return " ".join(f"{x:.{prec}f}".rstrip('0').rstrip('.') or "0" for x in nums)

    print(f"\n<pose>{fmt(L_new)}</pose>")
    print(f"<visual name='visual'>\n  <pose>{fmt(inv_J)}</pose>\n</visual>")

if __name__ == "__main__":
    process()