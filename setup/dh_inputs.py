"""
Simplified DH table collector.
All joints are revolute.
Only 'theta' may be variable using '*' mark.

Returned structure for each joint:
{
    'theta': float | None,   # None => variable
    'alpha': float,
    'd': float,
    'a': float
}

Angle units:
    - All angles returned in radians.
Translation units:
    - Meters.
"""

import math

def _parse_theta(token, angle_mode):
    token = token.strip()
    if token == "" or token == "*":
        return None  # variable theta

    # numeric angle
    val = float(token)
    return val if angle_mode == "rad" else math.radians(val)


def _parse_const(token, angle_mode, is_angle=False):
    token = token.strip()
    if token == "":
        return 0.0

    val = float(token)
    if is_angle:
        return val if angle_mode == "rad" else math.radians(val)
    return val


def collect_dh_table():
    print("\n--- DH Table Collector (Simplified) ---")
    print("Input format per joint:  theta, alpha, d, a")
    print("Use '*' for variable theta. All translations are in METERS.\n")

    # angle mode
    angle_mode = ""
    while angle_mode not in ("deg", "rad"):
        angle_mode = input("Angle units? ('deg' default / 'rad'): ").strip().lower() or "deg"

    # number of joints
    try:
        n = int(input("Number of joints: ").strip())
    except:
        raise SystemExit("Invalid number of joints.")

    dh_rows = []

    for i in range(n):
        print(f"\n--- Joint {i} ---")
        line = input("Enter: theta, alpha, d, a  >> ").strip()

        if line == "":
            tokens = ["*", "0", "0", "0"]
        else:
            tokens = [t.strip() for t in line.split(",")]
            if len(tokens) != 4:
                raise SystemExit("Invalid format. Expected 4 comma-separated values.")

        t_tok, a_tok, d_tok, x_tok = tokens

        theta = _parse_theta(t_tok, angle_mode)
        alpha = _parse_const(a_tok, angle_mode, is_angle=True)
        d     = _parse_const(d_tok, angle_mode, is_angle=False)
        a     = _parse_const(x_tok, angle_mode, is_angle=False)

        row = {
            "theta": theta,   # None if variable
            "alpha": alpha,
            "d": d,
            "a": a
        }
        dh_rows.append(row)

    print("\nDH table collection complete.\n")
    return dh_rows


def get_dh_table():
    return collect_dh_table()


if __name__ == "__main__":
    dh = collect_dh_table()
    print("\nCollected DH rows:")
    import pprint
    pprint.pprint(dh)
