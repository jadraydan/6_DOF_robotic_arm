"""
Offset collector (pure data, no matrix math).

Usage:
    python -m setup.offset_inputs

Provides:
    collect_offsets(n) -> list[dict]
    get_offsets(n)     -> wrapper

Each entry is a dict of raw values:
{
    'tx': float,
    'ty': float,
    'tz': float,
    'rx': float,
    'ry': float,
    'rz': float,
    'is_identity': bool
}

Units:
- translations in METERS
- angles in DEGREES or RADIANS (format chosen at input)
"""
import math

def collect_offsets(n=None):
    if n is None:
        try:
            n = int(input("Number of links: ").strip())
        except:
            raise SystemExit("Invalid number.")

    # angle mode
    angle_mode = ""
    while angle_mode not in ("deg", "rad"):
        angle_mode = input("Angle units? ('deg' default / 'rad'): ").strip().lower() or "deg"

    offsets = []

    print("\n--- Offset Collection ---")
    print("Translations MUST be in METERS.")
    print("If a link has no physical offset, choose 'no' and identity will be stored.\n")

    for i in range(n):
        print(f"\n=== Link {i} ===")
        has = input("Does this link have a physical offset? (yes/no): ").strip().lower()

        if has in ("no", "n", ""):
            offsets.append({
                "tx": 0.0, "ty": 0.0, "tz": 0.0,
                "rx": 0.0, "ry": 0.0, "rz": 0.0,
                "is_identity": True
            })
            print(" → Saved identity offset.")
            continue

        print("Enter translation in METERS and RPY rotation:")
        print("Format: tx, ty, tz, rx, ry, rz")
        raw = input(" >> ").strip()
        toks = [t.strip() for t in raw.split(",")]

        if len(toks) != 6:
            raise SystemExit("Expected 6 comma-separated values.")

        tx, ty, tz = float(toks[0]), float(toks[1]), float(toks[2])
        rx, ry, rz = float(toks[3]), float(toks[4]), float(toks[5])

        # store raw, matrix construction happens in transforms.py
        if angle_mode == "deg":
            rx, ry, rz = math.radians(rx), math.radians(ry), math.radians(rz)

        offsets.append({
            "tx": tx,
            "ty": ty,
            "tz": tz,
            "rx": rx,
            "ry": ry,
            "rz": rz,
            "is_identity": False
        })

    print("\nOffset data collection complete.")
    return offsets

def get_offsets(n=None):
    return collect_offsets(n)

if __name__ == "__main__":
    data = collect_offsets()
    print("\nCollected offsets:\n", data)
