import xml.etree.ElementTree as ET
from pathlib import Path

# -------- CONFIG --------
INPUT_XML  = "/home/blank/Projects/Internship/sentienc/Robotic-Simulation/mujoco-experiments/models/S1/urdf/mjmodel.xml"
OUTPUT_XML = "humanoid_pkg_train_inertia_fixed.xml"

# default inertial values (stable for humanoids)
DEFAULTS = {
    "torso":  dict(mass=20.0, inertia=(1.0, 1.0, 1.0)),
    "pelvis": dict(mass=10.0, inertia=(0.5, 0.5, 0.5)),
    "thigh":  dict(mass=6.0,  inertia=(0.2, 0.2, 0.05)),
    "shin":   dict(mass=4.0,  inertia=(0.15, 0.15, 0.04)),
    "foot":   dict(mass=1.0,  inertia=(0.02, 0.02, 0.01)),
}

def classify_link(name: str):
    lname = name.lower()
    if "torso" in lname or "chest" in lname:
        return "torso"
    if "pelvis" in lname or "waist" in lname:
        return "pelvis"
    if "thigh" in lname or "hip" in lname:
        return "thigh"
    if "shin" in lname or "knee" in lname or "calf" in lname:
        return "shin"
    if "foot" in lname or "ankle" in lname:
        return "foot"
    return None

# -------- SCRIPT --------
tree = ET.parse(INPUT_XML)
root = tree.getroot()

count = 0

for body in root.iter("body"):
    name = body.get("name", "")
    cls = classify_link(name)
    if cls is None:
        continue

    # remove old inertial if present
    for inertial in body.findall("inertial"):
        body.remove(inertial)

    cfg = DEFAULTS[cls]

    inertial = ET.SubElement(body, "inertial")
    inertial.set("pos", "0 0 0")
    inertial.set("mass", str(cfg["mass"]))
    inertial.set(
        "diaginertia",
        f"{cfg['inertia'][0]} {cfg['inertia'][1]} {cfg['inertia'][2]}"
    )

    count += 1

tree.write(OUTPUT_XML)
print(f"✔ Overrode inertials for {count} links")
print(f"✔ Saved to {OUTPUT_XML}")
