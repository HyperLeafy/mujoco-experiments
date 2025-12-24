import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1]))


from utility_tools.actuator_sign_test import test_actuator_sign
from dm_control import mujoco

MODEL_PATH = "./models/S1/urdf/mjmodel.xml"

physics = mujoco.Physics.from_xml_path(MODEL_PATH)

# Joints to test
JOINT_ACTUATOR_PAIRS = [
    # ("left_hip_pitch_joint", "act_l_hip_pitch"),
    # ("right_hip_pitch_joint", "act_r_hip_pitch"),
    ("left_knee_joint", "act_l_knee"),
    ("right_knee_joint", "act_r_knee"),
]

for joint, actuator in JOINT_ACTUATOR_PAIRS:
    result = test_actuator_sign(physics, joint, actuator)

    status = "PASS" if result["pass"] else "FAIL"
    print(
        f"{status:4} | {joint:30} | "
        f"+qvel={result['qvel_pos']:+.3f} "
        f"-qvel={result['qvel_neg']:+.3f}"
    )