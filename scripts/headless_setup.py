import mujoco

# Load URDF directly - MuJoCo handles the conversion internally
model = mujoco.MjModel.from_xml_path("/home/blank/Projects/Internship/sentienc/Robotic-Simulation/mujoco-experiments/models/S1/urdf/humanoid_pkg.xml")
print("✅ Model loaded from URDF!")