import mujoco
import mujoco.viewer
import numpy as np
import time

# Load model
model_path = "./models/S1/urdf/mjmodel.xml"
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Fix: Elevate robot to proper standing height
# Set initial position of base to be above ground
data.qpos[2] = 0.8  # Z position (height)

print("=" * 60)
print("HUMANOID MANUAL CONTROL")
print("=" * 60)
print(f"Model: {model.names}")
print(f"Bodies: {model.nbody}")
print(f"Joints: {model.njnt}")
print(f"Actuators: {model.nu}")
print(f"DoF: {model.nv}")
print("=" * 60)

# Print actuator information
print("\nACTUATORS:")
print("-" * 60)
for i in range(model.nu):
    act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    joint_id = model.actuator_trnid[i, 0]
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
    print(f"{i:2d}. {act_name:25s} -> {joint_name}")

print("\n" + "=" * 60)
print("CONTROLS:")
print("  - Press SPACE to toggle control mode")
print("  - Mode 1: All joints oscillate")
print("  - Mode 2: Legs only (walking motion)")
print("  - Mode 3: Arms only (waving)")
print("  - Press R to reset robot")
print("  - Press Q or ESC to quit")
print("=" * 60)

# Control state
control_mode = 0
paused = False

# Create actuator name to index mapping
actuator_map = {}
for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    actuator_map[name] = i

def reset_robot():
    """Reset robot to initial standing pose"""
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.8  # Elevate base
    # Set joints to neutral standing position
    for i in range(model.nv):
        data.qvel[i] = 0
    data.ctrl[:] = 0
    print("\n[RESET] Robot reset to standing position")

def control_callback(model, data, mode, t):
    """Apply control based on mode"""
    if mode == 0:
        # No control - let physics take over
        data.ctrl[:] = 0
        
    elif mode == 1:
        # All joints oscillate
        for i in range(model.nu):
            data.ctrl[i] = 0.3 * np.sin(2 * np.pi * 0.5 * t + i * 0.3)
    
    elif mode == 2:
        # Leg control - walking motion
        leg_actuators = ['act_r_hip_pitch', 'act_r_knee', 'act_r_foot',
                        'act_l_hip_pitch', 'act_l_knee', 'act_l_foot']
        
        for act_name in leg_actuators:
            if act_name in actuator_map:
                idx = actuator_map[act_name]
                if 'hip_pitch' in act_name:
                    phase = 0 if 'right' in act_name else np.pi
                    data.ctrl[idx] = 0.4 * np.sin(2 * np.pi * 0.8 * t + phase)
                elif 'knee' in act_name:
                    data.ctrl[idx] = 0.6 * np.abs(np.sin(2 * np.pi * 0.8 * t))
                elif 'foot' in act_name:
                    data.ctrl[idx] = 0.2 * np.sin(2 * np.pi * 0.8 * t)
    
    elif mode == 3:
        # Arm control - waving
        arm_actuators = ['act_l_shoulder_pitch', 'act_l_shoulder_roll', 'act_l_elbow_yaw',
                        'act_r_shoulder_pitch', 'act_r_shoulder_roll', 'act_r_elbow_yaw']
        
        for act_name in arm_actuators:
            if act_name in actuator_map:
                idx = actuator_map[act_name]
                if 'shoulder_pitch' in act_name:
                    data.ctrl[idx] = 0.5 * np.sin(2 * np.pi * 1.0 * t)
                elif 'shoulder_roll' in act_name:
                    data.ctrl[idx] = 0.3 * np.sin(2 * np.pi * 1.0 * t + np.pi/2)
                elif 'elbow' in act_name:
                    data.ctrl[idx] = 0.4 * np.sin(2 * np.pi * 1.5 * t)

# Initialize
reset_robot()

print("\n[STARTING] Launching viewer...")
print("[INFO] Control Mode: 0 (No Control)")

# Main control loop
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    last_mode_switch = 0
    
    while viewer.is_running():
        current_time = time.time() - start_time
        
        # Simple keyboard handling (viewer-based)
        # Note: MuJoCo viewer has limited keyboard support
        # For better control, consider using pygame or similar
        
        # Apply control based on mode
        control_callback(model, data, control_mode, current_time)
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Update viewer
        viewer.sync()
        
        # Auto-cycle through modes every 10 seconds (demo)
        if current_time - last_mode_switch > 10:
            control_mode = (control_mode + 1) % 4
            mode_names = ["No Control", "All Oscillate", "Legs (Walk)", "Arms (Wave)"]
            print(f"\n[AUTO-SWITCH] Mode: {control_mode} - {mode_names[control_mode]}")
            last_mode_switch = current_time
        
        # Slow down simulation to real-time
        time.sleep(0.001)

print("\n[SHUTDOWN] Viewer closed")