import numpy as np

def test_actuator_sign(physics, joint_name:str, actuator_name:str, ctrl: float=0.3, steps:int = 50,):
    
    joint_id = physics.model.name2id(joint_name, "joint")
    actuator_id = physics.model.name2id(actuator_name, "actuator")

    def measure(ctrl_value):
        physics.reset()
        physics.data.ctrl[:] = 0.0
        for _ in range(steps):
            physics.data.ctrl[actuator_id] = ctrl_value
            physics.step()

        return physics.data.qvel[joint_id]
    
    qvel_pos = measure(+ctrl)
    qvel_neg = measure(-ctrl)
    passed = np.sign(qvel_pos) != np.sign(qvel_neg)

    return {
        "joint": joint_name,
        "actuator": actuator_name,
        "qvel_pos": qvel_pos,
        "qvel_neg": qvel_neg,
        "pass": passed,
    }