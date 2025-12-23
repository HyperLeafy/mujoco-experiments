import mujoco
import numpy as np

DT = 0.002
STEPS_5S = int(5.00/DT)
MAX_QACC = 1e5
ESP_VEL = 1e-6

def _finite(x, name):
    if x.size == 0:
        return 
    if not np.isfinite(x).all():
        raise RuntimeError(f"[{name}] has NaN/Inf")

# To check if the simluated valued are valid or not
def _step(model, data, steps, tag):
    for i in range(steps):
        mujoco.mj_step(model, data)
        _finite(data.qpos, "QPOS")
        _finite(data.qvel, "QVEL")
        _finite(data.qacc, "QACC")
        if data.qacc.size > 0:
            if np.max(np.abs(data.qacc)) > MAX_QACC:
                raise RuntimeError(f"[{tag}] QACC Spike")

def gravity_off(model):
    data = mujoco.MjData(model)
    model.opt.gravity[:] = 0
    mujoco.mj_forward(model, data)
    # Checking parater values are valid or not after toggeling gravity
    _step(model, data, STEPS_5S, "gravity_off")
    if data.qacc.size > 0:
        if np.max(np.abs(data.qvel)) > ESP_VEL:
            raise RuntimeError("Gracity OFF drift")

def passive_fall(model):
    data = mujoco.MjData(model)
    model.opt.gravity[:] = [0, 0, -9.81]
    mujoco.mj_forward(model, data)
    _step(model, data, STEPS_5S, "passive_fall")

def standing(model, qpos_stand):
    data = mujoco.MjData(model)
    model.opt.gravity[:] = [0,0,-9.81]
    data.qpos[:] = qpos_stand
    data.qvel[:] = 0
    data.ctrl[:] = qpos_stand[model.nq - model.nu :]
    mujoco.mj_forward(model, data)
    _step(model, data, STEPS_5S, "standing")

