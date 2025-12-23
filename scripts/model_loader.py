import mujoco
import os
import pathlib

def load_model(xml_path : str)->mujoco.MjModel:
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    return model, data