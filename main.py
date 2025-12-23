# Orchestration of Mujoco simulation
import numpy as np

from scripts.env_setup import setup_headless
setup_headless()

from scripts.model_loader import load_model
import scripts.stability_test  as tests


MODEL_PATH = "models/S1/urdf/humanoid_pkg.xml"

def main():
    # Load model
    model,data = load_model(MODEL_PATH)

    # Temp stand pos until actual is found 
    qpos_stand = data.qpos.copy()
    
    # Test data flow through the test pipe line
    tests.gravity_off(model=model)
    tests.standing(model=model, qpos_stand=qpos_stand)
    tests.passive_fall(model=model)

    print("ALL DATA FLOW TEST CASES PASSED")

    # Stability TEST (in progress)

if __name__ == "__main__":
    main()