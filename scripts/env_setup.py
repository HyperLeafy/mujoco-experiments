import os
import sys

def setup_headless():
    try:
        # Check that mujoco is loaded into the system before
        assert "mujoco" not in sys.modules, \
            "setup_headless must run before mujoco import"
        
        os.environ["MUJOCO_GL"] = "egl"      # or "offscreen"
        os.environ["PYOPENGL_PLATFORM"] = "egl"

        print("[env] setting to MUJOCO_GL =", os.environ.get("MUJOCO_GL"))
        print("[env] Headless/Graphics less simulation Pipleine setup")

        if os.environ.get("MUJOCO_GL") not in ("egl", "offscreen"):
            raise RuntimeError("Invalid MUJOCO_GL value")

    except Exception as e:
        print("[env] Headless setup failed")
        raise e   
