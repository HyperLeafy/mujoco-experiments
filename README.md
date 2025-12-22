# MuJoCo Bring-up (uv-managed)

## Scope
- Headless MuJoCo simulation
- URDF → MJCF conversion
- State inspection (`qpos`, `qvel`, `ctrl`)
- Model-level debugging and documentation

## Explicit Non-goals
- RL / training
- Controllers
- Rendering / visuals
- Performance optimization

## Tooling
- Python 3.10+
- MuJoCo (official bindings)
- Dependency management via **uv**

## Setup
```bash
uv sync


## To run mujaco viewer 
```bash
 uv run python -m mujoco.viewer --mjcf <path-to-mjmodel.xml>
