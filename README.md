# MuJoCo Bring-up (uv-managed)

Minimal, **engineering-focused** MuJoCo bring-up for humanoid models.  
This repository documents the **real bring-up process** of taking a URDF-derived humanoid into a **numerically stable MuJoCo simulation**.

---

## Scope

This repo focuses strictly on **model bring-up and physics sanity**, not downstream control or learning.

Included:
- Headless MuJoCo simulation (no viewer / no GL dependency)
- URDF → MJCF loading and conversion paths
- Floating-base (free joint) handling
- State inspection and debugging:
  - `qpos`, `qvel`, `qacc`, `ctrl`
- Actuator ↔ joint mapping validation
- Contact, inertia, and stability debugging
- Scripted re-authoring (e.g. inertial overrides)

---

## Explicit Non-Goals

Out of scope by design:
- Reinforcement learning / training
- High-level controllers (walking, balance, MPC)
- Performance optimization or real-time guarantees
- Polished rendering pipelines

This repository exists **before** those layers.

---

## Key Learnings Captured

- MuJoCo URDF support is partial and requires adaptation
- CAD / URDF inertias are often **numerically unstable**
- Stable simulation requires:
  - sane free-joint initialization
  - simplified, well-placed contact geometry
  - re-authored inertial parameters
- Actuator inputs do not propagate without `mj_forward()` / `mj_step()`
- Axis mirroring errors manifest as asymmetric limb behavior
- Stability must be validated **before** any control or learning work

---

## Repository Structure
```
.
├── scripts/ # headless tests, env setup, stability checks
├── notebooks/ # interactive debugging & inspection
├── models/ # URDF / MJCF variants
├── README.md
```

---

## Tooling

- Python 3.10+
- MuJoCo (official Python bindings)
- Dependency management via **uv**
- No ROS dependency

---

## Setup

```bash
uv sync
```
### Run headless scripts:

```uv run python main.py```

### Launch MuJoCo viewer (for manual inspection only):

```uv run python -m mujoco.viewer --mjcf <path-to-model.xml>```
