# MuJoCo Bring-up (uv-managed)

Minimal, **engineering-focused** MuJoCo bring-up for humanoid models.  
This repo documents the **messy but real** process of getting a URDF-origin model into a **numerically stable MuJoCo MJCF**.

---

## Scope

This repository focuses on **model bring-up and physics sanity**, not downstream tasks.

- Headless MuJoCo simulation
- URDF → MJCF loading and conversion paths
- Free-joint (floating base) handling
- State inspection and debugging:
  - `qpos`, `qvel`, `ctrl`
  - actuator ↔ joint mapping
- Contact, inertia, and stability debugging
- Scripted model re-authoring (e.g. inertial overrides)

---

## Explicit Non-Goals

These are **intentionally out of scope**:

- Reinforcement learning / training
- High-level controllers (walking, balance, MPC, etc.)
- Polished visualization or rendering pipelines
- Performance optimization or real-time guarantees

This repo exists **before** those layers.

---

## Key Learnings Captured

- MuJoCo’s URDF support is partial and requires adaptation
- Converted inertias from CAD/URDF are often **numerically unstable**
- Stable simulation requires:
  - sane free-joint initialization
  - controlled contact setup
  - re-authored inertial parameters
- Actuator sliders do **not** update unless `mj_forward()` / `mj_step()` is called
- Axis mirroring errors show up as asymmetric limb behavior

---

## Repository Structure
`
.
├── scripts/ # headless tests, XML transforms, inertia overrides
├── notebooks/ # interactive debugging & visualization
├── models/ # URDF / MJCF variants
├── README.md`


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


This creates an isolated environment with MuJoCo and required tooling.
Running Headless Scripts

uv run python scripts/<script_name>.py

Used for:

    model loading

    state inspection

    numerical stability checks

Launching MuJoCo Viewer

For interactive inspection and actuator debugging:

uv run python -m mujoco.viewer --mjcf <path-to-mjmodel.xml>

Note:
When paused, the viewer does not update kinematics unless mj_forward() is called from code.
Current Status

    Model loads successfully in MuJoCo

    Headless stepping verified

    Actuator mapping validated

    Numerical instability traced to inertial/contact issues

    Inertial re-authoring in progress to achieve stable standing

