# 🐜 Hexapod Simulation (Rectangular Body) — Python Only

A Python-based simulation framework for modeling, planning, and controlling a six-legged hexapod robot with a rectangular chassis. Designed for robotics research, terrain-aware locomotion, and gait experimentation — no MATLAB, no MapleSim, just Python.

## 📦 Repository Structure

```bash
├── hexapod/              # Core simulation modules
│   ├── kinematics.py     # Inverse kinematics for rectangular body
│   ├── dynamics.py       # Basic physics and torque modeling
│   ├── gait.py           # Tripod, ripple, wave gait generators
│   ├── planner.py        # A* and PRM path planning
│   ├── terrain.py        # Terrain generation and analysis
│   └── controller.py     # Hierarchical control logic
├── gui/                  # GUI interface and input handling
├── examples/             # Sample scripts and test cases
├── assets/               # Diagrams, terrain maps, and visuals
├── tests/                # Unit tests for each module
├── README.md             # This file
