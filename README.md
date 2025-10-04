# 🐜 Hexapod Simulation (Rectangular Body) — Python Only

A Python-based framework for modeling, planning, and controlling a six-legged hexapod robot with a rectangular chassis. Built for robotics research, terrain-aware locomotion, and gait exploration.

---

## 📦 Repository Structure

```bash
├── hexapod/              # Core simulation modules
│   ├── kinematics.py     # Inverse kinematics for rectangular body
│   ├── dynamics.py       # Physics and torque modeling
│   ├── gait.py           # Gait generators (tripod, ripple, wave)
│   ├── planner.py        # Path planning (A*, PRM)
│   ├── terrain.py        # Terrain generation/analysis
│   ├── controller.py     # Control logic
│   ├── gui.py            # GUI interface and visualization
│   └── input_handler.py  # Input handling for controls
├── README.md             # Project documentation
```

---

## 🤖 Robot Overview

**Rectangular Body Layout:**

Six legs mounted in three pairs along long edges:
- Left: l1 (front), l2 (middle), l3 (rear)
- Right: r1 (front), r2 (middle), r3 (rear)

```
      front
 ┌─────────────────┐
 │   l1       r1   │
 │                 │
 │   l2       r2   │
 │                 │
 │   l3       r3   │
 └─────────────────┘
      rear
```

Each leg: 3 DOF (coxa-yaw, femur-pitch, tibia-pitch)

---

## 🚀 Getting Started

### 🔧 Requirements

- Python 3.9+
- Recommended:  
  `numpy`, `matplotlib`, `networkx`, `scipy`

```bash
pip install numpy matplotlib networkx scipy
```

---

### 📥 Installation

```bash
git clone https://github.com/ChrisDelta3Robotics/HexaPodSim2.0.git
cd HexaPodSim2.0
```

---

## 🧠 Features

- Inverse kinematics for rectangular body geometry
- Gait generation: tripod, ripple, wave
- Hierarchical control (sense-plan-act)
- Terrain-aware path planning (A*, PRM)
- Stability analysis (support polygon, center of mass)
- Simulated maneuvers: slalom, lane-change, centroid rotation
- Real-time GUI for control & visualization

---

## 🎮 Control Interface

Integrated GUI for real-time control and visualization (located in `hexapod/gui.py`).

**Movement Keys:**
- **w** (hold): Walk forward
- **s** (hold): Walk backward
- **a** (hold): Walk left
- **d** (hold): Walk right
- **q** (hold): Rotate left
- **e** (hold): Rotate right
- **h**: Lift body
- **b**: Lower body  
*Keys can be combined for multi-directional motion.*

**Gait Selection:**  
Switch between tripod, ripple, and wave gaits (button in GUI).

**Body Pose Manipulation:**  
- Joystick (bottom-right in GUI)
- Keyboard:
  - **i**: Tilt up
  - **k**: Tilt down
  - **j**: Tilt left
  - **l**: Tilt right
  - **m**: Reset pose

**GUI Layout:**
- Upper right: Robot simulation
- Upper left: Simulation settings
- Bottom right: Joystick control

**Notes:**
- Default matplotlib key bindings disabled
- Built with `matplotlib`, `tkinter`, optionally `pygame`
- Responsive and real-time feedback

---

## 📊 Performance Metrics

- Stability index (COM vs support polygon)
- Gait efficiency (stride length vs cycle time)
- Terrain adaptability score
- Energy estimation (torque × time)

---

## 🧪 Testing

Basic testing can be done by running the simulation directly:
```bash
python -m hexapod
```

For development testing, you can add test functions directly in the modules.

---

## 📚 References

- Gurel, C.S. (2017). Hexapod Modelling, Path Planning and Control. University of Manchester.
- Latombe, J.-C. (1991). Robot Motion Planning
- Libraries: NumPy, SciPy, NetworkX, Matplotlib

---

## 🛠 Contributing

Pull requests welcome!  
Please follow [PEP8](https://www.python.org/dev/peps/pep-0008/) and include docstrings. Open an issue to discuss major changes.

---

## 📄 License

MIT License. See [LICENSE.md](./LICENSE.md) for details.

---