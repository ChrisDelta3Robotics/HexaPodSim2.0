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

🤖 Robot Overview
🔹 Rectangular Body Layout

    Six legs mounted in three pairs along the long edges:

        Left side: l1 (front), l2 (middle), l3 (rear)

        Right side: r1 (front), r2 (middle), r3 (rear)

          front
     ┌─────────────────┐
     │   l1       r1   │
     │                 │
     │   l2       r2   │
     │                 │
     │   l3       r3   │
     └─────────────────┘
          rear
Each leg has 3 degrees of freedom: coxa (yaw), femur (pitch), tibia (pitch)
🚀 Getting Started
🔧 Requirements

    Python 3.9+

    Recommended packages:

pip install numpy matplotlib networkx scipy

📥 Installation

git clone https://github.com/yourusername/hexapod-python-sim.git
cd hexapod-python-sim

🧠 Features

    Inverse kinematics tailored for rectangular body geometry

    Gait generation: tripod, ripple, wave

    Hierarchical control architecture (sense-plan-act)

    Terrain-aware path planning using A* and PRM

    Stability analysis via support polygon and center of mass projection

    Simulation of slalom, lane-change, and centroid rotation maneuvers

    Real-time GUI for interactive control and visualization

🎮 Control Interface (GUI)

This simulation includes a custom-built Graphical User Interface (GUI) for real-time control and visualization of the hexapod robot.
🕹 Movement Controls

    w (hold) → Walk forward

    s (hold) → Walk backward

    a (hold) → Walk left

    d (hold) → Walk right

    q (hold) → Rotate left

    e (hold) → Rotate right

    h → Lift the robot’s body

    b → Lower the robot’s body

    All movement keys can be combined for fluid multi-directional motion

🔁 Gait Selection

    A dedicated button allows switching between gaits (tripod, ripple, wave) during runtime.

🧍 Body Pose Manipulation

    Joystick in the bottom-right corner of the GUI allows intuitive pose adjustments.

    Alternatively, use keyboard shortcuts:

        i → Tilt body upward (look up)

        k → Tilt body downward (look down)

        j → Tilt body left

        l → Tilt body right

        m → Reset body pose to neutral

🖼 GUI Layout

    Upper right corner: Real-time simulation of the hexapod robot.

    Upper left corner: Key simulation settings and parameters.

    Bottom right corner: Joystick for body pose control.

⚙️ GUI Notes

    All default matplotlib key bindings are disabled to avoid conflicts.

    GUI is built using matplotlib, tkinter, and optionally pygame.

    Designed for responsive control and real-time feedback.

📊 Performance Metrics

    Stability index (center of mass vs support polygon)

    Gait efficiency (stride length vs cycle time)

    Terrain adaptability score

    Energy estimation (torque × time)

🧪 Testing

Run unit tests:

pytest tests/

Try a full simulation:

python examples/run_tripod_sim.py

📚 References

    Gurel, C.S. (2017). Hexapod Modelling, Path Planning and Control. University of Manchester.

    Latombe, J.-C. (1991). Robot Motion Planning

    Python libraries: NumPy, SciPy, NetworkX, Matplotlib

🛠 Contributing

Pull requests welcome. Please follow PEP8 and include docstrings. Open an issue to discuss major changes.
📄 License

MIT License. See LICENSE.md for details.

