# 🤖 HexaPodSim 2.0 — Star Configuration Hexapod Robot Simulation

A comprehensive Python framework for modeling, controlling, and visualizing a six-legged hexapod robot featuring an innovative **star-shaped leg configuration**. Built for advanced robotics research, real-time control, and modern GUI-based interaction.

---

## ⭐ Key Features

- **🌟 Star-Shaped Configuration**: Enhanced stability and maneuverability through angled front/back legs
- **🎮 Modern GUI Interface**: Black background with vibrant neon colors and intuitive controls
- **🤖 18 DOF Control**: Individual PID controllers for each joint with degree-based interface
- **🚶 Advanced Gaits**: Tripod, wave, and ripple locomotion patterns with real-time visualization
- **🗺️ Path Planning**: A* and PRM algorithms for autonomous navigation
- **📊 Real-time Analytics**: Live joint angles, foot trajectories, and robot status monitoring
- **⌨️ Natural Controls**: WASD + QE keyboard layout with virtual joystick for body pose

---

## 📦 Repository Structure

```bash
├── hexapod/                    # Core simulation modules
│   ├── HexaPodSim.py          # Main program entry point
│   ├── kinematics.py          # Forward/Inverse kinematics (star config)
│   ├── dynamics.py            # Physics and torque modeling
│   ├── gait.py                # Gait generators with timing control
│   ├── planner.py             # Path planning algorithms
│   ├── controller.py          # PID control system
│   ├── gui.py                 # Modern neon-themed GUI interface
│   └── utils.py               # Utility functions and constants
├── .github/
│   └── copilot-instructions.md # AI coding standards and conventions
├── README.md                   # Project documentation
├── IMPLEMENTATION.md           # Development roadmap (24 work packages)
├── ROBOT_DESIGN.md            # Visual specifications and diagrams
└── GUI_DESIGN.md              # Interface design specifications
```

---

## 🌟 Star Configuration Design

**Revolutionary Leg Layout:**

The star configuration provides superior stability and maneuverability compared to traditional rectangular layouts:

```
                    FRONT
              ┌─────────────────┐
              │        ^        │
              │        │ X      │  
        L1 ●╱─┤        │        ├─╲● R1
          ╱   │        │        │   ╲
       15°    │   ┌────┼────┐   │    15°
              │   │    │    │   │
    L2 ●──────┤   │    +────┼───┼──────● R2  
              │   │  BODY   │   │      (Y ←)
              │   │ CENTER  │   │
              │   │         │   │
        L3 ●╲─┤   └─────────┘   ├─╱● R3
          ╲   │                 │   ╱
       15°    │                 │    15°
              └─────────────────┘
                    REAR

★ CONFIGURATION ANGLES:
- L1, R1 (Front): 105° (90° + 15°) - Enhanced forward stability
- L2, R2 (Middle): 90° (perpendicular) - Lateral support
- L3, R3 (Back): 75° (90° - 15°) - Improved turning agility
```

**Benefits:**
- **Enhanced Stability**: Forward-angled front legs improve forward motion stability
- **Superior Maneuverability**: Backward-angled rear legs enable tighter turning radius
- **Adaptive Footprint**: Larger effective support polygon during locomotion
- **Configurable**: `INWARD_LEG_ANGLE` parameter allows real-time adjustment

---

## 🚀 Getting Started

### 🔧 Requirements

- **Python 3.9+** with the following packages:
- **Core Dependencies**: `numpy`, `matplotlib`, `networkx`, `scipy`
- **Optional**: `tkinter` (usually included with Python)

```bash
pip install numpy matplotlib networkx scipy
```

---

### 📥 Installation

```bash
git clone https://github.com/ChrisDelta3Robotics/HexaPodSim2.0.git
cd HexaPodSim2.0
```

### 🚀 Quick Start

**Basic simulation with GUI:**
```bash
python hexapod/HexaPodSim.py
```

**Advanced options:**
```bash
# Start with specific gait pattern
python hexapod/HexaPodSim.py --gait tripod

# Run without GUI (console mode)
python hexapod/HexaPodSim.py --no-gui

# Custom star configuration angle
python hexapod/HexaPodSim.py --leg-angle 20.0

# Run built-in demo sequences
python hexapod/HexaPodSim.py --demo walk

# Enable debug output
python hexapod/HexaPodSim.py --debug
```

---

## 🎮 GUI Interface

The modern interface features a **black background with vibrant neon colors** and intuitive control layout:

### 🎛️ Control Sections

**Movement Controls (Bottom-Left):**
```
[Q] ⤺    [E] ⤻     ← Turn Left/Right
    [W] ▲          ← Forward
[A]◄─┼─►[D]        ← Strafe Left/Right  
    [S] ▼          ← Backward
```

**Virtual Joystick (Bottom-Right):**
- Drag-to-control body pose (pitch, roll, yaw)
- Real-time position feedback
- Quick preset buttons (Center, Crouch, Stand, Lean)

**3D Visualization (Top-Right):**
- Real-time robot rendering with star configuration
- Interactive camera controls (rotate, pan, zoom)
- Color-coded legs for easy identification

**Main Workspace (Center-Left):**
- Live gait pattern visualization
- All 18 joint angles in degrees
- Foot trajectory tracking
- Robot status and performance metrics

### 🌈 Color Coding

Each leg has a unique neon color for easy identification:
- **L1 (Left Front)**: Neon Green (`#00FF00`)
- **R1 (Right Front)**: Neon Cyan (`#00FFFF`)
- **L2 (Left Middle)**: Neon Magenta (`#FF00FF`)
- **R2 (Right Middle)**: Neon Yellow (`#FFFF00`)
- **L3 (Left Back)**: Neon Orange (`#FF6600`)
- **R3 (Right Back)**: Neon Pink (`#FF0080`)

---

## 🧠 Advanced Features

### 🚶 Gait Patterns
- **Tripod Gait**: Fast locomotion (50% duty factor)
- **Wave Gait**: Maximum stability (83% duty factor)
- **Ripple Gait**: Balanced performance (67% duty factor)
- **Real-time switching** between gait patterns

### 🎯 Control Systems
- **18 Individual PID Controllers**: One per joint with degree-based interface
- **Star Configuration**: Configurable `INWARD_LEG_ANGLE` parameter
- **Body Pose Control**: 6-DOF body positioning via virtual joystick
- **Emergency Stop**: Immediate halt functionality

### 🗺️ Path Planning
- **A* Algorithm**: Optimal path finding in known environments
- **PRM (Probabilistic Roadmap)**: Efficient navigation in complex spaces
- **Real-time visualization** of planned vs actual paths
- **Obstacle avoidance** integration

### 📊 Real-time Monitoring
- **Joint Status**: All angles, targets, and PID outputs in degrees
- **Robot Pose**: Position (X, Y, Z) and orientation
- **Performance Metrics**: FPS, CPU usage, memory consumption
- **Foot Trajectories**: Planned vs actual foot positions
---

## 📚 Documentation

### 🏗️ Development Resources
- **[IMPLEMENTATION.md](IMPLEMENTATION.md)**: 24-package development roadmap with AI-optimized prompts
- **[ROBOT_DESIGN.md](ROBOT_DESIGN.md)**: Comprehensive visual specifications and ASCII diagrams
- **[GUI_DESIGN.md](GUI_DESIGN.md)**: Complete interface design with mockups
- **[.github/copilot-instructions.md](.github/copilot-instructions.md)**: AI coding standards and conventions

### 🔧 Technical Specifications
- **Body Dimensions**: 20cm × 15cm × 5cm (Length × Width × Height)
- **Leg Segments**: Coxa (4cm), Femur (8cm), Tibia (12cm)
- **Joint Range**: -90° to +90° for all rotational joints
- **Control Frequency**: 100Hz minimum for real-time operation
- **GUI Refresh**: 60 FPS target for smooth visualization

---

## 🛠️ Development

### 🚀 Implementation Roadmap

The project follows a systematic 24-package development approach:

**Phase 1**: Core Mathematics & Kinematics (4 packages)
**Phase 2**: Robot Dynamics & Physics (3 packages)  
**Phase 3**: Gait Generation & Control (4 packages)
**Phase 4**: Path Planning & Navigation (4 packages)
**Phase 5**: GUI & Visualization (3 packages)
**Phase 6**: Integration & Testing (4 packages)
**Phase 7**: Documentation & Examples (2 packages)

See [IMPLEMENTATION.md](IMPLEMENTATION.md) for detailed work packages with specific AI prompts.

### 🎯 Coding Standards

All development follows strict standards defined in [.github/copilot-instructions.md](.github/copilot-instructions.md):
- **Degree-based calculations** throughout (never radians)
- **Individual PID controllers** for each of 18 joints
- **Star configuration** as configurable parameter
- **Modern GUI** with black/neon color scheme
- **Comprehensive testing** and documentation

### 🧪 Testing

```bash
# Run unit tests (when implemented)
python -m pytest tests/

# Performance benchmarks
python hexapod/HexaPodSim.py --debug --demo walk

# Validate kinematics
python -c "from hexapod.kinematics import validate_star_config; validate_star_config()"
```

---

## 🎮 Usage Examples

### Basic Operation
```python
from hexapod.HexaPodSim import HexapodSimulator

# Initialize with star configuration
sim = HexapodSimulator(inward_leg_angle=15.0)

# Start tripod gait
sim.set_gait('tripod')
sim.start_walking()

# Control body pose
sim.set_body_pose(pitch=5.0, roll=0.0, yaw=10.0)
```

### Advanced Control
```python
# Custom star angle
sim = HexapodSimulator(inward_leg_angle=20.0)

# Path planning
target = (2.0, 1.0, 0.0)  # X, Y, Z in meters
path = sim.plan_path(target, algorithm='A*')
sim.execute_path(path)

# Real-time joint control
joint_angles = sim.get_joint_angles()  # All in degrees
sim.set_joint_target('L1_coxa', 45.0)  # Degree-based
```

---

## 🤝 Contributing

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Follow** coding standards in [.github/copilot-instructions.md](.github/copilot-instructions.md)
4. **Test** your changes thoroughly
5. **Commit** with descriptive messages (`git commit -m 'feat: Add amazing feature'`)
6. **Push** to the branch (`git push origin feature/amazing-feature`)
7. **Open** a Pull Request

### 📋 Development Guidelines
- Use **degrees** for all angle calculations
- Implement **PID controllers** for joint control
- Follow **star configuration** conventions
- Maintain **GUI design standards** (black/neon theme)
- Include **comprehensive tests** and documentation

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🙏 Acknowledgments

- **Hackaday.io Project**: [Hexapod Modelling Path Planning and Control](https://hackaday.io/project/21904-hexapod-modelling-path-planning-and-control) for research foundation
- **Python Robotics Community** for algorithm implementations
- **Modern GUI Design** inspired by contemporary neon aesthetics

---

## 📞 Contact

**Chris Delta3 Robotics**
- GitHub: [@ChrisDelta3Robotics](https://github.com/ChrisDelta3Robotics)
- Project: [HexaPodSim2.0](https://github.com/ChrisDelta3Robotics/HexaPodSim2.0)

---

*Built with ❤️ for the robotics community*
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

Run the main simulation:
```bash
python hexapod/HexaPodSim.py
```

Test with different configurations:
```bash
python hexapod/HexaPodSim.py --demo walk      # Test walking demo
python hexapod/HexaPodSim.py --gait wave      # Test wave gait
python hexapod/HexaPodSim.py --debug          # Enable debug output
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