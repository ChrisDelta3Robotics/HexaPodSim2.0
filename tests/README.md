# 🧪 HexaPodSim 2.0 Tests

This directory contains all test files, demo scripts, and experimental code for HexaPodSim 2.0.

## 📁 Test Categories

### 🤖 Core System Tests
- `test_kinematics.py` - Forward/inverse kinematics validation
- `test_inverse_kinematics.py` - IK solver testing
- `test_gait.py` - Gait generation and patterns
- `test_motion.py` - Motion control systems
- `test_controller.py` / `test_controllers.py` - PID controller tests
- `test_planner.py` - Path planning algorithms

### 🎮 GUI and Visualization Tests
- `test_gui.py` - Main GUI functionality
- `test_gui_simple.py` - Simplified GUI tests
- `test_gui_layout.py` - Layout and positioning
- `test_minimal_gui.py` - Minimal GUI implementation
- `test_3d_viz.py` - 3D visualization components
- `test_3d_visibility.py` - 3D rendering visibility
- `test_forced_3d.py` - Force-enabled 3D display
- `test_window_position.py` - Window positioning tests

### 🖥️ Display and Screen Tests
- `display_test.py` - Display compatibility testing
- `simple_gui.py` - Simple GUI implementation test

### 🏗️ Physics and Dynamics Tests
- `test_body_dynamics.py` - Robot body dynamics
- `test_leg_dynamics.py` - Individual leg dynamics
- `test_contact_forces.py` - Ground contact simulation
- `test_adaptive.py` - Adaptive control systems

### 🔧 Configuration and Integration Tests
- `test_robot_configuration.py` - Robot setup validation
- `test_phase6.py` - Phase 6 integration test
- `test_phase6_basic.py` - Basic Phase 6 functionality

### 🎯 Demo Scripts
- `demo_phase5.py` - Phase 5 demonstration script

## 🚀 Running Tests

### Individual Tests
```bash
# Run a specific test
python tests/test_kinematics.py

# Run GUI tests
python tests/test_gui.py
python tests/test_3d_viz.py
```

### Multiple Tests
```bash
# Run all tests in the directory
cd tests/
python -m pytest .

# Or run with unittest
python -m unittest discover -s tests -p "test_*.py"
```

### GUI Tests with Multi-Window Mode
```bash
# Test multi-window GUI
python tests/test_window_position.py
python tests/test_gui_layout.py
```

## 📝 Test Development Guidelines

- All test files should start with `test_`
- Demo and experimental files should be clearly named
- Include docstrings explaining the test purpose
- Use meaningful assertions and error messages
- Test both single-window and multi-window GUI modes where applicable

## 🐛 Debugging

Most tests include debug output and logging. Check the console output and `hexapodsim.log` for detailed information when tests fail.