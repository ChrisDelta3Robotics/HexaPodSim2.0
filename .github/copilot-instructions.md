```instructions
---
applyTo: "**/*.py"
---
# GitHub Copilot Instructions for HexaPodSim 2.0

This file contains project-specific coding standards and conventions for the HexaPodSim 2.0 hexapod robot simulation framework. All AI assistants working on this project must follow these guidelines to ensure consistency and quality.

## Project Overview

HexaPodSim 2.0 is a comprehensive hexapod robot simulation framework featuring:
- **Star-shaped leg configuration** for enhanced stability and maneuverability
- **18 DOF control system** with individual PID controllers for each joint
- **Real-time 3D visualization** with modern neon-themed GUI interface
- **Advanced locomotion** including tripod, wave, and ripple gait patterns
- **Path planning algorithms** (A*, PRM) for autonomous navigation
- **Forward/Inverse kinematics** using Denavit-Hartenberg convention

## Robot Configuration Standards

### Star-Shaped Leg Configuration
- **INWARD_LEG_ANGLE = 15°** (configurable parameter)
- **Front legs (L1, R1)**: Base angle = 90° + 15° = 105° from body centerline
- **Middle legs (L2, R2)**: Base angle = 90° (perpendicular to body)
- **Back legs (L3, R3)**: Base angle = 90° - 15° = 75° from body centerline
- **Implementation**: Apply base rotation as body-frame transform before D-H calculations
- **Benefits**: Enhanced forward stability, improved turning maneuverability

### Joint Angle Conventions
- **ALL angles in degrees** (never radians throughout entire codebase)
- **Joint limits**: -90° to +90° for all rotational joints
- **Position convention**: -90° (rightmost), 0° (center), +90° (leftmost)
- **Naming**: Use consistent naming: `coxa_angle`, `femur_angle`, `tibia_angle`
- **Positive rotations**: Follow right-hand rule around joint axis

### Leg Identification and Coloring
```python
LEG_NAMES = ['L1', 'R1', 'L2', 'R2', 'L3', 'R3']
LEG_COLORS = {
    'L1': '#00FF00',  # Neon Green (Left Front)
    'R1': '#00FFFF',  # Neon Cyan (Right Front)  
    'L2': '#FF00FF',  # Neon Magenta (Left Middle)
    'R2': '#FFFF00',  # Neon Yellow (Right Middle)
    'L3': '#FF6600',  # Neon Orange (Left Back)
    'R3': '#FF0080'   # Neon Pink (Right Back)
}
```

## PID Controller Requirements

### Individual Joint Control
- **Each joint has its own PID controller** (18 total PID instances)
- **Degree-based interface**: All setpoints and feedback in degrees
- **Standard PID parameters**: Kp, Ki, Kd with anti-windup protection
- **Real-time operation**: 100Hz control loop minimum
- **Error handling**: Graceful degradation on sensor failures

### PID Implementation Standards
```python
class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_limits: tuple = (-90.0, 90.0)):
        """All parameters in degrees"""
        
    def update(self, setpoint_deg: float, feedback_deg: float, dt: float) -> float:
        """Returns control output in degrees"""
```

## GUI Design Standards

### Color Scheme (Black + Neon)
```python
GUI_COLORS = {
    'background': '#000000',     # Pure Black
    'text_primary': '#00FF00',   # Neon Green
    'text_secondary': '#00FFFF', # Neon Cyan
    'accent_1': '#FF00FF',       # Neon Magenta
    'accent_2': '#FFFF00',       # Neon Yellow
    'warning': '#FF4444',        # Neon Red
    'success': '#44FF44'         # Bright Green
}
```

### Control Layout Standards
- **Single window design** with organized panels
- **3D visualization**: Top-right corner (35% width)
- **Main workspace**: Center-left (60% width) 
- **Control buttons**: Bottom-left with natural keyboard layout
- **Virtual joystick**: Bottom-right for body pose control

### Keyboard Control Mapping
```
Movement Controls:
[Q] ⤺    [E] ⤻     ← Turning Left/Right
    [W] ▲          ← Forward
[A]◄─┼─►[D]        ← Strafe Left/Right  
    [S] ▼          ← Backward
```

## Code Quality Standards

### Python Standards
- **PEP 8 compliance** with 88-character line length
- **Type hints required** for all function parameters and returns
- **Comprehensive docstrings** using Google/NumPy style
- **Error handling**: Use appropriate exceptions with clear messages
- **Logging**: Use Python logging module with appropriate levels

### Function and Class Design
```python
def forward_kinematics(joint_angles_deg: np.ndarray, 
                      leg_id: str, 
                      inward_leg_angle: float = 15.0) -> np.ndarray:
    """
    Calculate forward kinematics for hexapod leg with star configuration.
    
    Args:
        joint_angles_deg: Joint angles in degrees [coxa, femur, tibia]
        leg_id: Leg identifier ('L1', 'R1', 'L2', 'R2', 'L3', 'R3')
        inward_leg_angle: Star configuration angle in degrees
        
    Returns:
        End-effector position [x, y, z] in meters
        
    Raises:
        ValueError: If joint angles exceed [-90, 90] degree limits
    """
```

### Testing Requirements
- **Unit tests required** for all mathematical functions
- **Integration tests** for multi-component interactions
- **Validation tests** using known kinematic solutions
- **Performance tests** ensuring real-time operation targets
- **GUI tests** for user interaction workflows

## Mathematical Conventions

### Coordinate Systems
- **World frame**: X-forward, Y-left, Z-up (right-handed)
- **Body frame**: Origin at geometric center of hexapod body
- **Leg frames**: Denavit-Hartenberg convention with degree calculations
- **Star rotation**: Applied as body-frame rotation before D-H transforms

### Physical Parameters
```python
# Robot dimensions (meters)
BODY_LENGTH = 0.20      # 20cm
BODY_WIDTH = 0.15       # 15cm
COXA_LENGTH = 0.04      # 4cm
FEMUR_LENGTH = 0.08     # 8cm  
TIBIA_LENGTH = 0.12     # 12cm

# Star configuration
INWARD_LEG_ANGLE = 15.0  # degrees (configurable)
```

## Performance Requirements

### Real-time Operation Targets
- **Control loop**: 100Hz minimum (10ms cycle time)
- **GUI refresh**: 60 FPS for smooth visualization
- **Kinematics**: < 1ms per leg calculation
- **Path planning**: < 100ms for local planning updates
- **Memory usage**: < 512MB for full simulation

### Optimization Guidelines
- **NumPy vectorization** for matrix operations
- **Caching** of expensive calculations when possible
- **Profiling** critical paths for performance bottlenecks
- **Memory management** for long-running simulations

## Integration Standards

### Module Interface Design
- **Clear separation** between kinematics, dynamics, control, and GUI
- **Consistent parameter passing** using degrees throughout
- **Event-driven architecture** for GUI interactions
- **Modular testing** enabling independent component validation

### Configuration Management
```python
@dataclass
class RobotConfig:
    inward_leg_angle: float = 15.0  # Star configuration angle
    body_length: float = 0.20       # Body dimensions
    body_width: float = 0.15
    coxa_length: float = 0.04       # Link lengths
    femur_length: float = 0.08
    tibia_length: float = 0.12
    pid_gains: dict = field(default_factory=dict)  # PID parameters
```

## Documentation Requirements

### Code Documentation
- **Module docstrings** explaining purpose and usage
- **Class docstrings** with attribute descriptions
- **Function docstrings** with Args/Returns/Raises sections
- **Inline comments** for complex algorithmic sections

### User Documentation
- **README**: Installation, usage, and examples
- **Implementation Guide**: Step-by-step development roadmap
- **GUI Design**: Interface specifications and interactions
- **Robot Design**: Mechanical and mathematical specifications

## Version Control Standards

### Commit Message Format
```
Type: Brief description

- Detailed change 1
- Detailed change 2
- Benefits/impact of changes

Examples:
feat: Add star configuration to forward kinematics
fix: Correct PID controller degree conversion
docs: Update implementation guide for GUI integration
```

### Branch Strategy
- **main**: Stable, tested code only
- **develop**: Integration of new features
- **feature/**: Individual feature development
- **hotfix/**: Critical bug fixes

---

**Remember**: This project prioritizes real-time performance, user-friendly interfaces, and mathematical accuracy. All code should reflect these core principles while maintaining the star configuration design and modern GUI aesthetic.

```
