# 🤖 HexaPodSim 2.0 Robot Visual Specification

## Robot Overview Diagram

### Top View - Rectangular Body Layout
```
                    FRONT
              ┌─────────────────┐
              │        ^        │
              │        │ X      │  
    L1 ●──────┤        │        ├──────● R1
              │        │        │
              │   ┌────┼────┐   │
              │   │    │    │   │
    L2 ●──────┤   │    +────┼───┼──────● R2  
              │   │  BODY   │   │      (Y ←)
              │   │ CENTER  │   │
              │   │         │   │
    L3 ●──────┤   └─────────┘   ├──────● R3
              │                 │
              │                 │
              └─────────────────┘
                    REAR

Coordinate System: X-forward, Y-left, Z-up (right-handed)
Body Dimensions: 20cm × 15cm (length × width)

Leg Positions (from body center):
- L1 (Left Front):   ( 0.10,  0.075, 0)
- R1 (Right Front):  ( 0.10, -0.075, 0)  
- L2 (Left Middle):  ( 0.00,  0.075, 0)
- R2 (Right Middle): ( 0.00, -0.075, 0)
- L3 (Left Rear):    (-0.10,  0.075, 0)
- R3 (Right Rear):   (-0.10, -0.075, 0)
```

### Side View - Leg Structure
```
                    Z (up)
                    ↑
                    │
     BODY ═══════════════════
                    │
                    ● ← Coxa Joint (4cm)
                    │
                    ●─────────● ← Femur Joint + Link (8cm)
                            ╱
                         ╱
                      ╱ 
                   ●  ← Tibia Joint + Link (12cm)
                  ╱
               ╱
            ● ← Foot (ground contact)
```

### Individual Leg Kinematics (3 DOF per leg)
```
Joint Configuration:

1. COXA (Hip Joint):
   - Type: Yaw rotation around Z-axis
   - Range: -90° to +90°
   - Convention: -90°=rightmost, 0°=center, +90°=leftmost
   - Length: 4cm

2. FEMUR (Upper Leg Joint):
   - Type: Pitch rotation around Y-axis  
   - Range: -90° to +90°
   - Length: 8cm

3. TIBIA (Lower Leg Joint):
   - Type: Pitch rotation around Y-axis
   - Range: -90° to +90°
   - Length: 12cm

Total: 18 DOF (6 legs × 3 joints each)
All joints equipped with PID controllers
```

### Leg Workspace Visualization
```
Side View Workspace (Femur-Tibia plane):

     ╭─────────────────╮
   ╱                     ╲
 ╱         REACHABLE       ╲
│           WORKSPACE        │
│             ╭─╮            │
 ╲           ╱   ╲          ╱
   ╲       ╱ BODY ╲       ╱
     ╲───╱    LEG   ╲───╱
         │  ATTACH   │
         │   POINT   │
         └───────────┘

Approximate radius: ~20cm (fully extended)
Dead zone: ~5cm (minimum reach)
```

### Gait Patterns - Leg Phase Diagrams

#### Tripod Gait (Fast, 50% duty factor)
```
Time →  0%    25%   50%   75%   100%
      ┌─────┬─────┬─────┬─────┬─────┐
  L1  │ ███ │     │ ███ │     │ ███ │  
  R1  │     │ ███ │     │ ███ │     │
  L2  │     │ ███ │     │ ███ │     │
  R2  │ ███ │     │ ███ │     │ ███ │
  L3  │ ███ │     │ ███ │     │ ███ │
  R3  │     │ ███ │     │ ███ │     │
      └─────┴─────┴─────┴─────┴─────┘

███ = Swing Phase (leg in air)
    = Stance Phase (leg on ground)

Groups: (L1,R2,L3) then (R1,L2,R3)
```

#### Wave Gait (Stable, 83% duty factor)
```
Time →  0%    17%   33%   50%   67%   83%   100%
      ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┐
  L1  │ ███ │     │     │     │     │     │ ███ │
  L2  │     │ ███ │     │     │     │     │     │
  L3  │     │     │ ███ │     │     │     │     │
  R3  │     │     │     │ ███ │     │     │     │
  R2  │     │     │     │     │ ███ │     │     │
  R1  │     │     │     │     │     │ ███ │     │
      └─────┴─────┴─────┴─────┴─────┴─────┴─────┘

Sequential: L1→L2→L3→R3→R2→R1 (repeat)
Only one leg moving at a time
```

#### Ripple Gait (Balanced, 67% duty factor)
```
Time →  0%    33%   67%   100%
      ┌─────┬─────┬─────┬─────┐
  L1  │ ███ │     │     │ ███ │
  R1  │     │     │ ███ │     │
  L2  │     │ ███ │     │     │
  R2  │ ███ │     │     │ ███ │
  L3  │     │     │ ███ │     │
  R3  │     │ ███ │     │     │
      └─────┴─────┴─────┴─────┘

Pairs with 180° offset: (L1,R2) (L2,R3) (L3,R1)
```

### Control Architecture Diagram
```
┌─────────────────────────────────────────────────────────┐
│                    USER INPUT                           │
│              (Keyboard/GUI Controls)                    │
└─────────────────┬───────────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────────────────────┐
│              HIGH LEVEL PLANNER                         │
│  ┌─────────────────┐    ┌─────────────────────────┐     │
│  │  Body Path      │    │   Footstep Planner      │     │
│  │  Planner (A*)   │    │   (Terrain Analysis)    │     │
│  └─────────────────┘    └─────────────────────────┘     │
└─────────────────┬───────────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────────────────────┐
│               MID LEVEL PLANNER                         │
│  ┌─────────────────┐    ┌─────────────────────────┐     │
│  │ Foot Trajectory │    │  Body Trajectory        │     │
│  │ Planner (PRM)   │    │  Planner (ZMP)          │     │
│  └─────────────────┘    └─────────────────────────┘     │
└─────────────────┬───────────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────────────────────┐
│              LOW LEVEL CONTROLLER                       │
│  ┌─────────────────┐    ┌─────────────────────────┐     │
│  │ Inverse         │    │   18 PID Controllers    │     │
│  │ Kinematics      │◄───┤   (Joint Control)       │     │
│  └─────────────────┘    └─────────────────────────┘     │
└─────────────────┬───────────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────────────────────┐
│                  ROBOT HARDWARE                         │
│    [L1]  [R1]  [L2]  [R2]  [L3]  [R3]                 │
│   (3DOF)(3DOF)(3DOF)(3DOF)(3DOF)(3DOF)                │
└─────────────────────────────────────────────────────────┘
```

### Joint Coordinate Frames (Denavit-Hartenberg)
```
For each leg (example: Right Front - R1):

Body Frame → Coxa Frame:
  Translation: (0.10, -0.075, 0)
  Rotation: Coxa angle (θ1) around Z-axis

Coxa Frame → Femur Frame:
  Translation: (0.04, 0, 0) along coxa
  Rotation: Femur angle (θ2) around Y-axis

Femur Frame → Tibia Frame:
  Translation: (0.08, 0, 0) along femur
  Rotation: Tibia angle (θ3) around Y-axis

Tibia Frame → Foot Frame:
  Translation: (0.12, 0, 0) along tibia
  Final foot position in world coordinates
```

### Physical Specifications
```
BODY:
├── Material: Lightweight composite/aluminum
├── Dimensions: 200mm × 150mm × 50mm (L×W×H)
├── Mass: 2.0 kg
├── Center of Mass: Geometric center
└── Mounting Points: 6 legs at specified positions

LEGS (×6):
├── Coxa Link:
│   ├── Length: 40mm
│   ├── Mass: 0.05kg
│   └── Material: Carbon fiber tube
├── Femur Link:
│   ├── Length: 80mm
│   ├── Mass: 0.08kg
│   └── Material: Carbon fiber tube
└── Tibia Link:
    ├── Length: 120mm
    ├── Mass: 0.12kg
    └── Material: Carbon fiber tube

JOINTS (×18):
├── Type: Servo motors with encoders
├── Range: -90° to +90° (all joints)
├── Resolution: 0.1° positioning accuracy
├── Control: Individual PID controllers
└── Feedback: Real-time position sensing

TOTAL SYSTEM:
├── Total Mass: ~3.5kg (body + legs + actuators)
├── Footprint: ~400mm × 350mm (legs extended)
├── Height: ~200mm (normal walking height)
└── DOF: 18 (3 per leg × 6 legs)
```

### Performance Characteristics
```
LOCOMOTION:
├── Max Speed: ~0.5 m/s (tripod gait)
├── Stability: 15° slope capability
├── Step Height: 30mm obstacle clearance
├── Turning Radius: In-place rotation capable
└── Gait Efficiency: Variable duty factors

CONTROL:
├── Update Rate: 30+ FPS simulation
├── PID Frequency: 100Hz joint control
├── Response Time: <100ms path planning
├── Positioning: ±1° joint accuracy
└── Safety: Real-time limit enforcement
```

This visual specification represents a sophisticated hexapod robot with rectangular body geometry, optimized for research applications in terrain-aware locomotion and gait exploration.