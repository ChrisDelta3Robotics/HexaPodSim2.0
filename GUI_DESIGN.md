# 🎮 HexaPodSim 2.0 GUI Design Specification

## GUI Overview

HexaPodSim 2.0 features **dual GUI modes** to accommodate different display sizes and user preferences. Both interfaces share the same modern design philosophy with black background and neon color accents.

### Design Philosophy
- **Adaptive Layout**: Two modes - single-window and multi-window
- **Color Scheme**: Black background with vibrant neon accents (cyan, green, magenta, yellow)
- **Modern Flat Design**: Clean buttons and panels with subtle shadows
- **User-Friendly**: Intuitive placement and clear visual hierarchy
- **Screen-Aware**: Optimized for both large (1400x900+) and small (720x720) displays

---

## 🖥️ Single-Window Mode (1400x900)

Perfect for large displays and traditional desktop setups with integrated interface:

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│ 🤖 HexaPodSim 2.0 - Hexapod Robot Simulation                                    [─][□][×]│
├─────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                         │
│  ┌─────────────────────────────────────────────────────┐  ┌─────────────────────────┐   │
│  │                                                     │  │    3D ROBOT VIEW        │   │
│  │                                                     │  │                         │   │
│  │                                                     │  │         ╱╲              │   │
│  │                                                     │  │       ╱    ╲            │   │
│  │                MAIN WORKSPACE                       │  │     ╱   🤖   ╲          │   │
│  │                                                     │  │   ╱           ╲        │   │
│  │            (Gait Visualization,                     │  │ ╱               ╲      │   │
│  │             Joint Angles,                           │  │ L1  L2  L3  R3  R2  R1  │   │
│  │             Foot Trajectories,                      │  │                         │   │
│  │             Robot Status)                           │  │    Forward Kinematics   │   │
│  │                                                     │  │      Real-time 3D       │   │
│  │                                                     │  │                         │   │
│  │                                                     │  └─────────────────────────┘   │
│  │                                                     │                                │
│  │                                                     │                                │
│  │                                                     │                                │
│  │                                                     │                                │
│  │                                                     │                                │
│  │                                                     │                                │
│  │                                                     │                                │
│  │                                                     │                                │
│  │                                                     │                                │
│  └─────────────────────────────────────────────────────┘                                │
│                                                                                         │
│  ┌─────────────────────────────────┐                       ┌─────────────────────────┐   │
│  │       CONTROL BUTTONS           │                       │    VIRTUAL JOYSTICK     │   │
│  │                                 │                       │                         │   │
│  │  [W]     [ GAIT ]    [PATH]     │                       │           ●             │   │
│  │ [A][S]   [ DEMO ]   [RESET]     │                       │         ╱   ╲           │   │
│  │  [D]     [START]    [STOP ]     │                       │       ╱   ⊕   ╲         │   │
│  │          [Q] [E]                │                       │     ╱           ╲       │   │
│  │       (Turn L/R)                │                       │   ╱               ╲     │   │
│  │                                 │                       │                         │   │
│  │                                 │                       │   Body Pose Control     │   │
│  └─────────────────────────────────┘                       └─────────────────────────┘   │
│                                                                                         │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## Detailed Panel Specifications

### 1. Main Workspace (Center-Left, 60% width)

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              MAIN WORKSPACE                                        │
├─────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                     │
│  ┌─────────────────────────────────────────────────────────────────────────────┐   │
│  │                        GAIT PATTERN DISPLAY                                │   │
│  │                                                                             │   │
│  │  Current Gait: [TRIPOD ▼] │ Speed: 1.2 m/s │ Duty Factor: 50%             │   │
│  │                                                                             │   │
│  │  Time →  0%    25%   50%   75%   100%                                      │   │
│  │        ┌─────┬─────┬─────┬─────┬─────┐                                      │   │
│  │    L1  │ ███ │     │ ███ │     │ ███ │  ◄── Neon Green                    │   │
│  │    R1  │     │ ███ │     │ ███ │     │  ◄── Neon Cyan                     │   │
│  │    L2  │     │ ███ │     │ ███ │     │  ◄── Neon Magenta                  │   │
│  │    R2  │ ███ │     │ ███ │     │ ███ │  ◄── Neon Yellow                   │   │
│  │    L3  │ ███ │     │ ███ │     │ ███ │  ◄── Neon Orange                   │   │
│  │    R3  │     │ ███ │     │ ███ │     │  ◄── Neon Pink                     │   │
│  │        └─────┴─────┴─────┴─────┴─────┘                                      │   │
│  │                                                                             │   │
│  │  ███ = Swing Phase (leg in air)     ░░░ = Stance Phase (leg on ground)     │   │
│  └─────────────────────────────────────────────────────────────────────────────┘   │
│                                                                                     │
│  ┌─────────────────────────────────────────────────────────────────────────────┐   │
│  │                         JOINT ANGLES (DEGREES)                             │   │
│  │                                                                             │   │
│  │  LEFT LEGS               │  RIGHT LEGS                                     │   │
│  │  ┌─────────────────────┐ │  ┌─────────────────────┐                       │   │
│  │  │ L1: C: 45° F:-12° T: 30° │ │ R1: C:-45° F:-12° T: 30°                  │   │
│  │  │ L2: C: 90° F: 15° T:-45° │ │ R2: C:-90° F: 15° T:-45°                  │   │
│  │  │ L3: C:135° F: 22° T: 18° │ │ R3: C:-135° F: 22° T: 18°                 │   │
│  │  └─────────────────────┘ │  └─────────────────────┘                       │   │
│  │                           │                                                │   │
│  │  C=Coxa, F=Femur, T=Tibia │  Color: Neon Green = Target                   │   │
│  │                           │         Neon Red = Current                     │   │
│  └─────────────────────────────────────────────────────────────────────────────┘   │
│                                                                                     │
│  ┌─────────────────────────────────────────────────────────────────────────────┐   │
│  │                       FOOT TRAJECTORIES                                    │   │
│  │                                                                             │   │
│  │      Y (Left)                                                               │   │
│  │        ▲                                                                    │   │
│  │        │     ● L1 (Neon Green)                                              │   │
│  │        │   ●   L2 (Neon Cyan)                                               │   │
│  │────────┼─●─────────► X (Forward)                                            │   │
│  │        │   ●   L3 (Neon Magenta)                                            │   │
│  │        │     ● R3 (Neon Yellow)                                             │   │
│  │        │   ●   R2 (Neon Orange)                                             │   │
│  │        │     ● R1 (Neon Pink)                                               │   │
│  │                                                                             │   │
│  │  ═══════ Planned Path    ┈┈┈┈┈┈┈ Actual Path    ● Current Position         │   │
│  └─────────────────────────────────────────────────────────────────────────────┘   │
│                                                                                     │
│  ┌─────────────────────────────────────────────────────────────────────────────┐   │
│  │                        ROBOT STATUS                                        │   │
│  │                                                                             │   │
│  │  Position: X: 1.25m  Y: 0.33m  Z: 0.15m     Orientation: 15.3°            │   │
│  │  Velocity: 1.2 m/s   Angular: 5.7°/s        Battery: ████████░░ 82%       │   │
│  │  CPU Load: ██████░░░░ 60%    Memory: ████░░░░░░ 40%    FPS: 60             │   │
│  │                                                                             │   │
│  │  Status: [🟢 RUNNING]  Mode: [AUTONOMOUS]  Errors: [0]  Warnings: [1]      │   │
│  └─────────────────────────────────────────────────────────────────────────────┘   │
│                                                                                     │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

### 2. 3D Robot View (Top-Right, 35% width)

```
┌─────────────────────────────────────┐
│           3D ROBOT VIEW             │
├─────────────────────────────────────┤
│                                     │
│    ┌─── View Controls ───┐          │
│    │ [⟲] [🔍+] [🔍-] [⌂] │          │
│    └─────────────────────┘          │
│                                     │
│         ╱╲                          │
│       ╱    ╲                        │
│     ╱   🤖   ╲                      │
│   ╱           ╲                     │
│ ╱               ╲                   │
│                                     │
│ L1              R1 ◄── Neon Green   │
│   ╲            ╱                    │
│     ╲        ╱                      │
│ L2 ── ╲    ╱ ── R2 ◄── Neon Cyan    │
│         ╲╱                          │
│          │                          │
│ L3 ──────┼────── R3 ◄── Neon Mag    │
│                                     │
│  Grid: [ON]  Shadows: [ON]          │
│  Coord: [ON] Trails: [OFF]          │
│                                     │
│  ┌─ Camera Controls ─┐              │
│  │ Azimuth:    45°   │              │
│  │ Elevation:  30°   │              │
│  │ Distance:  2.5m   │              │
│  └───────────────────┘              │
│                                     │
└─────────────────────────────────────┘
```

### 3. Control Buttons (Bottom-Left)

```
┌───────────────────────────────────────┐
│           CONTROL BUTTONS             │
├───────────────────────────────────────┤
│                                       │
│  ┌─── Movement ───┐  ┌─── Actions ──┐ │
│  │                │  │              │ │
│  │   [Q] ⤺  [E] ⤻ │  │ [GAIT ▼]     │ │
│  │      [W] ▲     │  │ [DEMO ▼]     │ │
│  │   [A]◄─┼─►[D]  │  │ [PATH ▼]     │ │
│  │      [S] ▼     │  │              │ │
│  │                │  │ [START/STOP] │ │
│  │  WASD + QE     │  │              │ │
│  └────────────────┘  └──────────────┘ │
│                                       │
│  ┌─── System ─────┐  ┌─── Quick ────┐ │
│  │                │  │              │ │
│  │ [RESET]        │  │ [CROUCH]     │ │
│  │ [SAVE]         │  │ [STAND]      │ │
│  │ [LOAD]         │  │ [CENTER]     │ │
│  │ [SETTINGS]     │  │ [LEAN]       │ │
│  │                │  │              │ │
│  └────────────────┘  └──────────────┘ │
│                                       │
│  ┌─────── Dropdowns ────────────────┐ │
│  │                                  │ │
│  │ Gait: [Tripod    ▼]             │ │
│  │ Demo: [Basic Walk ▼]             │ │
│  │ Path: [A* Search  ▼]             │ │
│  │                                  │ │
│  └──────────────────────────────────┘ │
│                                       │
└───────────────────────────────────────┘
```

### 4. Virtual Joystick (Bottom-Right)

```
┌─────────────────────────────────┐
│        VIRTUAL JOYSTICK         │
├─────────────────────────────────┤
│                                 │
│  ┌─── Body Pose Control ───┐    │
│  │                         │    │
│  │           ●             │    │
│  │         ╱   ╲           │    │
│  │       ╱   ⊕   ╲         │    │
│  │     ╱           ╲       │    │
│  │   ╱      ││       ╲     │    │
│  │          ││             │    │
│  │     Drag to move        │    │
│  │     robot pose          │    │
│  │                         │    │
│  └─────────────────────────┘    │
│                                 │
│  Current Pose:                  │
│  • X: +0.12m  (Forward/Back)    │
│  • Y: -0.05m  (Left/Right)      │
│  • Z: +0.03m  (Up/Down)         │
│  • Pitch: 5°   (Tilt)           │
│  • Roll: -2°   (Lean)           │
│  • Yaw: 15°    (Rotation)       │
│                                 │
│  ┌──── Quick Presets ────┐      │
│  │ [CENTER] [CROUCH]     │      │
│  │ [STAND]  [LEAN]       │      │
│  └───────────────────────┘      │
│                                 │
└─────────────────────────────────┘
```

---

## Color Scheme Specification

### Primary Colors (Neon on Black)
```
Background:     #000000 (Pure Black)
Text Primary:   #00FF00 (Neon Green)
Text Secondary: #00FFFF (Neon Cyan)
Accent 1:       #FF00FF (Neon Magenta)
Accent 2:       #FFFF00 (Neon Yellow)
Accent 3:       #FF6600 (Neon Orange)
Accent 4:       #FF0080 (Neon Pink)
Warning:        #FF4444 (Neon Red)
Success:        #44FF44 (Bright Green)
```

### UI Element Colors
```
Buttons:        Gradient with neon borders and glow effects
Button Hover:   Brighter glow, slight scale increase
Button Active:  Inner glow, slight inset shadow
Panels:         Dark gray (#1a1a1a) with neon borders
Input Fields:   Dark gray with neon cyan focus outline
Progress Bars:  Neon gradient fill on dark background
```

---

## Interactive Features

### Keyboard Controls
```
Movement:
  W, A, S, D    - Robot walking direction
  Q, E          - Turn left/right
  SPACE         - Emergency stop
  TAB           - Cycle through UI elements

Camera:
  Arrow Keys    - Rotate 3D view
  +/-           - Zoom in/out
  HOME          - Reset camera position

System:
  F1            - Help/shortcuts
  F5            - Reset simulation
  F11           - Toggle fullscreen
  ESC           - Cancel current action
```

### Mouse Interactions
```
3D View:
  Left Click + Drag   - Rotate camera
  Right Click + Drag  - Pan camera
  Scroll Wheel        - Zoom in/out
  Double Click        - Focus on robot

Joystick:
  Left Click + Drag   - Control body pose
  Right Click         - Reset to center
  Double Click        - Auto-level robot

Buttons:
  Hover              - Glow effect
  Click              - Execute action
  Right Click        - Context menu (where applicable)
```

---

## Status Indicators

### Robot State Icons
```
🟢 RUNNING      - Normal operation
🟡 PAUSED       - Simulation paused
🔴 ERROR        - Critical error occurred
🟠 WARNING      - Non-critical warning
🔵 MANUAL       - Manual control mode
🟣 AUTO         - Autonomous mode
⚪ DISCONNECTED - No robot connection
```

### Connection Status
```
████████████ 100%  - Excellent connection
██████████░░ 83%   - Good connection  
████████░░░░ 67%   - Fair connection
██████░░░░░░ 50%   - Poor connection
████░░░░░░░░ 33%   - Weak connection
██░░░░░░░░░░ 17%   - Very poor connection
░░░░░░░░░░░░ 0%    - No connection
```

---

## Responsive Design Notes

The GUI offers **dual modes** to accommodate different display sizes:

### 🖥️ Single-Window Mode
- **Minimum**: 1280x720 (panels stack vertically if needed)
- **Optimal**: 1400x900+ (full layout as shown above)
- **Maximum**: 4K displays (UI scales 2x with crisp neon effects)

### 🪟 Multi-Window Mode
- **Target**: 720x720 displays
- **Layout**: 4 optimized windows with automatic positioning
- **Performance**: Optimized update rates per window type
- **Management**: Independent window control with synchronized state

Both modes maintain aspect ratios and functionality across different screen sizes while preserving the modern, user-friendly neon aesthetic.

---

## 🪟 Multi-Window Mode Detailed Design

### Window Layout Overview

```
┌─────────────────┬──────────────────────────────┐
│   Control       │       3D Robot               │
│   Window        │       Visualization          │
│   (350x650)     │       (700x700)              │
│                 │                              │
│   Position:     │   Position:                  │
│   X: 10         │   X: 380                     │
│   Y: 10         │   Y: 10                      │
│                 │                              │
├─────────────────┼──────────────────────────────┤
│   Gait Window   │   Data Window                │
│   (700x350)     │   (350x350)                  │
│                 │                              │
│   Position:     │   Position:                  │
│   X: 10         │   X: 730                     │
│   Y: 680        │   Y: 680                     │
└─────────────────┴──────────────────────────────┘
```

### 1. Control Window (350x650)

```
┌─────────────────────────────────────┐
│ 🤖 CONTROL CENTER                   │
├─────────────────────────────────────┤
│ ┌─── System Status ──────────────┐  │
│ │ Mode: Multi-Window             │  │
│ │ Simulation: Active             │  │
│ │ Gait: Tripod                   │  │
│ │ Phase: 45.3%                   │  │
│ │ Position: (1.2, 0.8)           │  │
│ │ Timestamp: 14:32:15            │  │
│ └───────────────────────────────────┘  │
│                                     │
│ ┌─── Movement Controls ───────────┐  │
│ │   [Q] ⤺    [E] ⤻               │  │
│ │       [W] ▲                    │  │
│ │   [A]◄─┼─►[D]                  │  │
│ │       [S] ▼                    │  │
│ │                                │  │
│ │ [Forward]  [Backward]          │  │
│ │ [Left]     [Right]             │  │
│ │ [Turn L]   [Turn R]            │  │
│ │                                │  │
│ │ [START]    [STOP]              │  │
│ │ [RESET]    [E-STOP]            │  │
│ └───────────────────────────────────┘  │
│                                     │
│ ┌─── System Actions ─────────────┐  │
│ │ [Calibrate]  [Self Test]       │  │
│ │ [Save Config] [Load Config]    │  │
│ └───────────────────────────────────┘  │
│                                     │
│ ┌─── Window Management ──────────┐  │
│ │ [Show All Windows]             │  │
│ │ [Close All]                    │  │
│ └───────────────────────────────────┘  │
└─────────────────────────────────────┘
```

### 2. 3D Visualization Window (700x700)

```
┌─────────────────────────────────────────────────────────────────┐
│ 🤖 3D ROBOT VISUALIZATION                                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│              ╱╲                                                 │
│            ╱    ╲                                               │
│          ╱   🤖   ╲                                             │
│        ╱           ╲                                            │
│      ╱               ╲                                          │
│      L1  L2  L3  R3  R2  R1                                     │
│                                                                 │
│                                                                 │
│  ┌─ 3D Controls ─────────────────────────────────────────────┐  │
│  │ Camera:                                                  │  │
│  │   Rotate: Left Click + Drag                            │  │
│  │   Pan: Right Click + Drag                              │  │
│  │   Zoom: Scroll Wheel                                   │  │
│  │                                                         │  │
│  │ View Options:                                           │  │
│  │   [Grid] [Shadows] [Axes] [Trails]                     │  │
│  │                                                         │  │
│  │ Azimuth: 45°  Elevation: 30°  Distance: 2.5m          │  │
│  └─────────────────────────────────────────────────────────┘  │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 3. Gait Window (700x350)

```
┌─────────────────────────────────────────────────────────────────┐
│ 🦵 GAIT PATTERNS                                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│ ┌─── Gait Visualization ────────────────────────────────────┐   │
│ │                                                           │   │
│ │ Current: [TRIPOD ▼] Speed: 1.2 m/s  Duty: 50%           │   │
│ │                                                           │   │
│ │ Time → 0%    25%   50%   75%   100%                      │   │
│ │      ┌─────┬─────┬─────┬─────┬─────┐                     │   │
│ │  L1  │ ███ │     │ ███ │     │ ███ │ ◄── Green          │   │
│ │  R1  │     │ ███ │     │ ███ │     │ ◄── Cyan           │   │
│ │  L2  │     │ ███ │     │ ███ │     │ ◄── Magenta        │   │
│ │  R2  │ ███ │     │ ███ │     │ ███ │ ◄── Yellow         │   │
│ │  L3  │ ███ │     │ ███ │     │ ███ │ ◄── Orange         │   │
│ │  R3  │     │ ███ │     │ ███ │     │ ◄── Pink           │   │
│ │      └─────┴─────┴─────┴─────┴─────┘                     │   │
│ │                                                           │   │
│ │ ███ = Swing (air)    ░░░ = Stance (ground)               │   │
│ └───────────────────────────────────────────────────────────┘   │
│                                                                 │
│ ┌─── Gait Controls ─────────────────────────────────────────┐   │
│ │ [Tripod] [Wave] [Ripple] [Custom]                        │   │
│ │ Speed: [━━━━━━━━━━] 1.2 m/s                               │   │
│ │ Step Height: [━━━━━━━━] 8.0 cm                            │   │
│ └───────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### 4. Data Window (350x350)

```
┌─────────────────────────────────────┐
│ 📊 ROBOT DATA                       │
├─────────────────────────────────────┤
│ ┌─── Joint Angles (°) ───────────┐  │
│ │                                │  │
│ │ LEFT LEGS    │ RIGHT LEGS      │  │
│ │ ────────────────────────────────│  │
│ │ L1:          │ R1:             │  │
│ │ C: 45°       │ C: -45°         │  │
│ │ F: -12°      │ F: -12°         │  │
│ │ T: 30°       │ T: 30°          │  │
│ │              │                 │  │
│ │ L2:          │ R2:             │  │
│ │ C: 90°       │ C: -90°         │  │
│ │ F: 15°       │ F: 15°          │  │
│ │ T: -45°      │ T: -45°         │  │
│ │              │                 │  │
│ │ L3:          │ R3:             │  │
│ │ C: 135°      │ C: -135°        │  │
│ │ F: 8°        │ F: 8°           │  │
│ │ T: -20°      │ T: -20°         │  │
│ └────────────────────────────────┘  │
│                                     │
│ ┌─── System Metrics ─────────────┐  │
│ │ CPU: 12.3%  Memory: 45.8%      │  │
│ │ Update Rate: 50 Hz             │  │
│ │ Frame Rate: 60 FPS             │  │
│ │ Uptime: 00:12:34               │  │
│ └───────────────────────────────────┘  │
└─────────────────────────────────────┘
```

### Multi-Window Features

**🔄 Inter-Window Communication:**
- All windows share the same robot state
- Real-time synchronization across all displays
- Commands from control window affect all visualizations

**🪟 Window Management:**
- Independent positioning and sizing
- Each window can be minimized/maximized individually
- "Show All Windows" button restores all minimized windows
- "Close All" safely shuts down entire application

**🎯 Screen Optimization:**
- Total layout fits perfectly in 720x720 screen space
- Window positioning calculated for optimal viewing
- No overlapping when properly arranged
- Minimal title bar space usage

**⚡ Performance:**
- Optimized update rates per window type
- 3D visualization: 30 FPS
- Control updates: 10 Hz
- Data displays: 5 Hz
- Gait visualization: 20 Hz