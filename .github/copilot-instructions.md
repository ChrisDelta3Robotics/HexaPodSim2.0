---
applyTo: "**/*.py"
---
# Project coding standards for Python
- Follow the PEP 8 style guide for Python.
- Always prioritize readability and clarity.
- Write clear and concise comments for each function.
- Ensure functions have descriptive names and include type hints.
- Maintain proper indentation (use 4 spaces for each level of indentation).
- Use docstrings to describe the purpose and usage of classes and methods.
- Limit lines to a maximum of 79 characters.
- Use meaningful variable names that convey intent.
- Avoid using single letter variable names (e.g., "x" and "y") except for loop indices.
- Group related functions into modules and packages.
- Include error handling to manage exceptions gracefully.
- Avoid complexity; break down large functions into smaller, manageable pieces.
## Robot Configuration Standards

### Leg Positioning
- **Star-shaped configuration**: Front and back legs angled for improved stability
- `INWARD_LEG_ANGLE = 15` degrees (configurable variable)
- Front legs (L1, R1): Base angle = 90° + 15° = 105° from body centerline
- Middle legs (L2, R2): Base angle = 90° (perpendicular to body)
- Back legs (L3, R3): Base angle = 90° - 15° = 75° from body centerline
- Apply as body-frame rotation before Denavit-Hartenberg calculations

### Joint Angle Conventions
- **ALL angles in degrees** (never radians)
- Joint limits: -90° to +90° for all rotational joints
- Use consistent naming: `coxa_angle`, `femur_angle`, `tibia_angle`
- Positive rotations follow right-hand rule around joint axis
- Rotational joints start at -90 degrees at the rightmost position, 0 degrees at the center position, and +90 degrees at the leftmost position.
- rotational joints have a PID controller to reach the target angle.
- make the gui responsive and user-friendly.
- use black and neon colors for the gui.
