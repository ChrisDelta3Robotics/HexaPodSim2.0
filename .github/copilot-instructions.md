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
- For all rotational joints, use degrees instead of radians. 
- All rotational joints are limited to a range of -90 to +90 degrees.
- Rotational joints start at -90 degrees at the rightmost position, 0 degrees at the center position, and +90 degrees at the leftmost position.
- rotational joints have a PID controller to reach the target angle.