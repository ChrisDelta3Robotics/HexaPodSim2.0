#!/usr/bin/env python3
"""
GUI Layout Debug - Test to check if 3D visualization appears in main window
"""

import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import sys
import os
from pathlib import Path

# Add the hexapod module to Python path
sys.path.insert(0, str(Path(__file__).parent / "hexapod"))

def test_gui_layout_with_3d():
    """Test the main GUI layout with 3D visualization to see if it shows up"""
    
    try:
        from hexapod.gui import Colors
    except:
        # Fallback colors
        class Colors:
            BACKGROUND = '#000000'
            PANEL_BG = '#111111'
            TEXT_PRIMARY = '#FFFFFF'
            TEXT_SECONDARY = '#CCCCCC'
            BUTTON_NORMAL = '#333333'
            ACCENT_1 = '#00FF00'
    
    print("Creating GUI layout test...")
    
    root = tk.Tk()
    root.title("🤖 HexaPodSim GUI Layout Test - 3D Debug")
    root.geometry("1400x900")
    root.configure(bg=Colors.BACKGROUND)
    
    # Title
    title_label = tk.Label(
        root,
        text="🤖 HexaPodSim GUI Layout Test",
        font=('Consolas', 16, 'bold'),
        fg=Colors.ACCENT_1,
        bg=Colors.BACKGROUND
    )
    title_label.pack(pady=10)
    
    # Main layout frame
    main_frame = tk.Frame(root, bg=Colors.BACKGROUND)
    main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    # Control panel at top
    control_frame = tk.LabelFrame(
        main_frame,
        text="Control Panel",
        bg=Colors.PANEL_BG,
        fg=Colors.TEXT_PRIMARY,
        font=('Arial', 12, 'bold')
    )
    control_frame.pack(fill=tk.X, pady=(0, 10))
    
    # Add some buttons to control panel
    button_frame = tk.Frame(control_frame, bg=Colors.PANEL_BG)
    button_frame.pack(pady=10)
    
    buttons = ["Forward", "Backward", "Left", "Right", "Start", "Stop"]
    for i, btn_text in enumerate(buttons):
        btn = tk.Button(button_frame, text=btn_text, 
                       bg=Colors.BUTTON_NORMAL, fg=Colors.TEXT_PRIMARY,
                       width=10, height=2)
        btn.grid(row=0, column=i, padx=5)
    
    # Secondary panels container
    panels_frame = tk.Frame(main_frame, bg=Colors.BACKGROUND)
    panels_frame.pack(fill=tk.BOTH, expand=True)
    
    # Left panel - Gait and joint info
    left_panel = tk.LabelFrame(
        panels_frame,
        text="Robot Information",
        bg=Colors.PANEL_BG,
        fg=Colors.TEXT_PRIMARY,
        font=('Arial', 10, 'bold')
    )
    left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
    
    # Add some info text
    info_text = tk.Text(left_panel, bg=Colors.BACKGROUND, fg=Colors.TEXT_PRIMARY,
                       height=15, font=('Consolas', 9))
    info_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
    info_text.insert(tk.END, "Robot Status:\\n")
    info_text.insert(tk.END, "=" * 30 + "\\n\\n")
    info_text.insert(tk.END, "✓ Kinematics: Active\\n")
    info_text.insert(tk.END, "✓ Gait Generator: Running\\n")
    info_text.insert(tk.END, "✓ Motion Controller: Ready\\n\\n")
    info_text.insert(tk.END, "Joint Angles:\\n")
    info_text.insert(tk.END, "L1: (0°, -20°, 40°)\\n")
    info_text.insert(tk.END, "R1: (0°, -20°, 40°)\\n")
    info_text.insert(tk.END, "L2: (0°, -20°, 40°)\\n")
    info_text.insert(tk.END, "R2: (0°, -20°, 40°)\\n")
    info_text.insert(tk.END, "L3: (0°, -20°, 40°)\\n")
    info_text.insert(tk.END, "R3: (0°, -20°, 40°)\\n\\n")
    info_text.insert(tk.END, "Gait: TRIPOD\\n")
    info_text.insert(tk.END, "Phase: 45.2%\\n")
    
    # Right panel - 3D visualization
    right_panel = tk.LabelFrame(
        panels_frame,
        text="3D Robot Visualization",
        bg=Colors.PANEL_BG,
        fg=Colors.TEXT_PRIMARY,
        font=('Arial', 10, 'bold')
    )
    right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(5, 0))
    
    # Create 3D visualization
    print("Creating 3D matplotlib plot...")
    
    try:
        # Create matplotlib figure for 3D plot
        fig = Figure(figsize=(6, 8), facecolor=Colors.BACKGROUND)
        ax = fig.add_subplot(111, projection='3d', facecolor=Colors.BACKGROUND)
        
        print("✓ 3D matplotlib figure created")
        
        # Create a simple hexapod robot
        # Robot body (hexagon)
        angles = np.linspace(0, 2*np.pi, 7)
        body_radius = 0.1
        x_body = body_radius * np.cos(angles)
        y_body = body_radius * np.sin(angles)
        z_body = np.zeros_like(x_body)
        
        ax.plot(x_body, y_body, z_body, color='white', linewidth=3, label='Robot Body')
        
        # Robot legs (6 legs in star formation)  
        leg_angles = np.array([0, 60, 120, 180, 240, 300])
        leg_colors = ['red', 'green', 'blue', 'yellow', 'orange', 'purple']
        
        for i, (angle, color) in enumerate(zip(leg_angles, leg_colors)):
            angle_rad = np.radians(angle)
            
            # Leg base on body
            base_x = 0.08 * np.cos(angle_rad)
            base_y = 0.08 * np.sin(angle_rad)
            base_z = 0
            
            # Leg extends outward and down
            foot_x = 0.2 * np.cos(angle_rad)
            foot_y = 0.2 * np.sin(angle_rad)
            foot_z = -0.15
            
            # Draw leg
            ax.plot([base_x, foot_x], [base_y, foot_y], [base_z, foot_z], 
                   color=color, linewidth=3, alpha=0.8)
            
            # Draw foot
            ax.scatter([foot_x], [foot_y], [foot_z], color=color, s=50)
        
        # Configure plot
        ax.set_xlim(-0.3, 0.3)
        ax.set_ylim(-0.3, 0.3)
        ax.set_zlim(-0.2, 0.1)
        ax.set_xlabel('X (m)', color='white')
        ax.set_ylabel('Y (m)', color='white')
        ax.set_zlabel('Z (m)', color='white')
        ax.tick_params(colors='white')
        ax.set_title('Hexapod Robot 3D View', color='white')
        
        # Ground grid
        grid_size = 0.3
        grid_spacing = 0.05
        x_grid = np.arange(-grid_size, grid_size + grid_spacing, grid_spacing)
        y_grid = np.arange(-grid_size, grid_size + grid_spacing, grid_spacing)
        
        for x in x_grid:
            ax.plot([x, x], [-grid_size, grid_size], [-0.2, -0.2], color='gray', alpha=0.3)
        for y in y_grid:
            ax.plot([-grid_size, grid_size], [y, y], [-0.2, -0.2], color='gray', alpha=0.3)
        
        print("✓ 3D robot model created")
        
        # Embed matplotlib figure in tkinter
        canvas = FigureCanvasTkAgg(fig, right_panel)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        print("✓ 3D plot embedded in tkinter")
        
        # Force canvas update
        canvas.draw()
        
        print("✓ Canvas draw completed")
        
    except Exception as e:
        print(f"✗ Error creating 3D visualization: {e}")
        import traceback
        traceback.print_exc()
        
        # Create fallback display
        fallback_label = tk.Label(
            right_panel,
            text=f"3D Visualization Error:\\n{str(e)}\\n\\nCheck matplotlib 3D support",
            bg=Colors.PANEL_BG,
            fg='red',
            font=('Arial', 10),
            justify=tk.LEFT
        )
        fallback_label.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    # Status bar
    status_frame = tk.Frame(root, bg=Colors.PANEL_BG, height=30)
    status_frame.pack(fill=tk.X, side=tk.BOTTOM)
    
    status_label = tk.Label(
        status_frame,
        text="GUI Layout Test - Check if 3D robot is visible in right panel",
        bg=Colors.PANEL_BG,
        fg=Colors.TEXT_PRIMARY,
        font=('Arial', 9)
    )
    status_label.pack(pady=5)
    
    print("\\n" + "="*60)
    print("GUI Layout Test Created Successfully!")
    print("="*60)
    print("\\nYou should see:")
    print("1. Main window (1400x900)")
    print("2. Control buttons at top")
    print("3. Left panel with robot information")
    print("4. RIGHT PANEL with 3D robot visualization")
    print("5. Status bar at bottom")
    print("\\nThe 3D robot should show:")
    print("- Hexagonal body (white)")
    print("- 6 colored legs (red, green, blue, yellow, orange, purple)")
    print("- Ground grid")
    print("- 3D axes")
    print("\\nIf you DON'T see the 3D robot, there may be an issue with:")
    print("- Matplotlib 3D backend")
    print("- Figure embedding in tkinter")
    print("- Canvas drawing")
    
    return root

if __name__ == "__main__":
    root = test_gui_layout_with_3d()
    
    try:
        root.mainloop()
        print("\\nGUI test completed")
    except Exception as e:
        print(f"\\nGUI test error: {e}")
        import traceback
        traceback.print_exc()