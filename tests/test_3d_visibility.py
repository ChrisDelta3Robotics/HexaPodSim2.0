#!/usr/bin/env python3
"""
3D Robot Visibility Diagnostic Test
This test checks if the 3D robot is being drawn but not visible due to sizing issues
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

def test_3d_visibility():
    """Test 3D robot visibility with exact main GUI layout"""
    
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
    
    print("Creating exact main GUI layout for 3D diagnostic...")
    
    root = tk.Tk()
    root.title("🤖 3D Robot Visibility Diagnostic")
    root.geometry("1400x900")
    root.configure(bg=Colors.BACKGROUND)
    
    # Main container - EXACT SAME AS MAIN GUI
    main_frame = tk.Frame(root, bg=Colors.BACKGROUND)
    main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    # Title section - EXACT SAME AS MAIN GUI
    title_label = tk.Label(
        main_frame,
        text="🤖 HexaPodSim 2.0 - 3D Diagnostic Test",
        font=('Consolas', 16, 'bold'),
        fg=Colors.ACCENT_1,
        bg=Colors.BACKGROUND
    )
    title_label.pack(pady=(0, 10))
    
    # Control panel section - SAME AS MAIN GUI
    control_frame = tk.LabelFrame(
        main_frame,
        text="Control Panel (Diagnostic)",
        bg=Colors.PANEL_BG,
        fg=Colors.TEXT_PRIMARY,
        font=('Arial', 12, 'bold')
    )
    control_frame.pack(fill=tk.X, pady=(0, 10))
    
    # Add control buttons (simplified)
    button_frame = tk.Frame(control_frame, bg=Colors.PANEL_BG)
    button_frame.pack(pady=10)
    
    buttons = ["Start", "Stop", "Forward", "Backward"]
    for i, btn_text in enumerate(buttons):
        btn = tk.Button(button_frame, text=btn_text, 
                       bg=Colors.BUTTON_NORMAL, fg=Colors.TEXT_PRIMARY,
                       width=12, height=2)
        btn.grid(row=0, column=i, padx=5)
    
    # Secondary panels - EXACT SAME AS MAIN GUI
    panels_frame = tk.Frame(main_frame, bg=Colors.BACKGROUND)
    panels_frame.pack(fill=tk.BOTH, expand=True)
    
    # Left panel - SAME AS MAIN GUI
    left_panel = tk.Frame(panels_frame, bg=Colors.BACKGROUND)
    left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
    
    left_content = tk.LabelFrame(
        left_panel,
        text="Robot Information (Diagnostic)",
        bg=Colors.PANEL_BG,
        fg=Colors.TEXT_PRIMARY,
        font=('Arial', 10, 'bold')
    )
    left_content.pack(fill=tk.BOTH, expand=True)
    
    # Add some content to left panel
    info_text = tk.Text(left_content, bg=Colors.BACKGROUND, fg=Colors.TEXT_PRIMARY,
                       height=15, font=('Consolas', 9))
    info_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
    info_text.insert(tk.END, "DIAGNOSTIC TEST\\n")
    info_text.insert(tk.END, "=" * 30 + "\\n\\n")
    info_text.insert(tk.END, "Testing 3D robot visibility...\\n\\n")
    info_text.insert(tk.END, "Expected 3D robot in right panel:\\n")
    info_text.insert(tk.END, "- Hexagonal white body\\n")
    info_text.insert(tk.END, "- 6 colored legs\\n")
    info_text.insert(tk.END, "- Ground grid\\n")
    info_text.insert(tk.END, "- 3D axes\\n\\n")
    info_text.insert(tk.END, "If you can't see it, there may be:\\n")
    info_text.insert(tk.END, "- Panel sizing issue\\n")
    info_text.insert(tk.END, "- Figure size problem\\n")
    info_text.insert(tk.END, "- Canvas embedding issue\\n")
    
    # Right panel - EXACT SAME AS MAIN GUI
    right_panel = tk.Frame(panels_frame, bg=Colors.BACKGROUND)
    right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
    
    print("Right panel created with expand=True")
    
    # 3D Visualization - EXACT SAME AS WORKING VERSION
    viz_frame = tk.Frame(right_panel, bg=Colors.PANEL_BG, relief=tk.RAISED, bd=2)
    viz_frame.pack(fill=tk.BOTH, expand=True)
    
    # Title
    title_3d = tk.Label(viz_frame, text="3D Robot View (Diagnostic)", 
                       bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY,
                       font=("Arial", 12, "bold"))
    title_3d.pack(pady=5)
    
    print("Creating 3D matplotlib visualization...")
    
    try:
        # Create matplotlib figure for 3D plot - EXACT SAME SIZE AS FIXED VERSION
        fig = Figure(figsize=(6, 7), facecolor=Colors.BACKGROUND)
        ax = fig.add_subplot(111, projection='3d', facecolor=Colors.BACKGROUND)
        
        print("✓ 3D matplotlib figure created")
        
        # Configure 3D plot
        ax.set_xlim(-0.3, 0.3)
        ax.set_ylim(-0.3, 0.3)
        ax.set_zlim(-0.3, 0.1)
        ax.set_xlabel('X (m)', color=Colors.TEXT_PRIMARY)
        ax.set_ylabel('Y (m)', color=Colors.TEXT_PRIMARY)
        ax.set_zlabel('Z (m)', color=Colors.TEXT_PRIMARY)
        ax.tick_params(colors=Colors.TEXT_PRIMARY)
        ax.set_facecolor(Colors.BACKGROUND)
        
        # Draw simple robot body (hexagon) - EXACT SAME AS WORKING TEST
        angles = np.linspace(0, 2*np.pi, 7)
        body_radius = 0.1
        x_body = body_radius * np.cos(angles)
        y_body = body_radius * np.sin(angles)
        z_body = np.zeros_like(x_body)
        
        ax.plot(x_body, y_body, z_body, color='white', linewidth=3, label='Robot Body')
        
        # Draw robot legs (6 legs in star formation) - EXACT SAME AS WORKING TEST
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
        
        # Ground grid
        grid_size = 0.3
        grid_spacing = 0.05
        x_grid = np.arange(-grid_size, grid_size + grid_spacing, grid_spacing)
        y_grid = np.arange(-grid_size, grid_size + grid_spacing, grid_spacing)
        
        for x in x_grid:
            ax.plot([x, x], [-grid_size, grid_size], [-0.2, -0.2], color='gray', alpha=0.3)
        for y in y_grid:
            ax.plot([-grid_size, grid_size], [y, y], [-0.2, -0.2], color='gray', alpha=0.3)
        
        # Set title
        ax.set_title('Hexapod Robot 3D View', color='white')
        
        print("✓ 3D robot model created")
        
        # Embed matplotlib figure in tkinter - EXACT SAME AS FIXED VERSION
        canvas = FigureCanvasTkAgg(fig, viz_frame)
        canvas_widget = canvas.get_tk_widget()
        canvas_widget.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        print("✓ 3D plot embedded in tkinter")
        
        # Force canvas update
        canvas.draw()
        
        print("✓ Canvas draw completed")
        
        # Add debug info to show canvas size
        def show_canvas_info():
            try:
                widget_width = canvas_widget.winfo_width()
                widget_height = canvas_widget.winfo_height()
                panel_width = viz_frame.winfo_width()
                panel_height = viz_frame.winfo_height()
                
                print(f"Canvas widget size: {widget_width}x{widget_height}")
                print(f"Viz frame size: {panel_width}x{panel_height}")
                
                if widget_width < 50 or widget_height < 50:
                    print("⚠️  WARNING: Canvas widget is very small!")
                if panel_width < 100 or panel_height < 100:
                    print("⚠️  WARNING: Visualization frame is very small!")
                    
            except Exception as e:
                print(f"Could not get size info: {e}")
        
        # Check sizes after a short delay
        root.after(1000, show_canvas_info)
        
    except Exception as e:
        print(f"✗ Error creating 3D visualization: {e}")
        import traceback
        traceback.print_exc()
        
        # Create fallback display
        fallback_label = tk.Label(
            viz_frame,
            text=f"3D Visualization Error:\\n{str(e)}\\n\\nCheck matplotlib 3D support",
            bg=Colors.PANEL_BG,
            fg='red',
            font=('Arial', 10),
            justify=tk.LEFT
        )
        fallback_label.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    # Status
    status_frame = tk.Frame(root, bg=Colors.PANEL_BG, height=30)
    status_frame.pack(fill=tk.X, side=tk.BOTTOM)
    
    status_label = tk.Label(
        status_frame,
        text="DIAGNOSTIC: Testing exact main GUI layout for 3D visibility issues",
        bg=Colors.PANEL_BG,
        fg=Colors.TEXT_PRIMARY,
        font=('Arial', 9)
    )
    status_label.pack(pady=5)
    
    print("\\n" + "="*60)
    print("3D ROBOT VISIBILITY DIAGNOSTIC TEST")
    print("="*60)
    print("\\nThis window uses the EXACT same layout as main GUI")
    print("\\nIf you can see the 3D robot here but not in main GUI:")
    print("- There's a specific issue with main GUI implementation")
    print("\\nIf you can't see the 3D robot here either:")
    print("- There's a fundamental layout or sizing issue")
    print("- Check console for canvas size warnings")
    print("\\nExpected in right panel:")
    print("- Hexagonal white robot body")
    print("- 6 colored legs extending outward")
    print("- Ground grid")
    print("- 3D coordinate axes")
    
    return root

if __name__ == "__main__":
    root = test_3d_visibility()
    
    try:
        root.mainloop()
        print("\\n3D visibility diagnostic completed")
    except Exception as e:
        print(f"\\nDiagnostic test error: {e}")
        import traceback
        traceback.print_exc()