#!/usr/bin/env python3
"""
3D Layout Debug - Force visible 3D plot in main GUI layout
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

def create_forced_visible_3d_gui():
    """Create main GUI with forced visible 3D panel"""
    
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
    
    print("Creating main GUI with FORCED VISIBLE 3D panel...")
    
    root = tk.Tk()
    root.title("🤖 HexaPodSim - FORCED 3D VISIBILITY TEST")
    root.geometry("1400x900")
    root.configure(bg=Colors.BACKGROUND)
    
    # Main container
    main_frame = tk.Frame(root, bg=Colors.BACKGROUND)
    main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    # Title
    title_label = tk.Label(
        main_frame,
        text="🤖 HexaPodSim - FORCED 3D VISIBILITY TEST",
        font=('Consolas', 16, 'bold'),
        fg=Colors.ACCENT_1,
        bg=Colors.BACKGROUND
    )
    title_label.pack(pady=(0, 10))
    
    # Control panel (MINIMAL to save space)
    control_frame = tk.LabelFrame(
        main_frame,
        text="Control Panel",
        bg=Colors.PANEL_BG,
        fg=Colors.TEXT_PRIMARY,
        font=('Arial', 12, 'bold')
    )
    control_frame.pack(fill=tk.X, pady=(0, 10))
    
    # Just a few buttons
    button_frame = tk.Frame(control_frame, bg=Colors.PANEL_BG)
    button_frame.pack(pady=5)
    
    buttons = ["Start", "Stop", "Forward", "Backward"]
    for i, btn_text in enumerate(buttons):
        btn = tk.Button(button_frame, text=btn_text, 
                       bg=Colors.BUTTON_NORMAL, fg=Colors.TEXT_PRIMARY,
                       width=10, height=1)
        btn.grid(row=0, column=i, padx=3)
    
    # Secondary panels with FORCED 3D VISIBILITY
    panels_frame = tk.Frame(main_frame, bg=Colors.BACKGROUND)
    panels_frame.pack(fill=tk.BOTH, expand=True)
    
    # Left panel - SMALLER to give more space to 3D
    left_panel = tk.Frame(panels_frame, bg=Colors.BACKGROUND)
    left_panel.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 5))  # NO expand=True - fixed width
    
    left_content = tk.LabelFrame(
        left_panel,
        text="Robot Info",
        bg=Colors.PANEL_BG,
        fg=Colors.TEXT_PRIMARY,
        font=('Arial', 10, 'bold'),
        width=300  # FIXED WIDTH
    )
    left_content.pack(fill=tk.Y)
    left_content.pack_propagate(False)  # Don't shrink
    
    # Minimal left content
    info_text = tk.Text(left_content, bg=Colors.BACKGROUND, fg=Colors.TEXT_PRIMARY,
                       height=20, width=35, font=('Consolas', 8))
    info_text.pack(padx=5, pady=5)
    info_text.insert(tk.END, "FORCED 3D VISIBILITY TEST\\n")
    info_text.insert(tk.END, "=" * 25 + "\\n\\n")
    info_text.insert(tk.END, "Left panel is now FIXED WIDTH\\n")
    info_text.insert(tk.END, "to give maximum space to 3D\\n\\n")
    info_text.insert(tk.END, "If you STILL can't see 3D robot:\\n")
    info_text.insert(tk.END, "- 3D plot is outside screen\\n")
    info_text.insert(tk.END, "- Canvas embedding issue\\n")
    info_text.insert(tk.END, "- Figure display problem\\n\\n")
    info_text.insert(tk.END, "Expected in right panel:\\n")
    info_text.insert(tk.END, "- LARGE 3D robot display\\n")
    info_text.insert(tk.END, "- Hexagonal body\\n")
    info_text.insert(tk.END, "- 6 colored legs\\n")
    info_text.insert(tk.END, "- Ground grid\\n")
    
    # Right panel - MAXIMUM SPACE for 3D
    right_panel = tk.Frame(panels_frame, bg='red', relief=tk.RAISED, bd=3)  # RED BORDER for debugging
    right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
    
    print("Right panel created with RED BORDER and expand=True")
    
    # 3D Visualization with MAXIMUM visibility
    viz_frame = tk.Frame(right_panel, bg='blue', relief=tk.RAISED, bd=2)  # BLUE BORDER for debugging
    viz_frame.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
    
    # Title with background color for visibility
    title_3d = tk.Label(viz_frame, text="3D ROBOT VIEW - FORCED VISIBLE", 
                       bg='yellow', fg='black',  # YELLOW background for visibility
                       font=("Arial", 12, "bold"))
    title_3d.pack(pady=2)
    
    print("Creating MAXIMUM SIZE 3D matplotlib visualization...")
    
    try:
        # Create matplotlib figure - LARGER SIZE
        fig = Figure(figsize=(8, 9), facecolor=Colors.BACKGROUND)  # BIGGER than before
        ax = fig.add_subplot(111, projection='3d', facecolor=Colors.BACKGROUND)
        
        print("✓ LARGE 3D matplotlib figure created")
        
        # Configure 3D plot
        ax.set_xlim(-0.3, 0.3)
        ax.set_ylim(-0.3, 0.3)
        ax.set_zlim(-0.3, 0.1)
        ax.set_xlabel('X (m)', color=Colors.TEXT_PRIMARY)
        ax.set_ylabel('Y (m)', color=Colors.TEXT_PRIMARY)
        ax.set_zlabel('Z (m)', color=Colors.TEXT_PRIMARY)
        ax.tick_params(colors=Colors.TEXT_PRIMARY)
        ax.set_facecolor(Colors.BACKGROUND)
        
        # Draw ENHANCED robot for maximum visibility
        angles = np.linspace(0, 2*np.pi, 7)
        body_radius = 0.12  # BIGGER body
        x_body = body_radius * np.cos(angles)
        y_body = body_radius * np.sin(angles)
        z_body = np.zeros_like(x_body)
        
        # THICKER, BRIGHTER robot body
        ax.plot(x_body, y_body, z_body, color='white', linewidth=5, label='Robot Body')
        
        # ENHANCED robot legs for maximum visibility
        leg_angles = np.array([0, 60, 120, 180, 240, 300])
        leg_colors = ['red', 'lime', 'blue', 'yellow', 'orange', 'magenta']  # BRIGHTER colors
        
        for i, (angle, color) in enumerate(zip(leg_angles, leg_colors)):
            angle_rad = np.radians(angle)
            
            # Leg base on body
            base_x = 0.10 * np.cos(angle_rad)  # BIGGER attachment
            base_y = 0.10 * np.sin(angle_rad)
            base_z = 0
            
            # Leg extends outward and down - LONGER legs
            foot_x = 0.25 * np.cos(angle_rad)  # LONGER reach
            foot_y = 0.25 * np.sin(angle_rad)
            foot_z = -0.18  # LOWER feet
            
            # Draw THICKER legs
            ax.plot([base_x, foot_x], [base_y, foot_y], [base_z, foot_z], 
                   color=color, linewidth=5, alpha=0.9)  # THICKER
            
            # Draw BIGGER feet
            ax.scatter([foot_x], [foot_y], [foot_z], color=color, s=100)  # BIGGER
        
        # ENHANCED ground grid
        grid_size = 0.3
        grid_spacing = 0.03  # DENSER grid
        x_grid = np.arange(-grid_size, grid_size + grid_spacing, grid_spacing)
        y_grid = np.arange(-grid_size, grid_size + grid_spacing, grid_spacing)
        
        for x in x_grid:
            ax.plot([x, x], [-grid_size, grid_size], [-0.2, -0.2], color='gray', alpha=0.5)
        for y in y_grid:
            ax.plot([-grid_size, grid_size], [y, y], [-0.2, -0.2], color='gray', alpha=0.5)
        
        # BRIGHT title
        ax.set_title('ENHANCED HEXAPOD ROBOT 3D VIEW', color='white', fontsize=14)
        
        print("✓ ENHANCED 3D robot model created")
        
        # Embed with MAXIMUM space
        canvas = FigureCanvasTkAgg(fig, viz_frame)
        canvas_widget = canvas.get_tk_widget()
        canvas_widget.configure(bg='green')  # GREEN background for debugging
        canvas_widget.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
        
        print("✓ 3D plot embedded with GREEN canvas background")
        
        # Force canvas update
        canvas.draw()
        
        print("✓ Canvas draw completed")
        
        # Debug info function
        def show_detailed_debug_info():
            try:
                widget_width = canvas_widget.winfo_width()
                widget_height = canvas_widget.winfo_height()
                panel_width = viz_frame.winfo_width()
                panel_height = viz_frame.winfo_height()
                right_width = right_panel.winfo_width()
                right_height = right_panel.winfo_height()
                
                print(f"\\n=== 3D VISIBILITY DEBUG INFO ===")
                print(f"Canvas widget size: {widget_width}x{widget_height}")
                print(f"Viz frame size: {panel_width}x{panel_height}")
                print(f"Right panel size: {right_width}x{right_height}")
                print(f"Figure size: 8x9 inches")
                
                if widget_width < 100 or widget_height < 100:
                    print("⚠️  WARNING: Canvas widget is TOO SMALL!")
                elif widget_width > 400 and widget_height > 400:
                    print("✅ Canvas widget has GOOD SIZE for 3D display")
                else:
                    print("⚠️  Canvas widget size is marginal")
                    
                # Check if we can see the red and blue borders
                print("\\nLook for colored borders:")
                print("- RED border around right panel")
                print("- BLUE border around 3D frame")
                print("- YELLOW title background")
                print("- GREEN canvas background")
                    
            except Exception as e:
                print(f"Could not get debug info: {e}")
        
        # Check sizes after GUI is fully rendered
        root.after(1500, show_detailed_debug_info)
        
    except Exception as e:
        print(f"✗ Error creating 3D visualization: {e}")
        import traceback
        traceback.print_exc()
        
        # Create OBVIOUS fallback
        fallback_label = tk.Label(
            viz_frame,
            text=f"3D VISUALIZATION FAILED\\n{str(e)}\\nThis should be VERY OBVIOUS",
            bg='red',
            fg='white',
            font=('Arial', 14, 'bold'),
            justify=tk.CENTER
        )
        fallback_label.pack(fill=tk.BOTH, expand=True)
    
    # Status with colored background
    status_frame = tk.Frame(root, bg='purple', height=35)  # PURPLE for visibility
    status_frame.pack(fill=tk.X, side=tk.BOTTOM)
    
    status_label = tk.Label(
        status_frame,
        text="FORCED 3D VISIBILITY TEST - Look for colored borders and 3D robot",
        bg='purple',
        fg='white',
        font=('Arial', 10, 'bold')
    )
    status_label.pack(pady=5)
    
    print("\\n" + "="*70)
    print("FORCED 3D VISIBILITY TEST CREATED")
    print("="*70)
    print("\\nLook for these VISUAL MARKERS:")
    print("1. RED border around right panel")
    print("2. BLUE border around 3D frame")
    print("3. YELLOW title background")
    print("4. GREEN canvas background")
    print("5. PURPLE status bar")
    print("\\nIf you see these colors but NO 3D robot:")
    print("- The 3D plot is definitely not rendering properly")
    print("\\nIf you don't see ANY of these colors:")
    print("- There's a fundamental window/layout issue")
    print("\\nExpected 3D robot features:")
    print("- LARGER hexagonal white body")
    print("- THICKER, BRIGHTER colored legs") 
    print("- DENSER ground grid")
    print("- ENHANCED visibility overall")
    
    return root

if __name__ == "__main__":
    root = create_forced_visible_3d_gui()
    
    try:
        root.mainloop()
        print("\\nForced 3D visibility test completed")
    except Exception as e:
        print(f"\\nForced visibility test error: {e}")
        import traceback
        traceback.print_exc()