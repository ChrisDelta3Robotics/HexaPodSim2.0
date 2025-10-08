#!/usr/bin/env python3
"""
Simplified HexaPodSim GUI - Focuses on displaying buttons and basic functionality
"""

import tkinter as tk
from tkinter import ttk
import sys
import os
from pathlib import Path

# Add the hexapod module to Python path
sys.path.insert(0, str(Path(__file__).parent / "hexapod"))

def create_simple_hexapod_gui():
    """Create a simplified HexaPodSim GUI with visible buttons"""
    print("Creating simplified HexaPodSim GUI...")
    
    try:
        from hexapod.gui import Colors
    except:
        # Fallback colors if import fails
        class Colors:
            BACKGROUND = '#000000'
            PANEL_BG = '#111111'
            TEXT_PRIMARY = '#FFFFFF'
            TEXT_SECONDARY = '#CCCCCC'
            BUTTON_NORMAL = '#333333'
            BUTTON_ACTIVE = '#555555'
            ACCENT_1 = '#00FF00'
    
    root = tk.Tk()
    root.title("🤖 HexaPodSim 2.0 - Simplified Interface")
    root.geometry("1200x800")
    root.configure(bg=Colors.BACKGROUND)
    
    # Main container
    main_frame = tk.Frame(root, bg=Colors.BACKGROUND)
    main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    # Title
    title_label = tk.Label(
        main_frame,
        text="🤖 HexaPodSim 2.0 - Robot Control Interface",
        font=('Consolas', 16, 'bold'),
        fg=Colors.ACCENT_1,
        bg=Colors.BACKGROUND
    )
    title_label.pack(pady=10)
    
    # Control panel frame
    control_frame = tk.LabelFrame(
        main_frame,
        text="Robot Control Panel",
        bg=Colors.PANEL_BG,
        fg=Colors.TEXT_PRIMARY,
        font=('Arial', 12, 'bold'),
        relief=tk.RAISED,
        bd=2
    )
    control_frame.pack(fill=tk.X, pady=10, padx=20)
    
    # Movement controls section
    movement_section = tk.LabelFrame(
        control_frame,
        text="Movement Controls",
        bg=Colors.PANEL_BG,
        fg=Colors.TEXT_PRIMARY,
        font=('Arial', 10, 'bold')
    )
    movement_section.pack(fill=tk.X, padx=10, pady=10)
    
    # Button style
    button_style = {
        'bg': Colors.BUTTON_NORMAL,
        'fg': Colors.TEXT_PRIMARY,
        'activebackground': Colors.BUTTON_ACTIVE,
        'activeforeground': Colors.TEXT_PRIMARY,
        'relief': tk.RAISED,
        'bd': 2,
        'font': ('Arial', 10, 'bold'),
        'width': 12,
        'height': 2
    }
    
    # Create movement buttons in a grid
    movement_buttons = [
        ("Forward", 0, 1),
        ("Backward", 2, 1),
        ("Turn Left", 1, 0),
        ("Turn Right", 1, 2),
        ("Strafe Left", 1, 0),
        ("Strafe Right", 1, 2)
    ]
    
    button_frame = tk.Frame(movement_section, bg=Colors.PANEL_BG)
    button_frame.pack(pady=10)
    
    # WASD-style layout
    forward_btn = tk.Button(button_frame, text="FORWARD (W)", **button_style,
                           command=lambda: button_clicked("Forward"))
    forward_btn.grid(row=0, column=1, padx=5, pady=5)
    
    left_btn = tk.Button(button_frame, text="LEFT (A)", **button_style,
                        command=lambda: button_clicked("Left"))
    left_btn.grid(row=1, column=0, padx=5, pady=5)
    
    backward_btn = tk.Button(button_frame, text="BACKWARD (S)", **button_style,
                            command=lambda: button_clicked("Backward"))
    backward_btn.grid(row=1, column=1, padx=5, pady=5)
    
    right_btn = tk.Button(button_frame, text="RIGHT (D)", **button_style,
                         command=lambda: button_clicked("Right"))
    right_btn.grid(row=1, column=2, padx=5, pady=5)
    
    turn_left_btn = tk.Button(button_frame, text="TURN LEFT (Q)", **button_style,
                             command=lambda: button_clicked("Turn Left"))
    turn_left_btn.grid(row=2, column=0, padx=5, pady=5)
    
    turn_right_btn = tk.Button(button_frame, text="TURN RIGHT (E)", **button_style,
                              command=lambda: button_clicked("Turn Right"))
    turn_right_btn.grid(row=2, column=2, padx=5, pady=5)
    
    # Action controls section
    action_section = tk.LabelFrame(
        control_frame,
        text="Robot Actions",
        bg=Colors.PANEL_BG,
        fg=Colors.TEXT_PRIMARY,
        font=('Arial', 10, 'bold')
    )
    action_section.pack(fill=tk.X, padx=10, pady=10)
    
    action_frame = tk.Frame(action_section, bg=Colors.PANEL_BG)
    action_frame.pack(pady=10)
    
    start_btn = tk.Button(action_frame, text="START", bg='#00ff00', fg='#000000',
                         font=('Arial', 12, 'bold'), width=10, height=2,
                         command=lambda: button_clicked("Start"))
    start_btn.pack(side=tk.LEFT, padx=10)
    
    stop_btn = tk.Button(action_frame, text="STOP", bg='#ff0000', fg='#ffffff',
                        font=('Arial', 12, 'bold'), width=10, height=2,
                        command=lambda: button_clicked("Stop"))
    stop_btn.pack(side=tk.LEFT, padx=10)
    
    reset_btn = tk.Button(action_frame, text="RESET", bg='#ffff00', fg='#000000',
                         font=('Arial', 12, 'bold'), width=10, height=2,
                         command=lambda: button_clicked("Reset"))
    reset_btn.pack(side=tk.LEFT, padx=10)
    
    # Status display
    status_section = tk.LabelFrame(
        main_frame,
        text="Robot Status",
        bg=Colors.PANEL_BG,
        fg=Colors.TEXT_PRIMARY,
        font=('Arial', 10, 'bold')
    )
    status_section.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
    
    # Status text area
    status_text = tk.Text(
        status_section,
        bg=Colors.BACKGROUND,
        fg=Colors.TEXT_PRIMARY,
        font=('Consolas', 10),
        height=10,
        wrap=tk.WORD
    )
    status_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    # Add initial status message
    status_text.insert(tk.END, "🤖 HexaPodSim 2.0 - Simplified Interface\\n")
    status_text.insert(tk.END, "=" * 50 + "\\n")
    status_text.insert(tk.END, "✓ GUI created successfully\\n")
    status_text.insert(tk.END, "✓ Control buttons available\\n")
    status_text.insert(tk.END, "✓ Ready for robot commands\\n\\n")
    status_text.insert(tk.END, "Instructions:\\n")
    status_text.insert(tk.END, "- Use movement buttons to control robot\\n")
    status_text.insert(tk.END, "- Click START to begin simulation\\n")
    status_text.insert(tk.END, "- Click STOP to halt robot\\n")
    status_text.insert(tk.END, "- Click RESET to return to home position\\n\\n")
    status_text.insert(tk.END, "Waiting for commands...\\n")
    
    # Global reference for status updates
    global status_display
    status_display = status_text
    
    return root

def button_clicked(action):
    """Handle button clicks and update status"""
    print(f"Button clicked: {action}")
    
    if 'status_display' in globals():
        status_display.insert(tk.END, f"🎯 Action: {action}\\n")
        status_display.see(tk.END)  # Scroll to bottom
        
        # Add specific feedback for each action
        if action == "Forward":
            status_display.insert(tk.END, "   → Robot moving forward\\n")
        elif action == "Backward":
            status_display.insert(tk.END, "   ← Robot moving backward\\n")
        elif action == "Left":
            status_display.insert(tk.END, "   ← Robot strafing left\\n")
        elif action == "Right":
            status_display.insert(tk.END, "   → Robot strafing right\\n")
        elif action == "Turn Left":
            status_display.insert(tk.END, "   ↺ Robot turning left\\n")
        elif action == "Turn Right":
            status_display.insert(tk.END, "   ↻ Robot turning right\\n")
        elif action == "Start":
            status_display.insert(tk.END, "   ✓ Robot simulation started\\n")
        elif action == "Stop":
            status_display.insert(tk.END, "   ⏹ Robot simulation stopped\\n")
        elif action == "Reset":
            status_display.insert(tk.END, "   🔄 Robot reset to home position\\n")
        
        status_display.insert(tk.END, "\\n")
        status_display.see(tk.END)

if __name__ == "__main__":
    print("🤖 HexaPodSim 2.0 - Simplified GUI")
    print("=" * 40)
    print("Creating a simplified interface to test button visibility...")
    
    root = create_simple_hexapod_gui()
    
    print("\\n✓ GUI created successfully")
    print("You should now see:")
    print("- Large window with dark theme")
    print("- Movement control buttons (WASD layout)")
    print("- Action buttons (Start, Stop, Reset)")
    print("- Status display area")
    print("\\nIf you can see these buttons, the GUI system is working!")
    print("Starting GUI mainloop...")
    
    try:
        root.mainloop()
        print("\\nGUI closed normally")
    except Exception as e:
        print(f"\\nGUI error: {e}")
        import traceback
        traceback.print_exc()