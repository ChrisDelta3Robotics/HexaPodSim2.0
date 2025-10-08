#!/usr/bin/env python3
"""
Minimal GUI Test - Simple version to check what displays
"""

import tkinter as tk
from tkinter import ttk
import sys
import os
from pathlib import Path

# Add the hexapod module to Python path
sys.path.insert(0, str(Path(__file__).parent / "hexapod"))

def create_minimal_gui():
    """Create a minimal GUI with just basic elements"""
    print("Creating minimal GUI...")
    
    root = tk.Tk()
    root.title("🤖 HexaPodSim 2.0 - Minimal Test")
    root.geometry("1200x800")
    root.configure(bg='#000000')
    
    # Main frame
    main_frame = tk.Frame(root, bg='#000000')
    main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    # Title
    title_label = tk.Label(
        main_frame,
        text="🤖 HexaPodSim 2.0 - Minimal GUI Test",
        font=('Consolas', 18, 'bold'),
        fg='#00ff00',
        bg='#000000'
    )
    title_label.pack(pady=10)
    
    # Create a frame for buttons
    button_frame = tk.Frame(main_frame, bg='#000000')
    button_frame.pack(pady=20)
    
    # Create some test buttons
    buttons = [
        ("Forward", '#00ff00'),
        ("Backward", '#ff0000'),
        ("Turn Left", '#0000ff'),
        ("Turn Right", '#ffff00'),
        ("Start", '#00ffff'),
        ("Stop", '#ff00ff')
    ]
    
    for i, (text, color) in enumerate(buttons):
        btn = tk.Button(
            button_frame,
            text=text,
            font=('Consolas', 12),
            bg='#333333',
            fg=color,
            activebackground='#555555',
            activeforeground=color,
            width=12,
            height=2,
            command=lambda t=text: button_clicked(t)
        )
        row = i // 3
        col = i % 3
        btn.grid(row=row, column=col, padx=5, pady=5)
    
    # Status area
    status_frame = tk.Frame(main_frame, bg='#111111', relief=tk.RAISED, bd=2)
    status_frame.pack(fill=tk.BOTH, expand=True, pady=10)
    
    status_label = tk.Label(
        status_frame,
        text="Status: Ready\nWaiting for button clicks...",
        font=('Consolas', 10),
        fg='#ffffff',
        bg='#111111',
        justify=tk.LEFT
    )
    status_label.pack(pady=10)
    
    # Create a simple canvas for drawing
    canvas_frame = tk.Frame(main_frame, bg='#000000')
    canvas_frame.pack(fill=tk.BOTH, expand=True, pady=10)
    
    canvas = tk.Canvas(canvas_frame, bg='#222222', height=200)
    canvas.pack(fill=tk.BOTH, expand=True)
    
    # Draw a simple robot shape
    canvas.create_oval(100, 80, 140, 120, fill='#00ff00', outline='#ffffff')  # Body
    canvas.create_line(120, 100, 80, 80, fill='#ff0000', width=3)    # Leg 1
    canvas.create_line(120, 100, 160, 80, fill='#ff0000', width=3)   # Leg 2
    canvas.create_line(120, 100, 80, 120, fill='#ff0000', width=3)   # Leg 3
    canvas.create_line(120, 100, 160, 120, fill='#ff0000', width=3)  # Leg 4
    canvas.create_line(120, 100, 80, 140, fill='#ff0000', width=3)   # Leg 5
    canvas.create_line(120, 100, 160, 140, fill='#ff0000', width=3)  # Leg 6
    
    canvas.create_text(250, 100, text="Hexapod Robot Simulation", 
                      fill='#ffffff', font=('Consolas', 14))
    
    # Global variable to update status
    global status_widget
    status_widget = status_label
    
    return root

def button_clicked(button_name):
    """Handle button clicks"""
    print(f"Button clicked: {button_name}")
    if 'status_widget' in globals():
        status_widget.config(text=f"Status: {button_name} button clicked!\nLast action: {button_name}")

def test_with_hexapod_panels():
    """Test with actual HexaPodSim panels"""
    print("Testing with HexaPodSim panels...")
    
    try:
        from hexapod.gui import Colors, ControlPanel
        
        root = tk.Tk()
        root.title("🤖 HexaPodSim 2.0 - Panel Test")
        root.geometry("1200x800")
        root.configure(bg=Colors.BACKGROUND)
        
        # Main frame
        main_frame = tk.Frame(root, bg=Colors.BACKGROUND)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Title
        title_label = tk.Label(
            main_frame,
            text="🤖 HexaPodSim Panel Test",
            font=('Consolas', 16, 'bold'),
            fg=Colors.ACCENT_1,
            bg=Colors.BACKGROUND
        )
        title_label.pack(pady=10)
        
        # Try to create a control panel
        try:
            control_panel = ControlPanel(main_frame, width=1000, height=200)
            control_panel.pack(fill=tk.X, pady=10)
            print("✓ ControlPanel created successfully")
        except Exception as e:
            print(f"✗ ControlPanel failed: {e}")
            # Create fallback
            fallback_frame = tk.Frame(main_frame, bg=Colors.PANEL_BG, height=100)
            fallback_frame.pack(fill=tk.X, pady=10)
            tk.Label(fallback_frame, text="Control Panel (fallback)", 
                    bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY).pack(pady=20)
        
        # Simple status
        status_label = tk.Label(
            main_frame,
            text="✓ Panel test running\n✓ Colors working\n✓ Basic GUI functional",
            font=('Consolas', 10),
            fg=Colors.TEXT_PRIMARY,
            bg=Colors.BACKGROUND,
            justify=tk.LEFT
        )
        status_label.pack(pady=10)
        
        return root
        
    except Exception as e:
        print(f"Failed to create panel test: {e}")
        return create_minimal_gui()

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Test GUI visibility')
    parser.add_argument('--minimal', action='store_true', help='Run minimal test')
    parser.add_argument('--panels', action='store_true', help='Test with HexaPodSim panels')
    
    args = parser.parse_args()
    
    if args.minimal:
        print("Testing minimal GUI...")
        root = create_minimal_gui()
    elif args.panels:
        print("Testing with HexaPodSim panels...")
        root = test_with_hexapod_panels()
    else:
        print("Testing minimal GUI first...")
        root = create_minimal_gui()
    
    print("GUI created, starting mainloop...")
    print("You should see:")
    print("- Window title: HexaPodSim 2.0")
    print("- Buttons: Forward, Backward, Turn Left, Turn Right, Start, Stop")
    print("- Status area showing button clicks")
    print("- Simple robot drawing")
    print("\nIf you see this window, the GUI is working!")
    
    try:
        root.mainloop()
        print("GUI closed normally")
    except Exception as e:
        print(f"GUI error: {e}")