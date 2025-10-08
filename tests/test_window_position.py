#!/usr/bin/env python3
"""
Window Position Test - Force GUI to appear at specific screen coordinates
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

def create_positioned_gui():
    """Create GUI at specific screen position"""
    
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
    
    print("Creating GUI with FORCED SCREEN POSITION...")
    
    root = tk.Tk()
    root.title("🤖 SCREEN POSITION TEST - Should be visible!")
    
    # Force window to appear at top-left corner of screen
    root.geometry("800x600+100+100")  # width x height + x_offset + y_offset
    root.configure(bg='red')  # RED background for maximum visibility
    
    # Make window stay on top and focused
    root.lift()
    root.attributes('-topmost', True)
    root.focus_force()
    
    print("Window positioned at coordinates (100, 100) with red background")
    print("Window should be 800x600 pixels")
    
    # Create a VERY OBVIOUS test interface
    main_frame = tk.Frame(root, bg='yellow', relief=tk.RAISED, bd=10)  # YELLOW with thick border
    main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
    
    # HUGE title
    title_label = tk.Label(
        main_frame,
        text="WINDOW POSITION TEST",
        font=('Arial', 24, 'bold'),
        fg='black',
        bg='yellow'
    )
    title_label.pack(pady=20)
    
    # Instructions
    instructions = tk.Label(
        main_frame,
        text="If you can see this text, the window positioning works!\\n\\n" +
             "This window should be:\\n" +
             "• 800x600 pixels\\n" +
             "• Located at screen position (100, 100)\\n" +
             "• Red outer background\\n" +
             "• Yellow inner frame\\n" +
             "• Always on top\\n\\n" +
             "If you see this, the main GUI windows are probably\\n" +
             "opening outside your visible screen area.",
        font=('Arial', 14),
        fg='black',
        bg='yellow',
        justify=tk.LEFT
    )
    instructions.pack(pady=20)
    
    # Test 3D plot in this positioned window
    viz_frame = tk.Frame(main_frame, bg='blue', relief=tk.RAISED, bd=5)
    viz_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    viz_title = tk.Label(viz_frame, text="3D TEST IN POSITIONED WINDOW", 
                        bg='blue', fg='white', font=("Arial", 12, "bold"))
    viz_title.pack(pady=5)
    
    try:
        # Create 3D plot
        fig = Figure(figsize=(6, 4), facecolor='white')
        ax = fig.add_subplot(111, projection='3d')
        
        # Simple 3D test
        angles = np.linspace(0, 2*np.pi, 7)
        x = 0.1 * np.cos(angles)
        y = 0.1 * np.sin(angles)
        z = np.zeros_like(x)
        
        ax.plot(x, y, z, color='red', linewidth=3)
        ax.set_title('3D Test Plot', color='black')
        
        # Embed in positioned window
        canvas = FigureCanvasTkAgg(fig, viz_frame)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        canvas.draw()
        
        print("✓ 3D plot created in positioned window")
        
    except Exception as e:
        print(f"✗ 3D plot error: {e}")
        error_label = tk.Label(viz_frame, text=f"3D Error: {e}", 
                              bg='blue', fg='white')
        error_label.pack(expand=True)
    
    # Close button
    close_button = tk.Button(
        main_frame,
        text="CLOSE TEST",
        font=('Arial', 16, 'bold'),
        bg='red',
        fg='white',
        command=root.destroy
    )
    close_button.pack(pady=10)
    
    # Get screen dimensions
    def show_screen_info():
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        window_x = root.winfo_x()
        window_y = root.winfo_y()
        window_width = root.winfo_width()
        window_height = root.winfo_height()
        
        print(f"\\n=== SCREEN INFORMATION ===")
        print(f"Screen size: {screen_width}x{screen_height}")
        print(f"Window position: ({window_x}, {window_y})")
        print(f"Window size: {window_width}x{window_height}")
        
        if window_x < 0 or window_y < 0:
            print("⚠️  WARNING: Window has negative coordinates!")
        if window_x > screen_width or window_y > screen_height:
            print("⚠️  WARNING: Window is outside screen bounds!")
        if window_x + window_width > screen_width:
            print("⚠️  WARNING: Window extends beyond right edge!")
        if window_y + window_height > screen_height:
            print("⚠️  WARNING: Window extends beyond bottom edge!")
    
    # Check positioning after window is rendered
    root.after(1000, show_screen_info)
    
    print("\\n" + "="*50)
    print("WINDOW POSITION TEST")
    print("="*50)
    print("\\nThis window should be VERY OBVIOUS:")
    print("- Red background")
    print("- Yellow center frame")
    print("- Blue 3D section")
    print("- Large black text")
    print("- Always on top")
    print("- Position: (100, 100)")
    print("\\nIf you CAN'T see this window:")
    print("- All GUI windows are opening off-screen")
    print("- There's a display/window manager issue")
    print("\\nIf you CAN see this window:")
    print("- The main GUI is opening at wrong coordinates")
    
    return root

if __name__ == "__main__":
    root = create_positioned_gui()
    
    try:
        root.mainloop()
        print("\\nPosition test completed")
    except Exception as e:
        print(f"\\nPosition test error: {e}")
        import traceback
        traceback.print_exc()