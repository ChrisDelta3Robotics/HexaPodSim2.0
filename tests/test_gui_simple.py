#!/usr/bin/env python3
"""
Simple GUI Test - Minimal version to check if GUI displays
"""

import tkinter as tk
from tkinter import ttk
import sys
import os
from pathlib import Path

# Add the hexapod module to Python path
sys.path.insert(0, str(Path(__file__).parent / "hexapod"))

def test_simple_gui():
    """Test basic GUI functionality"""
    print("Creating simple test GUI...")
    
    root = tk.Tk()
    root.title("HexaPodSim 2.0 - Simple Test")
    root.geometry("800x600")
    root.configure(bg='#1a1a1a')
    
    # Add some basic content
    main_frame = tk.Frame(root, bg='#1a1a1a')
    main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    title_label = tk.Label(
        main_frame,
        text="🤖 HexaPodSim 2.0 - Test GUI",
        font=('Consolas', 16, 'bold'),
        fg='#00ff00',
        bg='#1a1a1a'
    )
    title_label.pack(pady=20)
    
    status_label = tk.Label(
        main_frame,
        text="✓ GUI is working!\n✓ Display is functional\n✓ Ready to load full simulation",
        font=('Consolas', 12),
        fg='#00ffff',
        bg='#1a1a1a',
        justify=tk.LEFT
    )
    status_label.pack(pady=20)
    
    # Add a button to test interactivity
    def test_button():
        status_label.config(text="✓ Button clicked!\n✓ GUI is interactive\n✓ Ready for robot simulation")
    
    test_btn = tk.Button(
        main_frame,
        text="Test Button",
        command=test_button,
        font=('Consolas', 12),
        bg='#333333',
        fg='#00ff00',
        activebackground='#555555',
        activeforeground='#00ff00'
    )
    test_btn.pack(pady=10)
    
    # Add quit button
    quit_btn = tk.Button(
        main_frame,
        text="Quit Test",
        command=root.quit,
        font=('Consolas', 12),
        bg='#ff3333',
        fg='white',
        activebackground='#ff5555',
        activeforeground='white'
    )
    quit_btn.pack(pady=10)
    
    print("Starting GUI mainloop...")
    try:
        root.mainloop()
        print("GUI closed normally")
    except Exception as e:
        print(f"GUI error: {e}")
    finally:
        root.destroy()

def test_full_gui():
    """Test the full HexaPodSim GUI"""
    print("Testing full HexaPodSim GUI...")
    
    try:
        from hexapod.gui import HexaPodSimGUI
        
        print("Creating HexaPodSim GUI...")
        app = HexaPodSimGUI()
        
        print("Starting GUI...")
        app.run()
        
    except Exception as e:
        print(f"Full GUI error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Test HexaPodSim GUI')
    parser.add_argument('--simple', action='store_true', help='Run simple GUI test')
    parser.add_argument('--full', action='store_true', help='Run full GUI test')
    
    args = parser.parse_args()
    
    if args.simple:
        test_simple_gui()
    elif args.full:
        test_full_gui()
    else:
        print("Testing simple GUI first...")
        test_simple_gui()
        
        print("\nNow testing full GUI...")
        test_full_gui()