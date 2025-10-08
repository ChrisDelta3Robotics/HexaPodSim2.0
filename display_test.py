#!/usr/bin/env python3
"""
Display Diagnostic Tool
Check if GUI display is working properly
"""

import tkinter as tk
import os
import subprocess

def check_display():
    """Check display environment"""
    print("🔍 Display Environment Check")
    print("=" * 40)
    
    # Check DISPLAY variable
    display = os.environ.get('DISPLAY', 'Not set')
    print(f"DISPLAY environment: {display}")
    
    # Check if we're in SSH
    ssh_client = os.environ.get('SSH_CLIENT', None)
    ssh_connection = os.environ.get('SSH_CONNECTION', None)
    
    if ssh_client or ssh_connection:
        print("⚠️  SSH session detected")
        print("   You may need X11 forwarding: ssh -X username@host")
    else:
        print("✓ Local session detected")
    
    # Check if X server is running
    try:
        result = subprocess.run(['xset', 'q'], capture_output=True, text=True)
        if result.returncode == 0:
            print("✓ X server is running")
        else:
            print("❌ X server not accessible")
    except FileNotFoundError:
        print("❌ xset command not found")
    
    # Try to create a simple window
    print("\n🖥️  Testing GUI Window Creation")
    print("=" * 40)
    
    try:
        root = tk.Tk()
        root.title("Display Test")
        root.geometry("400x300")
        root.configure(bg='lightblue')
        
        label = tk.Label(root, text="✓ GUI Display Working!\n\nThis window proves that:\n• tkinter is functional\n• Display is available\n• GUI windows can be created", 
                        font=('Arial', 12), bg='lightblue', justify=tk.CENTER)
        label.pack(expand=True)
        
        def close_test():
            print("✓ GUI test window closed")
            root.quit()
        
        button = tk.Button(root, text="Close Test", command=close_test, 
                          font=('Arial', 10), bg='white')
        button.pack(pady=10)
        
        print("✓ Test window created successfully")
        print("📍 A test window should now be visible")
        print("   If you can see it, your display is working!")
        
        # Show window for 10 seconds or until closed
        root.after(10000, close_test)  # Auto-close after 10 seconds
        root.mainloop()
        
        print("✓ GUI test completed successfully")
        return True
        
    except Exception as e:
        print(f"❌ GUI test failed: {e}")
        return False

def check_hexapod_gui():
    """Check if HexaPodSim GUI components work"""
    print("\n🤖 HexaPodSim GUI Component Test")
    print("=" * 40)
    
    try:
        # Test matplotlib backend
        import matplotlib
        matplotlib.use('TkAgg')
        print(f"✓ Matplotlib backend: {matplotlib.get_backend()}")
        
        # Test imports
        import sys
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).parent / "hexapod"))
        
        from hexapod.gui import Colors, RobotState
        print("✓ HexaPodSim GUI modules import successfully")
        
        # Test basic GUI creation
        root = tk.Tk()
        root.title("HexaPodSim Component Test")
        root.configure(bg=Colors.BACKGROUND)
        root.geometry("600x400")
        
        frame = tk.Frame(root, bg=Colors.BACKGROUND)
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        title = tk.Label(frame, text="🤖 HexaPodSim GUI Components", 
                        font=('Consolas', 16, 'bold'), 
                        fg=Colors.ACCENT_1, bg=Colors.BACKGROUND)
        title.pack(pady=20)
        
        status = tk.Label(frame, text="✓ GUI framework functional\n✓ Colors loaded\n✓ RobotState available\n✓ Ready for full simulation", 
                         font=('Consolas', 12), fg=Colors.TEXT_PRIMARY, bg=Colors.BACKGROUND, justify=tk.LEFT)
        status.pack(pady=20)
        
        def close_component_test():
            print("✓ Component test completed")
            root.quit()
        
        close_btn = tk.Button(frame, text="Close Component Test", command=close_component_test,
                             font=('Consolas', 10), bg=Colors.BUTTON_NORMAL, fg=Colors.TEXT_PRIMARY)
        close_btn.pack(pady=10)
        
        print("✓ HexaPodSim GUI components working")
        print("📍 Component test window should be visible")
        
        root.after(15000, close_component_test)  # Auto-close after 15 seconds
        root.mainloop()
        
        return True
        
    except Exception as e:
        print(f"❌ HexaPodSim GUI component test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("🔧 HexaPodSim Display Diagnostic Tool")
    print("=" * 50)
    
    # Run display check
    display_ok = check_display()
    
    if display_ok:
        # Run component check
        component_ok = check_hexapod_gui()
        
        if component_ok:
            print("\n🎉 All Tests Passed!")
            print("=" * 30)
            print("✓ Display environment working")
            print("✓ GUI framework functional")
            print("✓ HexaPodSim components ready")
            print("\n🚀 Your HexaPodSim GUI should work!")
        else:
            print("\n⚠️  Component test failed")
            print("Display works but HexaPodSim components have issues")
    else:
        print("\n❌ Display test failed")
        print("GUI cannot be displayed in this environment")
        print("\nPossible solutions:")
        print("• If using SSH: connect with 'ssh -X username@host'")
        print("• If local: check X server and window manager")
        print("• Try running on a system with desktop environment")