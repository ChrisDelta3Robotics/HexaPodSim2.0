#!/usr/bin/env python3
"""
Test script for HexaPodSim 2.0 GUI components

This script tests the GUI components individually to ensure they work correctly
before launching the full application.
"""

import sys
import os
import tkinter as tk
from pathlib import Path

# Add hexapod directory to path
hexapod_dir = Path(__file__).parent / "hexapod"
sys.path.insert(0, str(hexapod_dir))

def test_gui_components():
    """Test individual GUI components"""
    print("🧪 Testing HexaPodSim 2.0 GUI Components")
    print("=" * 45)
    
    try:
        # Test basic imports
        print("Testing imports...")
        from gui import Colors, RobotState, GaitVisualizationPanel, Robot3DVisualization
        print("✓ Core GUI imports successful")
        
        # Test tkinter availability
        root = tk.Tk()
        root.withdraw()  # Hide the root window
        print("✓ Tkinter available")
        
        # Test matplotlib
        import matplotlib
        matplotlib.use('TkAgg')  # Use Tkinter backend
        print(f"✓ Matplotlib available (version {matplotlib.__version__})")
        
        # Test numpy
        import numpy as np
        print(f"✓ NumPy available (version {np.__version__})")
        
        root.destroy()
        
        print("\n🎉 All component tests passed!")
        return True
        
    except Exception as e:
        print(f"\n❌ Component test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def run_gui_demo():
    """Run a simple GUI demonstration"""
    print("\n🚀 Starting GUI demonstration...")
    
    try:
        from gui import HexaPodSimGUI
        
        print("Creating main application...")
        app = HexaPodSimGUI()
        
        print("🎮 GUI is now running!")
        print("Use WASD keys for movement, QE for turning")
        print("Close the window to exit")
        
        app.run()
        
        print("👋 GUI closed successfully")
        
    except Exception as e:
        print(f"❌ GUI demo failed: {e}")
        import traceback
        traceback.print_exc()

def main():
    """Main test function"""
    print("🤖 HexaPodSim 2.0 - GUI Test Suite")
    print("==================================")
    
    # Test components first
    if not test_gui_components():
        print("\n⚠️  Component tests failed. Please check your installation.")
        return 1
    
    # Ask user if they want to run the demo
    try:
        response = input("\nRun GUI demonstration? (y/n): ").lower()
        if response in ['y', 'yes']:
            run_gui_demo()
        else:
            print("Demo skipped. Tests completed successfully!")
    except KeyboardInterrupt:
        print("\n\n👋 Test interrupted by user")
    
    return 0

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)