#!/usr/bin/env python3
"""
HexaPodSim 2.0 - Main Application Entry Point

This is the main entry point for the HexaPodSim 2.0 hexapod robot simulation.
Features a modern neon-themed GUI with real-time gait visualization, 3D robot display,
and comprehensive control systems.

Supports both single-window mode (1400x900) and multi-window mode (4 windows for 720x720 displays).

Author: HexaPodSim Development Team
Version: 2.0
License: MIT
"""

import sys
import os
import logging
import traceback
import argparse
from pathlib import Path

# Add the hexapod module to Python path
sys.path.insert(0, str(Path(__file__).parent / "hexapod"))

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('hexapodsim.log')
    ]
)

logger = logging.getLogger(__name__)

def check_dependencies():
    """Check if all required dependencies are available"""
    required_modules = [
        'tkinter',
        'matplotlib',
        'numpy'
    ]
    
    missing = []
    for module in required_modules:
        try:
            __import__(module)
            logger.info(f"✓ {module} available")
        except ImportError:
            missing.append(module)
            logger.error(f"✗ {module} not available")
    
    if missing:
        print(f"\n❌ Missing required dependencies: {', '.join(missing)}")
        print("\nTo install missing dependencies:")
        print("  pip install matplotlib numpy")
        print("  tkinter is usually included with Python")
        return False
    
    return True

def main():
    """Main application entry point"""
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='HexaPodSim 2.0 - Hexapod Robot Simulation')
    parser.add_argument('--multi-window', action='store_true', 
                       help='Use multi-window mode (4 windows for 720x720 displays)')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug logging')
    
    args = parser.parse_args()
    
    # Configure logging level
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    print("🤖 HexaPodSim 2.0 - Hexapod Robot Simulation")
    print("=" * 50)
    
    if args.multi_window:
        print("🪟 Multi-Window Mode (4 windows for 720x720 displays)")
    else:
        print("🖥️  Single-Window Mode (1400x900 display)")
    
    # Check dependencies
    print("Checking dependencies...")
    if not check_dependencies():
        return 1
    
    try:
        print("\n✓ All dependencies satisfied")
        
        if args.multi_window:
            # Import and start multi-window GUI
            from hexapod.gui_multi import HexaPodSimMultiGUI
            print("🚀 Starting HexaPodSim 2.0 Multi-Window GUI...")
            app = HexaPodSimMultiGUI()
        else:
            # Import and start single-window GUI
            from hexapod.gui import HexaPodSimGUI
            print("🚀 Starting HexaPodSim 2.0 Single-Window GUI...")
            app = HexaPodSimGUI()
        
        app.run()
        
        logger.info("Application exited normally")
        return 0
        
    except ImportError as e:
        logger.error(f"Failed to import required modules: {e}")
        print(f"\n❌ Import Error: {e}")
        print("\nPlease ensure all hexapod modules are properly installed.")
        return 1
        
    except Exception as e:
        logger.error(f"Application error: {e}")
        print(f"\n💥 Application Error: {e}")
        print("\nTraceback:")
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)