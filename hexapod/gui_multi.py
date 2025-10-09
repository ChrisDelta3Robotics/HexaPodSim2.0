"""
HexaPodSim 2.0 - Multi-Window GUI System

This module provides a multi-window interface optimized for 720x720 displays.
The GUI is split into 4 separate windows:

1. Control Window - Robot movement controls and system status
2. 3D Visualization Window - 3D robot display
3. Gait Window - Gait pattern visualization
4. Data Window - Joint angles and sensor data

Each window is sized to fit comfortably on a 720x720 screen.

Author: HexaPodSim Development Team
Version: 2.0
"""

import tkinter as tk
from tkinter import ttk
import threading
import time
import logging
import numpy as np
from typing import Optional, Dict, Any
import sys
from pathlib import Path

# Import base GUI components
try:
    from .gui import Colors, RobotState, Robot3DVisualization, GaitVisualizationPanel, ControlPanel, JointAnglePanel
    from .motion import MotionController, MotionCommand
    from .kinematics import HexapodKinematics
    from .gait import GaitGenerator
except ImportError:
    # Fallback for direct execution
    sys.path.insert(0, str(Path(__file__).parent))
    from gui import Colors, RobotState, Robot3DVisualization, GaitVisualizationPanel, ControlPanel, JointAnglePanel
    from motion import MotionController, MotionCommand
    from kinematics import HexapodKinematics
    from gait import GaitGenerator

logger = logging.getLogger(__name__)

class ControlWindow:
    """Main control window for robot commands and system status"""
    
    def __init__(self, multi_gui_controller):
        self.controller = multi_gui_controller
        self.root = tk.Tk()
        self.root.title("🤖 HexaPodSim 2.0 - Control Center")
        self.root.geometry("350x450+10+10")
        self.root.configure(bg=Colors.BACKGROUND)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        self._create_interface()
        logger.info("Control window created")
    
    def _create_interface(self):
        """Create the control interface"""
        # Title
        title_label = tk.Label(
            self.root,
            text="🤖 CONTROL CENTER",
            font=('Consolas', 14, 'bold'),
            fg=Colors.ACCENT_1,
            bg=Colors.BACKGROUND
        )
        title_label.pack(pady=5)
        
        # Status frame
        status_frame = tk.LabelFrame(
            self.root,
            text="System Status",
            bg=Colors.PANEL_BG,
            fg=Colors.TEXT_PRIMARY,
            font=('Arial', 9, 'bold')
        )
        status_frame.pack(fill=tk.X, padx=10, pady=(0, 5))
        
        self.status_text = tk.Text(
            status_frame,
            height=4,
            bg=Colors.BACKGROUND,
            fg=Colors.TEXT_PRIMARY,
            font=('Consolas', 8),
            wrap=tk.WORD
        )
        self.status_text.pack(fill=tk.X, padx=5, pady=3)
        
        # Control panel
        try:
            self.control_panel = ControlPanel(self.root, width=330, height=220)
            self.control_panel.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 5))
            
            # Set up callbacks
            print("🔧 Setting up movement callbacks...")
            self.control_panel.movement_callbacks = {
                'forward': lambda: self.controller.handle_movement(0.2, 0.0, 0.0),  # Reduced from 1.0 to 0.2
                'backward': lambda: self.controller.handle_movement(-0.2, 0.0, 0.0),  # Reduced from -1.0 to -0.2
                'left': lambda: self.controller.handle_movement(0.0, 0.2, 0.0),  # Reduced from 1.0 to 0.2
                'right': lambda: self.controller.handle_movement(0.0, -0.2, 0.0),  # Reduced from -1.0 to -0.2
                'turn_left': lambda: self.controller.handle_movement(0.0, 0.0, 15.0),  # Reduced from 30.0 to 15.0
                'turn_right': lambda: self.controller.handle_movement(0.0, 0.0, -15.0),  # Reduced from -30.0 to -15.0
            }
            print(f"✅ Movement callbacks set: {list(self.control_panel.movement_callbacks.keys())}")
            
            self.control_panel.action_callbacks = {
                'start': self.controller.start_simulation,
                'stop': self.controller.stop_simulation,
                'reset': self.controller.reset_robot,
                'emergency_stop': self.controller.emergency_stop,
                'crouch': lambda: print("CROUCH: Not implemented yet"),
                'stand': lambda: print("STAND: Not implemented yet"),
            }
            print(f"✅ Action callbacks set: {list(self.control_panel.action_callbacks.keys())}")
            
            self.control_panel.system_callbacks = {
                'reset': self.controller.reset_robot,
                'settings': lambda: print("SETTINGS: Not implemented yet"),
                'calibrate': self.controller.calibrate_robot,
                'self_test': self.controller.run_self_test,
            }
            print(f"✅ System callbacks set: {list(self.control_panel.system_callbacks.keys())}")
            
        except Exception as e:
            logger.error(f"Failed to create control panel: {e}")
            # Create fallback controls
            self._create_fallback_controls()
        
        # Window management buttons
        window_frame = tk.Frame(self.root, bg=Colors.BACKGROUND)
        window_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Button(
            window_frame,
            text="Show All Windows",
            command=self.controller.show_all_windows,
            bg=Colors.BUTTON_NORMAL,
            fg=Colors.TEXT_PRIMARY,
            font=('Arial', 9)
        ).pack(side=tk.LEFT, padx=(0, 5))
        
        tk.Button(
            window_frame,
            text="Close All",
            command=self.controller.close_all_windows,
            bg=Colors.BUTTON_NORMAL,
            fg=Colors.TEXT_PRIMARY,
            font=('Arial', 9)
        ).pack(side=tk.RIGHT)
        
        # Auto-test button functionality after 3 seconds
        print("🧪 Will test button functionality in 3 seconds...")
        self.root.after(3000, self._test_button_functionality)
        
        # Start periodic button state monitoring
        self.root.after(5000, self._monitor_button_states)
    
    def _test_button_functionality(self):
        """Test button functionality by simulating a button press"""
        print("🧪 Testing button functionality...")
        try:
            # First, try to reset the robot to clear emergency stop
            print("🔄 Attempting to reset robot and clear emergency stop...")
            if hasattr(self.controller, 'reset_robot'):
                self.controller.reset_robot()
                print("✅ Robot reset called")
            
            # Wait a moment for reset to take effect
            self.root.after(1000, self._test_movement_after_reset)
            
        except Exception as e:
            print(f"💥 Error testing button functionality: {e}")
    
    def _test_movement_after_reset(self):
        """Test movement after robot reset"""
        try:
            print("🧪 Testing button functionality...")
            
            # Test START button first
            print("🔥 Testing START button...")
            if hasattr(self, 'controller') and hasattr(self.controller, 'start_simulation'):
                self.controller.start_simulation()
                
            # Wait a moment then test movement
            self.root.after(1000, self._test_movement_button)
                
        except Exception as e:
            print(f"💥 Error testing movement: {e}")
            import traceback
            traceback.print_exc()
    
    def _test_movement_button(self):
        """Test movement button after start"""
        try:
            # Test E button specifically (turn_right)
            print("🔥 Testing E button (turn_right)...")
            if hasattr(self.control_panel, 'e_button'):
                print("✅ Found E button widget")
                button_state = self.control_panel.e_button['state']
                print(f"📊 E button state: {button_state}")
                print("🖱️ Simulating E button click...")
                self.control_panel.e_button.invoke()
            else:
                print("❌ Could not find E button widget")
            
            # Wait a moment then test D button
            self.root.after(1000, self._test_d_button)
                
        except Exception as e:
            print(f"💥 Error testing E button: {e}")
            import traceback
            traceback.print_exc()
    
    def _test_d_button(self):
        """Test D button specifically"""
        try:
            # Test D button specifically (right)
            print("🔥 Testing D button (right)...")
            if hasattr(self.control_panel, 'd_button'):
                print("✅ Found D button widget")
                button_state = self.control_panel.d_button['state']
                print(f"📊 D button state: {button_state}")
                print("🖱️ Simulating D button click...")
                self.control_panel.d_button.invoke()
            else:
                print("❌ Could not find D button widget")
                
            # Also test direct callback calls
            print("🔬 Testing direct callback calls...")
            if hasattr(self.control_panel, '_handle_movement'):
                print("✅ Testing turn_right callback directly...")
                self.control_panel._handle_movement('turn_right')
                print("✅ Testing right callback directly...")
                self.control_panel._handle_movement('right')
            
        except Exception as e:
            print(f"💥 Error testing D button: {e}")
            import traceback
            traceback.print_exc()
    
    def _monitor_button_states(self):
        """Monitor button states periodically"""
        try:
            print("📊 BUTTON STATE MONITOR:")
            if hasattr(self, 'control_panel'):
                panel = self.control_panel
                buttons_to_check = ['w_button', 'a_button', 's_button', 'd_button', 'q_button', 'e_button']
                
                for button_name in buttons_to_check:
                    if hasattr(panel, button_name):
                        button = getattr(panel, button_name)
                        state = button['state']
                        bg_color = button['bg']
                        print(f"   - {button_name}: state='{state}', bg='{bg_color}'")
                        
                        # Check if button appears disabled
                        if state == 'disabled':
                            print(f"   ⚠️ {button_name} is DISABLED!")
                        elif state != 'normal':
                            print(f"   ⚠️ {button_name} has unusual state: {state}")
                    else:
                        print(f"   - {button_name}: NOT FOUND")
                        
                # Check action buttons too
                action_buttons = ['start_button', 'stop_button', 'reset_button']
                for button_name in action_buttons:
                    if hasattr(panel, button_name):
                        button = getattr(panel, button_name)
                        state = button['state']
                        print(f"   - {button_name}: state='{state}'")
                        if state == 'disabled':
                            print(f"   ⚠️ {button_name} is DISABLED!")
            else:
                print("   ❌ No control panel found")
                
            # Schedule next check in 10 seconds
            self.root.after(10000, self._monitor_button_states)
            
        except Exception as e:
            print(f"💥 Error monitoring button states: {e}")
            import traceback
            traceback.print_exc()
    
    def _create_fallback_controls(self):
        """Create simple fallback controls if ControlPanel fails"""
        fallback_frame = tk.LabelFrame(
            self.root,
            text="Basic Controls",
            bg=Colors.PANEL_BG,
            fg=Colors.TEXT_PRIMARY,
            font=('Arial', 10, 'bold')
        )
        fallback_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        
        # Movement buttons
        move_frame = tk.Frame(fallback_frame, bg=Colors.PANEL_BG)
        move_frame.pack(pady=10)
        
        buttons = [
            ("Forward", lambda: self.controller.handle_movement(1.0, 0.0, 0.0)),
            ("Backward", lambda: self.controller.handle_movement(-1.0, 0.0, 0.0)),
            ("Left", lambda: self.controller.handle_movement(0.0, 1.0, 0.0)),
            ("Right", lambda: self.controller.handle_movement(0.0, -1.0, 0.0)),
            ("Start", self.controller.start_simulation),
            ("Stop", self.controller.stop_simulation),
        ]
        
        for i, (text, command) in enumerate(buttons):
            btn = tk.Button(
                move_frame,
                text=text,
                command=command,
                bg=Colors.BUTTON_NORMAL,
                fg=Colors.TEXT_PRIMARY,
                width=10,
                height=2
            )
            btn.grid(row=i//2, column=i%2, padx=5, pady=5)
    
    def update_status(self, status_info: Dict[str, Any]):
        """Update status display"""
        try:
            self.status_text.delete(1.0, tk.END)
            self.status_text.insert(tk.END, "🤖 SYSTEM STATUS\\n")
            self.status_text.insert(tk.END, "=" * 30 + "\\n\\n")
            
            for key, value in status_info.items():
                self.status_text.insert(tk.END, f"{key}: {value}\\n")
                
        except Exception as e:
            logger.error(f"Error updating status: {e}")
    
    def on_close(self):
        """Handle window close event"""
        self.controller.close_all_windows()


class Visualization3DWindow:
    """3D robot visualization window"""
    
    def __init__(self, multi_gui_controller):
        self.controller = multi_gui_controller
        self.root = tk.Tk()
        self.root.title("🤖 HexaPodSim 2.0 - 3D Robot View")
        self.root.geometry("700x700+380+10")
        self.root.configure(bg=Colors.BACKGROUND)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        self._create_interface()
        logger.info("3D visualization window created")
    
    def _create_interface(self):
        """Create the 3D visualization interface"""
        # Title
        title_label = tk.Label(
            self.root,
            text="🤖 3D ROBOT VISUALIZATION",
            font=('Consolas', 14, 'bold'),
            fg=Colors.ACCENT_1,
            bg=Colors.BACKGROUND
        )
        title_label.pack(pady=5)
        
        # 3D visualization
        try:
            self.robot_3d = Robot3DVisualization(self.root, width=680, height=640)
            self.robot_3d.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        except Exception as e:
            logger.error(f"Failed to create 3D visualization: {e}")
            # Create fallback
            fallback_label = tk.Label(
                self.root,
                text="3D Visualization Error\\nCheck matplotlib installation",
                bg=Colors.PANEL_BG,
                fg=Colors.TEXT_PRIMARY,
                font=('Arial', 12),
                justify=tk.CENTER
            )
            fallback_label.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    def update_robot_display(self, robot_state: RobotState):
        """Update 3D robot display"""
        try:
            if hasattr(self, 'robot_3d'):
                self.robot_3d.update_robot_display(robot_state)
        except Exception as e:
            logger.error(f"Error updating 3D display: {e}")
    
    def on_close(self):
        """Handle window close event"""
        self.root.withdraw()


class GaitWindow:
    """Gait pattern visualization window"""
    
    def __init__(self, multi_gui_controller):
        self.controller = multi_gui_controller
        self.root = tk.Tk()
        self.root.title("🤖 HexaPodSim 2.0 - Gait Patterns")
        self.root.geometry("700x350+10+680")
        self.root.configure(bg=Colors.BACKGROUND)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        self._create_interface()
        logger.info("Gait window created")
    
    def _create_interface(self):
        """Create the gait visualization interface"""
        # Title
        title_label = tk.Label(
            self.root,
            text="🦵 GAIT PATTERNS",
            font=('Consolas', 14, 'bold'),
            fg=Colors.ACCENT_1,
            bg=Colors.BACKGROUND
        )
        title_label.pack(pady=5)
        
        # Gait visualization
        try:
            self.gait_panel = GaitVisualizationPanel(self.root, width=680, height=280)
            self.gait_panel.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        except Exception as e:
            logger.error(f"Failed to create gait panel: {e}")
            # Create fallback
            fallback_label = tk.Label(
                self.root,
                text="Gait Visualization Error\\nCheck matplotlib installation",
                bg=Colors.PANEL_BG,
                fg=Colors.TEXT_PRIMARY,
                font=('Arial', 12),
                justify=tk.CENTER
            )
            fallback_label.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    def update_gait_display(self, gait_data: Dict[str, Any]):
        """Update gait pattern display"""
        try:
            if hasattr(self, 'gait_panel'):
                self.gait_panel.update_gait_pattern(gait_data)
        except Exception as e:
            logger.error(f"Error updating gait display: {e}")
    
    def on_close(self):
        """Handle window close event"""
        self.root.withdraw()


class DataWindow:
    """Joint angles and sensor data window"""
    
    def __init__(self, multi_gui_controller):
        self.controller = multi_gui_controller
        self.root = tk.Tk()
        self.root.title("🤖 HexaPodSim 2.0 - Robot Data")
        self.root.geometry("350x350+730+680")
        self.root.configure(bg=Colors.BACKGROUND)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        self._create_interface()
        logger.info("Data window created")
    
    def _create_interface(self):
        """Create the data display interface"""
        # Title
        title_label = tk.Label(
            self.root,
            text="📊 ROBOT DATA",
            font=('Consolas', 14, 'bold'),
            fg=Colors.ACCENT_1,
            bg=Colors.BACKGROUND
        )
        title_label.pack(pady=5)
        
        # Joint angles panel
        try:
            self.joint_panel = JointAnglePanel(self.root, width=330, height=280)
            self.joint_panel.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        except Exception as e:
            logger.error(f"Failed to create joint panel: {e}")
            # Create fallback
            self.data_text = tk.Text(
                self.root,
                bg=Colors.BACKGROUND,
                fg=Colors.TEXT_PRIMARY,
                font=('Consolas', 9),
                wrap=tk.WORD
            )
            self.data_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
            self.data_text.insert(tk.END, "Joint Angles Data\\n(Fallback Display)")
    
    def update_joint_display(self, joint_data: Dict[str, Any]):
        """Update joint angles display"""
        try:
            if hasattr(self, 'joint_panel'):
                self.joint_panel.update_joint_angles(joint_data)
            elif hasattr(self, 'data_text'):
                self.data_text.delete(1.0, tk.END)
                self.data_text.insert(tk.END, "📊 JOINT ANGLES\\n")
                self.data_text.insert(tk.END, "=" * 20 + "\\n\\n")
                for joint, angles in joint_data.items():
                    self.data_text.insert(tk.END, f"{joint}: {angles}\\n")
        except Exception as e:
            logger.error(f"Error updating joint display: {e}")
    
    def on_close(self):
        """Handle window close event"""
        self.root.withdraw()


class HexaPodSimMultiGUI:
    """Main multi-window GUI controller for HexaPodSim 2.0"""
    
    def __init__(self):
        """Initialize the multi-window GUI system"""
        logger.info("Initializing HexaPodSim 2.0 Multi-Window GUI...")
        
        # Initialize robot systems
        self._init_robot_systems()
        
        # Create windows
        self.windows = {}
        self._create_windows()
        
        # Start update thread
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()
        
        logger.info("Multi-window GUI system initialized")
    
    def _init_robot_systems(self):
        """Initialize robot control systems"""
        try:
            logger.info("Initializing robot systems...")
            
            # Create kinematics
            self.kinematics = HexapodKinematics()
            logger.info("✓ Kinematics system created")
            
            # Create motion controller (creates its own gait generator internally)
            self.motion_controller = MotionController(self.kinematics)
            logger.info("✓ Motion controller created")
            
            # Get gait generator from motion controller
            self.gait_generator = self.motion_controller.gait_generator
            logger.info("✓ Gait generator obtained from motion controller")
            
            # Start motion controller
            self.motion_controller.start()
            logger.info("✓ Motion controller started")
            
            # Initialize robot state
            self.robot_state = RobotState()
            
            logger.info("Robot systems initialized successfully")
            
        except Exception as e:
            logger.error(f"Failed to initialize robot systems: {e}")
            # Create minimal fallback systems
            self.kinematics = None
            self.gait_generator = None
            self.motion_controller = None
            self.robot_state = RobotState()
    
    def _create_windows(self):
        """Create all GUI windows"""
        try:
            logger.info("Creating multi-window interface...")
            
            # Create windows
            self.windows['control'] = ControlWindow(self)
            self.windows['3d'] = Visualization3DWindow(self)
            self.windows['gait'] = GaitWindow(self)
            self.windows['data'] = DataWindow(self)
            
            logger.info("All windows created successfully")
            
        except Exception as e:
            logger.error(f"Failed to create windows: {e}")
            raise
    
    def _update_loop(self):
        """Main update loop for all windows"""
        while self.running:
            try:
                # Get current robot state
                if self.motion_controller:
                    motion_status = self.motion_controller.get_status()
                    self.robot_state = self._convert_motion_status_to_robot_state(motion_status)
                
                # Update status info
                status_info = {
                    "Mode": "Multi-Window",
                    "Simulation": "Active" if self.motion_controller and self.motion_controller.status.state.value == "active" else "Stopped",
                    "Gait": motion_status.current_gait.value if self.motion_controller else "Unknown",
                    "Phase": f"{motion_status.gait_phase * 100:.1f}%" if self.motion_controller else "0.0%",
                    "Position": f"({motion_status.current_velocity[0]:.2f}, {motion_status.current_velocity[1]:.2f})" if self.motion_controller else "(0.0, 0.0)",
                    "Timestamp": time.strftime("%H:%M:%S")
                }
                
                # Update windows (use after method instead of after_idle)
                if 'control' in self.windows:
                    try:
                        self.windows['control'].root.after(0, 
                            lambda: self.windows['control'].update_status(status_info)
                        )
                    except:
                        pass
                
                if '3d' in self.windows and self.robot_state:
                    try:
                        self.windows['3d'].root.after(0,
                            lambda: self.windows['3d'].update_robot_display(self.robot_state)
                        )
                    except:
                        pass
                
                if 'gait' in self.windows:
                    gait_data = {
                        'gait_type': getattr(self.robot_state, 'gait_type', 'tripod'),
                        'phase': getattr(self.robot_state, 'gait_phase', 0.0),
                        'leg_states': getattr(self.robot_state, 'leg_states', {})
                    }
                    try:
                        self.windows['gait'].root.after(0,
                            lambda: self.windows['gait'].update_gait_display(gait_data)
                        )
                    except:
                        pass
                
                if 'data' in self.windows:
                    joint_data = self.robot_state.joint_angles if self.robot_state else {}
                    try:
                        self.windows['data'].root.after(0,
                            lambda: self.windows['data'].update_joint_display(joint_data)
                        )
                    except:
                        pass
                
                time.sleep(0.1)  # 10 Hz update rate
                
            except Exception as e:
                logger.error(f"Error in update loop: {e}")
                time.sleep(1.0)  # Slow down on error
    
    def _convert_motion_status_to_robot_state(self, motion_status) -> RobotState:
        """Convert MotionStatus to RobotState for GUI compatibility"""
        robot_state = RobotState()
        
        # Update joint angles
        if hasattr(motion_status, 'joint_angles') and len(motion_status.joint_angles) >= 18:
            robot_state.joint_angles = {
                'L1': {'coxa': motion_status.joint_angles[0], 'femur': motion_status.joint_angles[1], 'tibia': motion_status.joint_angles[2]},
                'R1': {'coxa': motion_status.joint_angles[3], 'femur': motion_status.joint_angles[4], 'tibia': motion_status.joint_angles[5]},
                'L2': {'coxa': motion_status.joint_angles[6], 'femur': motion_status.joint_angles[7], 'tibia': motion_status.joint_angles[8]},
                'R2': {'coxa': motion_status.joint_angles[9], 'femur': motion_status.joint_angles[10], 'tibia': motion_status.joint_angles[11]},
                'L3': {'coxa': motion_status.joint_angles[12], 'femur': motion_status.joint_angles[13], 'tibia': motion_status.joint_angles[14]},
                'R3': {'coxa': motion_status.joint_angles[15], 'femur': motion_status.joint_angles[16], 'tibia': motion_status.joint_angles[17]}
            }
        
        # Update gait information
        robot_state.gait_type = motion_status.current_gait.value if hasattr(motion_status, 'current_gait') else 'tripod'
        robot_state.gait_phase = motion_status.gait_phase if hasattr(motion_status, 'gait_phase') else 0.0
        
        # Update position (use current velocity as proxy for position change)
        if hasattr(motion_status, 'current_velocity'):
            robot_state.position = motion_status.current_velocity[:2]  # x, y components
        
        return robot_state
    
    def handle_movement(self, linear_x: float, linear_y: float, angular_z: float):
        """Handle robot movement commands"""
        try:
            print(f"🎮 BUTTON PRESSED: Movement command received: ({linear_x}, {linear_y}, {angular_z})")
            logger.info(f"🎮 BUTTON PRESSED: Movement command: ({linear_x}, {linear_y}, {angular_z})")
            
            if self.motion_controller:
                # Create motion command with velocity
                command = MotionCommand(
                    velocity=np.array([linear_x, linear_y, 0.0]),  # vx, vy, vz
                    angular_velocity=np.radians(angular_z),  # Convert degrees to radians
                    max_speed=0.3,  # reasonable max speed
                    max_angular_speed=1.0
                )
                result = self.motion_controller.set_motion_command(command)
                print(f"✅ Motion command sent to controller, result: {result}")
                logger.info(f"✅ Motion command sent to controller, result: {result}")
            else:
                print("❌ No motion controller available")
                logger.error("❌ No motion controller available")
        except Exception as e:
            print(f"💥 Error handling movement: {e}")
            logger.error(f"Error handling movement: {e}")
    
    def start_simulation(self):
        """Start robot simulation"""
        print("🚀 START button pressed!")
        logger.info("🚀 START button pressed!")
        try:
            if self.motion_controller:
                # Check current state
                print(f"📊 Motion controller state before start:")
                print(f"   - Running: {self.motion_controller.running}")
                print(f"   - Emergency stop: {self.motion_controller.status.emergency_stop_active}")
                print(f"   - Mode: {self.motion_controller.status.mode}")
                print(f"   - State: {self.motion_controller.status.state}")
                
                if self.motion_controller.running:
                    print("ℹ️ Motion controller is already running - START successful!")
                    logger.info("Motion controller is already running - START successful!")
                    # Update GUI status to show START worked
                    try:
                        if hasattr(self, 'control_window') and hasattr(self.control_window, 'control_panel'):
                            control_panel = self.control_window.control_panel
                            if hasattr(control_panel, 'update_status'):
                                control_panel.update_status("🚀 SYSTEM READY - Robot is active and responsive!\n")
                    except Exception as status_error:
                        print(f"⚠️ Could not update status display: {status_error}")
                else:
                    # Try to start
                    result = self.motion_controller.start()
                    print(f"✅ Motion controller start result: {result}")
                    
                    # Check state after start
                    print(f"📊 Motion controller state after start:")
                    print(f"   - Running: {self.motion_controller.running}")
                    print(f"   - Emergency stop: {self.motion_controller.status.emergency_stop_active}")
                    print(f"   - Mode: {self.motion_controller.status.mode}")
                    print(f"   - State: {self.motion_controller.status.state}")
                
                logger.info("Simulation started")
            else:
                print("❌ No motion controller available")
                logger.error("No motion controller available")
        except Exception as e:
            print(f"💥 Error starting simulation: {e}")
            logger.error(f"Error starting simulation: {e}")
            import traceback
            traceback.print_exc()
    
    def stop_simulation(self):
        """Stop robot simulation"""
        try:
            if self.motion_controller:
                self.motion_controller.stop()
                logger.info("Simulation stopped")
        except Exception as e:
            logger.error(f"Error stopping simulation: {e}")
    
    def reset_robot(self):
        """Reset robot to initial state"""
        print("🔄 RESET button pressed!")
        logger.info("🔄 RESET button pressed!")
        try:
            if self.motion_controller:
                # Check state before reset
                print(f"📊 Motion controller state before reset:")
                print(f"   - Running: {self.motion_controller.running}")
                print(f"   - Emergency stop: {self.motion_controller.status.emergency_stop_active}")
                print(f"   - Mode: {self.motion_controller.status.mode}")
                print(f"   - State: {self.motion_controller.status.state}")
                
                print("🔧 Calling motion controller reset...")
                result = self.motion_controller.reset()
                print(f"✅ Motion controller reset result: {result}")
                
                # Update GUI status to show RESET worked
                print("🖥️ Attempting to update GUI status...")
                try:
                    if 'control' in self.windows and hasattr(self.windows['control'], 'control_panel'):
                        control_panel = self.windows['control'].control_panel
                        if hasattr(control_panel, 'update_status'):
                            if result:
                                control_panel.update_status("🔄 RESET COMPLETE - Emergency stops cleared, system ready!")
                                print("✅ Status updated: RESET COMPLETE")
                            else:
                                control_panel.update_status("❌ RESET FAILED - Check system status")
                                print("⚠️ Status updated: RESET FAILED")
                        else:
                            print("⚠️ Control panel has no update_status method")
                    else:
                        print("⚠️ Control window or control panel not found")
                except Exception as status_error:
                    print(f"💥 Error updating status display: {status_error}")
                    import traceback
                    traceback.print_exc()
                
                # Check state after reset
                print("📊 Motion controller state after reset:")
                print(f"   - Running: {self.motion_controller.running}")
                print(f"   - Emergency stop: {self.motion_controller.status.emergency_stop_active}")
                print(f"   - Mode: {self.motion_controller.status.mode}")
                print(f"   - State: {self.motion_controller.status.state}")
                
                logger.info("Robot reset")
                print("✅ RESET operation completed successfully")
            else:
                print("❌ No motion controller available")
                logger.error("❌ No motion controller available")
        except Exception as e:
            print(f"💥 Error resetting robot: {e}")
            logger.error(f"Error resetting robot: {e}")
            import traceback
            traceback.print_exc()
    
    def emergency_stop(self):
        """Emergency stop the robot"""
        try:
            if self.motion_controller:
                self.motion_controller.emergency_stop()
                logger.warning("Emergency stop activated")
        except Exception as e:
            logger.error(f"Error in emergency stop: {e}")
    
    def calibrate_robot(self):
        """Calibrate robot systems"""
        logger.info("Robot calibration initiated")
    
    def run_self_test(self):
        """Run robot self-test"""
        logger.info("Robot self-test initiated")
    
    def show_all_windows(self):
        """Show all windows"""
        try:
            for window in self.windows.values():
                window.root.deiconify()
                window.root.lift()
            logger.info("All windows shown")
        except Exception as e:
            logger.error(f"Error showing windows: {e}")
    
    def close_all_windows(self):
        """Close all windows and exit application"""
        logger.info("Closing all windows...")
        self.running = False
        
        try:
            if self.motion_controller:
                self.motion_controller.stop()
        except Exception as e:
            logger.error(f"Error stopping motion controller: {e}")
        
        try:
            for window in self.windows.values():
                window.root.quit()
                window.root.destroy()
        except Exception as e:
            logger.error(f"Error closing windows: {e}")
    
    def run(self):
        """Run the multi-window GUI application"""
        try:
            logger.info("Starting multi-window GUI mainloop...")
            
            # Start the main control window's mainloop
            # Other windows will be managed by the control window
            self.windows['control'].root.mainloop()
            
            logger.info("Multi-window GUI ended")
            
        except Exception as e:
            logger.error(f"Error in GUI mainloop: {e}")
        finally:
            self.running = False
            logger.info("Multi-window GUI cleaned up")


def test_multi_window_gui():
    """Test function for multi-window GUI"""
    print("🧪 Testing Multi-Window GUI Components...")
    
    try:
        app = HexaPodSimMultiGUI()
        app.run()
    except Exception as e:
        print(f"Test failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    test_multi_window_gui()