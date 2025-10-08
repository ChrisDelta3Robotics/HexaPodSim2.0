"""
HexaPodSim 2.0 - Main GUI Application

This module implements the modern neon-themed GUI interface for the HexaPodSim 2.0
hexapod robot simulation. Features include:
- Real-time 3D robot visualization  
- Interactive control panels
- Gait pattern display
- Navigation controls
- System monitoring

Author: HexaPodSim Team
Date: 2024
"""

import tkinter as tk
from tkinter import ttk
import matplotlib
matplotlib.use('TkAgg')  # Force matplotlib to use TkAgg backend
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.patches as patches
import numpy as np
import threading
import time
import logging
from typing import Dict, List, Tuple, Optional, Callable
from dataclasses import dataclass
from enum import Enum

# Configure logging first
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import hexapod modules with fallback to mocks
try:
    from hexapod.kinematics import HexapodKinematics
except ImportError:
    logger.warning("Kinematics module not found, using mock")
    class HexapodKinematics:
        def __init__(self):
            pass

try:
    from hexapod.gait import GaitGenerator, GaitType
except ImportError:
    logger.warning("Gait generation module not found, using mock")
    class GaitType(Enum):
        TRIPOD = "tripod"
        WAVE = "wave"
        RIPPLE = "ripple"
    
    class GaitGenerator:
        def __init__(self, kinematics=None):
            self.gait_type = GaitType.TRIPOD
            self.velocity = (0, 0, 0)
        
        def set_gait_type(self, gait_type):
            self.gait_type = gait_type
        
        def set_velocity(self, vx, vy, wz):
            self.velocity = (vx, vy, wz)

try:
    from hexapod.motion import MotionController
except ImportError:
    logger.warning("Motion controller module not found, using mock")
    class MotionController:
        def __init__(self):
            pass

try:
    from hexapod.planner import HierarchicalController, OccupancyGrid, NavigationCommand
except ImportError:
    logger.warning("Path planner module not found, using mock")
    class HierarchicalController:
        def __init__(self):
            pass
    class OccupancyGrid:
        def __init__(self):
            pass
    class NavigationCommand:
        def __init__(self):
            pass

# GUI Configuration Constants
class Colors:
    """Neon color scheme for modern GUI"""
    BACKGROUND = '#000000'      # Black background
    PANEL_BG = '#111111'        # Dark panel background
    ACCENT_1 = '#00FF00'        # Neon Green (L1)
    ACCENT_2 = '#00FFFF'        # Neon Cyan (R1)
    ACCENT_3 = '#FF00FF'        # Neon Magenta (L2)
    ACCENT_4 = '#FFFF00'        # Neon Yellow (R2)
    ACCENT_5 = '#FF6600'        # Neon Orange (L3)
    ACCENT_6 = '#FF0080'        # Neon Pink (R3)
    TEXT_PRIMARY = '#FFFFFF'    # White text
    TEXT_SECONDARY = '#CCCCCC'  # Light gray text
    BUTTON_NORMAL = '#666666'   # More visible button background
    BUTTON_ACTIVE = '#888888'   # Brighter active button
    BUTTON_HOVER = '#777777'    # Hover button
    BORDER = '#888888'          # Lighter border color

class LEG_CONFIG:
    """Leg configuration with star formation"""
    INWARD_LEG_ANGLE = 15.0  # degrees
    LEG_NAMES = ['L1', 'R1', 'L2', 'R2', 'L3', 'R3']
    LEG_COLORS = {
        'L1': Colors.ACCENT_1,  # Neon Green (Left Front)
        'R1': Colors.ACCENT_2,  # Neon Cyan (Right Front)
        'L2': Colors.ACCENT_3,  # Neon Magenta (Left Middle)
        'R2': Colors.ACCENT_4,  # Neon Yellow (Right Middle)
        'L3': Colors.ACCENT_5,  # Neon Orange (Left Back)
        'R3': Colors.ACCENT_6   # Neon Pink (Right Back)
    }
    LEG_POSITIONS = {
        'L1': (105.0, 0.15),    # Front left (105° = 90° + 15°)
        'R1': (75.0, 0.15),     # Front right (75° = 90° - 15°)
        'L2': (90.0, 0.15),     # Middle left
        'R2': (90.0, 0.15),     # Middle right
        'L3': (75.0, 0.15),     # Back left (75° = 90° - 15°)
        'R3': (105.0, 0.15)     # Back right (105° = 90° + 15°)
    }

@dataclass
class RobotState:
    """Current state of the hexapod robot"""
    position: Tuple[float, float, float] = (0.0, 0.0, 0.15)
    orientation: float = 0.0  # degrees
    velocity: Tuple[float, float] = (0.0, 0.0)  # linear x, y
    angular_velocity: float = 0.0  # degrees/sec
    joint_angles: Dict[str, Tuple[float, float, float]] = None
    foot_positions: Dict[str, Tuple[float, float, float]] = None
    gait_type: str = "tripod"
    gait_phase: float = 0.0
    is_moving: bool = False
    battery_level: float = 100.0
    cpu_load: float = 0.0
    memory_usage: float = 0.0
    fps: float = 60.0
    status: str = "IDLE"
    errors: int = 0
    warnings: int = 0

    def __post_init__(self):
        if self.joint_angles is None:
            self.joint_angles = {leg: (0.0, 0.0, 0.0) for leg in LEG_CONFIG.LEG_NAMES}
        if self.foot_positions is None:
            self.foot_positions = {leg: (0.0, 0.0, -0.2) for leg in LEG_CONFIG.LEG_NAMES}

class GaitVisualizationPanel:
    """Panel for displaying gait patterns and timing"""
    
    def __init__(self, parent, width=600, height=200):
        self.parent = parent
        self.width = width
        self.height = height
        
        # Create frame
        self.frame = tk.Frame(parent, bg=Colors.PANEL_BG, relief=tk.RAISED, bd=2)
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(width/100, height/100), facecolor=Colors.BACKGROUND)
        self.ax = self.fig.add_subplot(111, facecolor=Colors.BACKGROUND)
        
        # Configure plot
        self.ax.set_xlim(0, 100)  # 0-100% gait cycle
        self.ax.set_ylim(-0.5, 5.5)  # 6 legs
        self.ax.set_xlabel('Gait Cycle (%)', color=Colors.TEXT_PRIMARY)
        self.ax.set_ylabel('Legs', color=Colors.TEXT_PRIMARY)
        self.ax.tick_params(colors=Colors.TEXT_PRIMARY)
        
        # Set leg labels
        leg_labels = ['R3', 'L3', 'R2', 'L2', 'R1', 'L1']
        self.ax.set_yticks(range(6))
        self.ax.set_yticklabels(leg_labels)
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Current gait data
        self.current_gait = "tripod"
        self.gait_phase = 0.0
        self.leg_states = {leg: False for leg in LEG_CONFIG.LEG_NAMES}  # False=stance, True=swing
        
        logger.info("Gait visualization panel initialized")
    
    def update_gait_pattern(self, gait_type: str, phase: float, leg_states: Dict[str, bool]):
        """Update gait pattern display"""
        self.current_gait = gait_type
        self.gait_phase = phase
        self.leg_states = leg_states
        
        # Clear previous plot
        self.ax.clear()
        
        # Configure plot
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(-0.5, 5.5)
        self.ax.set_xlabel('Gait Cycle (%)', color=Colors.TEXT_PRIMARY)
        self.ax.set_ylabel('Legs', color=Colors.TEXT_PRIMARY)
        self.ax.tick_params(colors=Colors.TEXT_PRIMARY)
        self.ax.set_facecolor(Colors.BACKGROUND)
        
        # Set leg labels and colors
        leg_labels = ['R3', 'L3', 'R2', 'L2', 'R1', 'L1']
        leg_colors = [LEG_CONFIG.LEG_COLORS['R3'], LEG_CONFIG.LEG_COLORS['L3'],
                     LEG_CONFIG.LEG_COLORS['R2'], LEG_CONFIG.LEG_COLORS['L2'],
                     LEG_CONFIG.LEG_COLORS['R1'], LEG_CONFIG.LEG_COLORS['L1']]
        
        self.ax.set_yticks(range(6))
        self.ax.set_yticklabels(leg_labels)
        
        # Draw gait pattern based on type
        if gait_type.lower() == "tripod":
            self._draw_tripod_gait()
        elif gait_type.lower() == "wave":
            self._draw_wave_gait()
        elif gait_type.lower() == "ripple":
            self._draw_ripple_gait()
        
        # Draw current phase indicator
        phase_x = phase % 100
        self.ax.axvline(x=phase_x, color=Colors.TEXT_PRIMARY, linestyle='--', linewidth=2, alpha=0.8)
        
        # Add title
        self.ax.set_title(f'{gait_type.title()} Gait - Phase: {phase:.1f}%', 
                         color=Colors.TEXT_PRIMARY, fontsize=12)
        
        self.canvas.draw()
    
    def _draw_tripod_gait(self):
        """Draw tripod gait pattern"""
        # Tripod gait: L1,R2,L3 move together, then R1,L2,R3
        for i, leg in enumerate(['R3', 'L3', 'R2', 'L2', 'R1', 'L1']):
            y_pos = i
            leg_color = LEG_CONFIG.LEG_COLORS[leg]
            
            if leg in ['L1', 'R2', 'L3']:
                # First tripod group: swing 0-50%, stance 50-100%
                swing_rect = patches.Rectangle((0, y_pos-0.3), 50, 0.6, 
                                             facecolor=leg_color, alpha=0.7)
                stance_rect = patches.Rectangle((50, y_pos-0.3), 50, 0.6, 
                                              facecolor=leg_color, alpha=0.3)
            else:
                # Second tripod group: stance 0-50%, swing 50-100%
                stance_rect = patches.Rectangle((0, y_pos-0.3), 50, 0.6, 
                                              facecolor=leg_color, alpha=0.3)
                swing_rect = patches.Rectangle((50, y_pos-0.3), 50, 0.6, 
                                             facecolor=leg_color, alpha=0.7)
            
            self.ax.add_patch(swing_rect)
            self.ax.add_patch(stance_rect)
    
    def _draw_wave_gait(self):
        """Draw wave gait pattern"""
        # Wave gait: legs move in sequence L1->L2->L3->R3->R2->R1
        wave_sequence = ['L1', 'L2', 'L3', 'R3', 'R2', 'R1']
        swing_duration = 100 / 6  # Each leg swings for 1/6 of cycle
        
        for i, leg in enumerate(['R3', 'L3', 'R2', 'L2', 'R1', 'L1']):
            y_pos = i
            leg_color = LEG_CONFIG.LEG_COLORS[leg]
            
            # Find when this leg swings in the wave sequence
            wave_index = wave_sequence.index(leg)
            swing_start = wave_index * swing_duration
            swing_end = swing_start + swing_duration
            
            # Draw stance phases
            if swing_start > 0:
                stance1 = patches.Rectangle((0, y_pos-0.3), swing_start, 0.6,
                                          facecolor=leg_color, alpha=0.3)
                self.ax.add_patch(stance1)
            
            if swing_end < 100:
                stance2 = patches.Rectangle((swing_end, y_pos-0.3), 100-swing_end, 0.6,
                                          facecolor=leg_color, alpha=0.3)
                self.ax.add_patch(stance2)
            
            # Draw swing phase
            swing_rect = patches.Rectangle((swing_start, y_pos-0.3), swing_duration, 0.6,
                                         facecolor=leg_color, alpha=0.7)
            self.ax.add_patch(swing_rect)
    
    def _draw_ripple_gait(self):
        """Draw ripple gait pattern"""
        # Ripple gait: Similar to wave but with overlap
        ripple_sequence = ['L1', 'R1', 'L2', 'R2', 'L3', 'R3']
        swing_duration = 25  # Each leg swings for 25% of cycle
        phase_offset = 100 / 6  # 16.67% offset between legs
        
        for i, leg in enumerate(['R3', 'L3', 'R2', 'L2', 'R1', 'L1']):
            y_pos = i
            leg_color = LEG_CONFIG.LEG_COLORS[leg]
            
            # Find when this leg swings in the ripple sequence
            ripple_index = ripple_sequence.index(leg)
            swing_start = (ripple_index * phase_offset) % 100
            swing_end = (swing_start + swing_duration) % 100
            
            if swing_end > swing_start:
                # Normal case: swing doesn't wrap around
                stance1 = patches.Rectangle((0, y_pos-0.3), swing_start, 0.6,
                                          facecolor=leg_color, alpha=0.3)
                swing_rect = patches.Rectangle((swing_start, y_pos-0.3), swing_duration, 0.6,
                                             facecolor=leg_color, alpha=0.7)
                stance2 = patches.Rectangle((swing_end, y_pos-0.3), 100-swing_end, 0.6,
                                          facecolor=leg_color, alpha=0.3)
                self.ax.add_patch(stance1)
                self.ax.add_patch(swing_rect)
                self.ax.add_patch(stance2)
            else:
                # Wrap around case
                swing1 = patches.Rectangle((swing_start, y_pos-0.3), 100-swing_start, 0.6,
                                         facecolor=leg_color, alpha=0.7)
                swing2 = patches.Rectangle((0, y_pos-0.3), swing_end, 0.6,
                                         facecolor=leg_color, alpha=0.7)
                stance_rect = patches.Rectangle((swing_end, y_pos-0.3), swing_start-swing_end, 0.6,
                                              facecolor=leg_color, alpha=0.3)
                self.ax.add_patch(swing1)
                self.ax.add_patch(swing2)
                self.ax.add_patch(stance_rect)
    
    def pack(self, **kwargs):
        """Pack the frame"""
        self.frame.pack(**kwargs)
    
    def grid(self, **kwargs):
        """Grid the frame"""
        self.frame.grid(**kwargs)


class Robot3DVisualization:
    """3D robot visualization panel"""
    
    def __init__(self, parent, width=400, height=400):
        self.parent = parent
        self.width = width
        self.height = height
        
        # Create frame
        self.frame = tk.Frame(parent, bg=Colors.PANEL_BG, relief=tk.RAISED, bd=2)
        
        # Create title label
        title_label = tk.Label(self.frame, text="3D Robot View", 
                              bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY,
                              font=("Arial", 12, "bold"))
        title_label.pack(pady=5)
        
        try:
            # Create matplotlib figure for 3D plot
            self.fig = Figure(figsize=(6, 7), facecolor=Colors.BACKGROUND)
            self.ax = self.fig.add_subplot(111, projection='3d', facecolor=Colors.BACKGROUND)
            
            # Configure 3D plot
            self.ax.set_xlim(-0.3, 0.3)
            self.ax.set_ylim(-0.3, 0.3)
            self.ax.set_zlim(-0.3, 0.1)
            self.ax.set_xlabel('X (m)', color=Colors.TEXT_PRIMARY)
            self.ax.set_ylabel('Y (m)', color=Colors.TEXT_PRIMARY)
            self.ax.set_zlabel('Z (m)', color=Colors.TEXT_PRIMARY)
            self.ax.tick_params(colors=Colors.TEXT_PRIMARY)
            
            # Create canvas
            self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
            canvas_widget = self.canvas.get_tk_widget()
            canvas_widget.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
            
            logger.info("3D matplotlib canvas created and embedded")
            
            # Robot state
            self.robot_state = RobotState()
            self.show_grid = True
            self.show_shadows = True
            self.show_coordinates = True
            self.show_trails = False
            
            # Initialize kinematics for visualization
            try:
                from .kinematics import HexapodKinematics
                self.kinematics = HexapodKinematics()
            except Exception as e:
                logger.warning(f"Could not load kinematics for 3D visualization: {e}")
                self.kinematics = None
            
            # Draw initial robot
            self._draw_initial_robot()
            
            logger.info("3D robot visualization initialized")
            
        except Exception as e:
            logger.error(f"Failed to create 3D visualization: {e}")
            # Create fallback display
            fallback_label = tk.Label(self.frame, 
                                    text="3D Visualization Loading...\n(Matplotlib 3D)",
                                    bg=Colors.PANEL_BG, fg=Colors.TEXT_SECONDARY,
                                    font=("Arial", 10))
            fallback_label.pack(fill=tk.BOTH, expand=True)
            self.canvas = None
            self.ax = None
    
    def _draw_initial_robot(self):
        """Draw initial robot pose"""
        if self.ax is None:
            return
            
        try:
            # Clear any existing plots first
            self.ax.clear()
            
            # Configure 3D plot again after clearing
            self.ax.set_xlim(-0.3, 0.3)
            self.ax.set_ylim(-0.3, 0.3)
            self.ax.set_zlim(-0.3, 0.1)
            self.ax.set_xlabel('X (m)', color=Colors.TEXT_PRIMARY)
            self.ax.set_ylabel('Y (m)', color=Colors.TEXT_PRIMARY)
            self.ax.set_zlabel('Z (m)', color=Colors.TEXT_PRIMARY)
            self.ax.tick_params(colors=Colors.TEXT_PRIMARY)
            self.ax.set_facecolor(Colors.BACKGROUND)
            
            # Draw simple robot body (hexagon) - SAME AS WORKING TEST
            import numpy as np
            angles = np.linspace(0, 2*np.pi, 7)
            body_radius = 0.1
            x_body = body_radius * np.cos(angles)
            y_body = body_radius * np.sin(angles)
            z_body = np.zeros_like(x_body)
            
            self.ax.plot(x_body, y_body, z_body, color='white', linewidth=3, label='Robot Body')
            
            # Draw robot legs (6 legs in star formation) - SAME AS WORKING TEST
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
                self.ax.plot([base_x, foot_x], [base_y, foot_y], [base_z, foot_z], 
                           color=color, linewidth=3, alpha=0.8)
                
                # Draw foot
                self.ax.scatter([foot_x], [foot_y], [foot_z], color=color, s=50)
            
            # Draw grid
            if self.show_grid:
                self._draw_grid()
            
            # Draw coordinate frame
            if self.show_coordinates:
                self._draw_coordinate_frame()
            
            # Set title
            self.ax.set_title('Hexapod Robot 3D View', color='white')
            
            logger.info("3D robot drawn successfully")
            
        except Exception as e:
            logger.error(f"Error drawing initial robot: {e}")
            import traceback
            traceback.print_exc()
        
        # Force canvas update
        if self.canvas:
            try:
                self.canvas.draw()
                logger.info("3D canvas updated successfully")
            except Exception as e:
                logger.error(f"Error updating 3D canvas: {e}")
    
    def update_robot_display(self, robot_state: RobotState):
        """Update 3D robot display"""
        if self.ax is None or self.canvas is None:
            return
            
        self.robot_state = robot_state
        
        try:
            # Clear previous plot
            self.ax.clear()
            
            # Configure 3D plot
            self.ax.set_xlim(-0.3, 0.3)
            self.ax.set_ylim(-0.3, 0.3)
            self.ax.set_zlim(-0.3, 0.1)
            self.ax.set_xlabel('X (m)', color=Colors.TEXT_PRIMARY)
            self.ax.set_ylabel('Y (m)', color=Colors.TEXT_PRIMARY)
            self.ax.set_zlabel('Z (m)', color=Colors.TEXT_PRIMARY)
            self.ax.tick_params(colors=Colors.TEXT_PRIMARY)
            self.ax.set_facecolor(Colors.BACKGROUND)
            
            # Draw robot body (hexagon)
            self._draw_robot_body()
            
            # Draw legs
            self._draw_robot_legs()
            
            # Draw grid if enabled
            if self.show_grid:
                self._draw_grid()
            
            # Draw coordinate frame if enabled
            if self.show_coordinates:
                self._draw_coordinate_frame()
            
            self.canvas.draw()
            
        except Exception as e:
            logger.error(f"Error updating 3D robot display: {e}")
    
    def _draw_robot_body(self):
        """Draw hexagonal robot body"""
        # Body dimensions
        body_radius = 0.1
        body_height = 0.02
        
        # Create hexagon vertices
        angles = np.linspace(0, 2*np.pi, 7)  # 7 points to close the hexagon
        x_body = body_radius * np.cos(angles)
        y_body = body_radius * np.sin(angles)
        z_body = np.zeros_like(x_body) + body_height/2
        
        # Draw body outline
        self.ax.plot(x_body, y_body, z_body, color=Colors.TEXT_PRIMARY, linewidth=2)
        
        # Draw body top surface
        self.ax.plot_trisurf(x_body[:-1], y_body[:-1], z_body[:-1], 
                            color=Colors.BUTTON_NORMAL, alpha=0.3)
    
    def _draw_robot_legs(self):
        """Draw all robot legs with star configuration"""
        for leg_name in LEG_CONFIG.LEG_NAMES:
            self._draw_single_leg(leg_name)
    
    def _draw_single_leg(self, leg_name: str):
        """Draw a single leg"""
        if self.ax is None:
            return
            
        # Get leg color
        leg_color = LEG_CONFIG.LEG_COLORS.get(leg_name, Colors.TEXT_PRIMARY)
        
        # Get joint angles (with fallbacks)
        if leg_name in self.robot_state.joint_angles:
            coxa_angle, femur_angle, tibia_angle = self.robot_state.joint_angles[leg_name]
        else:
            # Default angles
            coxa_angle, femur_angle, tibia_angle = (0.0, -20.0, 40.0)
        
        # Calculate leg base position with star configuration
        base_angle, base_radius = LEG_CONFIG.LEG_POSITIONS.get(leg_name, (0.0, 0.08))
        
        # Apply star configuration offset
        if leg_name in ['L1', 'R3']:  # Front and back get inward angle
            if 'L' in leg_name:
                base_angle += LEG_CONFIG.INWARD_LEG_ANGLE
            else:
                base_angle -= LEG_CONFIG.INWARD_LEG_ANGLE
        elif leg_name in ['L3', 'R1']:  # Other front and back
            if 'L' in leg_name:
                base_angle -= LEG_CONFIG.INWARD_LEG_ANGLE
            else:
                base_angle += LEG_CONFIG.INWARD_LEG_ANGLE
        
        # Convert to radians for calculation
        base_angle_rad = np.radians(base_angle)
        
        # Base position on body
        base_x = 0.08 * np.cos(base_angle_rad)
        base_y = 0.08 * np.sin(base_angle_rad)
        base_z = 0.01
        
        # Leg segment lengths
        coxa_length = 0.04
        femur_length = 0.08
        tibia_length = 0.12
        
        # Calculate joint positions using forward kinematics if available
        if self.kinematics:
            try:
                foot_pos = self.kinematics.forward_kinematics(leg_name, 
                                                            np.radians(coxa_angle), 
                                                            np.radians(femur_angle), 
                                                            np.radians(tibia_angle))
                
                # Calculate intermediate joint positions (simplified)
                coxa_end_x = base_x + coxa_length * np.cos(np.radians(base_angle + coxa_angle))
                coxa_end_y = base_y + coxa_length * np.sin(np.radians(base_angle + coxa_angle))
                coxa_end_z = base_z
                
                # Femur end (knee) - simplified calculation
                femur_angle_world = femur_angle  
                knee_x = coxa_end_x + femur_length * np.cos(np.radians(base_angle + coxa_angle)) * np.cos(np.radians(femur_angle_world))
                knee_y = coxa_end_y + femur_length * np.sin(np.radians(base_angle + coxa_angle)) * np.cos(np.radians(femur_angle_world))
                knee_z = coxa_end_z + femur_length * np.sin(np.radians(femur_angle_world))
                
                # Foot position from kinematics
                foot_x, foot_y, foot_z = foot_pos
                
            except Exception as e:
                # Fallback to simple visualization
                logger.debug(f"Kinematics calculation failed for {leg_name}: {e}")
                foot_x, foot_y, foot_z, coxa_end_x, coxa_end_y, coxa_end_z, knee_x, knee_y, knee_z = self._calculate_simple_leg_positions(base_x, base_y, base_z, base_angle_rad, coxa_angle, femur_angle, tibia_angle)
        else:
            # Simple leg visualization without kinematics
            foot_x, foot_y, foot_z, coxa_end_x, coxa_end_y, coxa_end_z, knee_x, knee_y, knee_z = self._calculate_simple_leg_positions(base_x, base_y, base_z, base_angle_rad, coxa_angle, femur_angle, tibia_angle)
        
        # Draw leg segments
        try:
            # Coxa (hip)
            self.ax.plot([base_x, coxa_end_x], [base_y, coxa_end_y], [base_z, coxa_end_z], 
                        color=leg_color, linewidth=3, alpha=0.8)
            
            # Femur (thigh)
            self.ax.plot([coxa_end_x, knee_x], [coxa_end_y, knee_y], [coxa_end_z, knee_z], 
                        color=leg_color, linewidth=3, alpha=0.8)
            
            # Tibia (shin)
            self.ax.plot([knee_x, foot_x], [knee_y, foot_y], [knee_z, foot_z], 
                        color=leg_color, linewidth=3, alpha=0.8)
            
            # Draw joints
            self.ax.scatter([base_x, coxa_end_x, knee_x, foot_x], 
                           [base_y, coxa_end_y, knee_y, foot_y],
                           [base_z, coxa_end_z, knee_z, foot_z], 
                           color=leg_color, s=20)
            
            # Draw foot contact point
            if foot_z <= -0.18:  # If foot is on ground
                self.ax.scatter([foot_x], [foot_y], [-0.2], color=leg_color, s=50, marker='o')
                
        except Exception as e:
            logger.error(f"Error drawing leg {leg_name}: {e}")
    
    def _calculate_simple_leg_positions(self, base_x, base_y, base_z, base_angle_rad, coxa_angle, femur_angle, tibia_angle):
        """Calculate simple leg positions without full kinematics"""
        # Simple fallback calculations
        coxa_length = 0.04
        femur_length = 0.08
        tibia_length = 0.12
        
        # Coxa end
        coxa_end_x = base_x + coxa_length * np.cos(base_angle_rad + np.radians(coxa_angle))
        coxa_end_y = base_y + coxa_length * np.sin(base_angle_rad + np.radians(coxa_angle))
        coxa_end_z = base_z
        
        # Knee position (simplified)
        knee_x = coxa_end_x + femur_length * np.cos(base_angle_rad + np.radians(coxa_angle))
        knee_y = coxa_end_y + femur_length * np.sin(base_angle_rad + np.radians(coxa_angle))
        knee_z = base_z + femur_length * np.sin(np.radians(femur_angle))
        
        # Foot position (simplified)
        foot_x = knee_x + tibia_length * np.cos(base_angle_rad + np.radians(coxa_angle))
        foot_y = knee_y + tibia_length * np.sin(base_angle_rad + np.radians(coxa_angle))
        foot_z = knee_z + tibia_length * np.sin(np.radians(tibia_angle))
        
        return foot_x, foot_y, foot_z, coxa_end_x, coxa_end_y, coxa_end_z, knee_x, knee_y, knee_z
        
        # Draw leg segments
        try:
            # Coxa (hip)
            self.ax.plot([base_x, coxa_end_x], [base_y, coxa_end_y], [base_z, coxa_end_z], 
                        color=leg_color, linewidth=3, alpha=0.8)
            
            # Femur (thigh)
            self.ax.plot([coxa_end_x, knee_x], [coxa_end_y, knee_y], [coxa_end_z, knee_z], 
                        color=leg_color, linewidth=3, alpha=0.8)
            
            # Tibia (shin)
            self.ax.plot([knee_x, foot_x], [knee_y, foot_y], [knee_z, foot_z], 
                        color=leg_color, linewidth=3, alpha=0.8)
            
            # Draw joints
            self.ax.scatter([base_x, coxa_end_x, knee_x, foot_x], 
                           [base_y, coxa_end_y, knee_y, foot_y],
                           [base_z, coxa_end_z, knee_z, foot_z], 
                           color=leg_color, s=20)
            
            # Draw foot contact point
            if foot_z <= -0.18:  # If foot is on ground
                self.ax.scatter([foot_x], [foot_y], [-0.2], color=leg_color, s=50, marker='o')
                
        except Exception as e:
            logger.error(f"Error drawing leg {leg_name}: {e}")
    
    def _calculate_simple_leg_positions(self, base_x, base_y, base_z, base_angle_rad, coxa_angle, femur_angle, tibia_angle):
        """Calculate simple leg positions without full kinematics"""
        # Simple fallback calculations
        coxa_length = 0.04
        femur_length = 0.08
        tibia_length = 0.12
        
        # Coxa end
        coxa_end_x = base_x + coxa_length * np.cos(base_angle_rad + np.radians(coxa_angle))
        coxa_end_y = base_y + coxa_length * np.sin(base_angle_rad + np.radians(coxa_angle))
        coxa_end_z = base_z
        
        # Knee position (simplified)
        knee_x = coxa_end_x + femur_length * np.cos(base_angle_rad + np.radians(coxa_angle))
        knee_y = coxa_end_y + femur_length * np.sin(base_angle_rad + np.radians(coxa_angle))
        knee_z = base_z + femur_length * np.sin(np.radians(femur_angle))
        
        # Foot position (simplified)
        foot_x = knee_x + tibia_length * np.cos(base_angle_rad + np.radians(coxa_angle))
        foot_y = knee_y + tibia_length * np.sin(base_angle_rad + np.radians(coxa_angle))
        foot_z = knee_z + tibia_length * np.sin(np.radians(tibia_angle))
        
        return foot_x, foot_y, foot_z, coxa_end_x, coxa_end_y, coxa_end_z, knee_x, knee_y, knee_z
    
    def _draw_grid(self):
        """Draw ground grid"""
        if self.ax is None:
            return
            
        try:
            grid_size = 0.3
            grid_spacing = 0.05
            
            x_grid = np.arange(-grid_size, grid_size + grid_spacing, grid_spacing)
            y_grid = np.arange(-grid_size, grid_size + grid_spacing, grid_spacing)
            
            # Draw grid lines
            for x in x_grid:
                self.ax.plot([x, x], [-grid_size, grid_size], [-0.2, -0.2], 
                            color=Colors.BORDER, alpha=0.3)
            
            for y in y_grid:
                self.ax.plot([-grid_size, grid_size], [y, y], [-0.2, -0.2], 
                            color=Colors.BORDER, alpha=0.3)
        except Exception as e:
            logger.error(f"Error drawing grid: {e}")
    
    def _draw_coordinate_frame(self):
        """Draw coordinate frame at origin"""
        if self.ax is None:
            return
            
        try:
            axis_length = 0.05
            
            # X axis (red)
            self.ax.plot([0, axis_length], [0, 0], [0, 0], color='red', linewidth=2)
            # Y axis (green)
            self.ax.plot([0, 0], [0, axis_length], [0, 0], color='green', linewidth=2)
            # Z axis (blue)
            self.ax.plot([0, 0], [0, 0], [0, axis_length], color='blue', linewidth=2)
        except Exception as e:
            logger.error(f"Error drawing coordinate frame: {e}")
    
    def _draw_robot_body(self):
        """Draw hexagonal robot body"""
        if self.ax is None:
            return
            
        try:
            # Body dimensions
            body_radius = 0.1
            body_height = 0.02
            
            # Create hexagon vertices
            angles = np.linspace(0, 2*np.pi, 7)  # 7 points to close the hexagon
            x_body = body_radius * np.cos(angles)
            y_body = body_radius * np.sin(angles)
            z_body = np.zeros_like(x_body) + body_height/2
            
            # Draw body outline
            self.ax.plot(x_body, y_body, z_body, color=Colors.TEXT_PRIMARY, linewidth=2)
            
            # Draw body top surface (simplified)
            center_x, center_y, center_z = 0, 0, body_height/2
            for i in range(6):
                self.ax.plot([center_x, x_body[i]], [center_y, y_body[i]], [center_z, z_body[i]], 
                           color=Colors.BUTTON_NORMAL, alpha=0.3)
                           
        except Exception as e:
            logger.error(f"Error drawing robot body: {e}")
    
    def _draw_robot_legs(self):
        """Draw all robot legs with star configuration"""
        if self.ax is None:
            return
            
        for leg_name in LEG_CONFIG.LEG_NAMES:
            self._draw_single_leg(leg_name)
    
    def pack(self, **kwargs):
        """Pack the frame"""
        self.frame.pack(**kwargs)
    
    def grid(self, **kwargs):
        """Grid the frame"""
        self.frame.grid(**kwargs)


class ControlPanel:
    """Control buttons and system interface panel"""
    
    def __init__(self, parent, width=300, height=400):
        self.parent = parent
        self.width = width
        self.height = height
        
        # Create main frame
        self.frame = tk.Frame(parent, bg=Colors.PANEL_BG, relief=tk.RAISED, bd=2)
        
        # Callback functions (can be set externally)
        self.movement_callbacks = {}
        self.action_callbacks = {}
        self.system_callbacks = {}
        
        self._create_control_widgets()
        
        logger.info("Control panel initialized")
    
    def _create_control_widgets(self):
        """Create all control widgets"""
        # Title
        title_label = tk.Label(self.frame, text="ROBOT CONTROL", 
                              bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY,
                              font=("Arial", 12, "bold"))
        title_label.pack(pady=5)
        
        # Movement controls
        self._create_movement_controls()
        
        # Action controls  
        self._create_action_controls()
        
        # System controls
        self._create_system_controls()
        
        # Status display
        self._create_status_display()
    
    def _create_movement_controls(self):
        """Create movement control buttons (WASD + QE)"""
        movement_frame = tk.LabelFrame(self.frame, text="Movement Controls", 
                                     bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY,
                                     font=("Arial", 10, "bold"))
        movement_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Create button style
        button_style = {
            'bg': Colors.BUTTON_NORMAL,
            'fg': Colors.TEXT_PRIMARY,
            'activebackground': Colors.BUTTON_ACTIVE,
            'activeforeground': Colors.TEXT_PRIMARY,
            'relief': tk.RAISED,
            'bd': 2,
            'font': ("Arial", 10, "bold"),
            'width': 3,
            'height': 1
        }
        
        # Top row: Q - W - E
        top_row = tk.Frame(movement_frame, bg=Colors.PANEL_BG)
        top_row.pack(pady=2)
        
        self.q_button = tk.Button(top_row, text="Q", **button_style,
                                 command=lambda: self._handle_movement('turn_left'))
        self.q_button.pack(side=tk.LEFT, padx=2)
        
        self.w_button = tk.Button(top_row, text="W", **button_style,
                                 command=lambda: self._handle_movement('forward'))
        self.w_button.pack(side=tk.LEFT, padx=2)
        
        self.e_button = tk.Button(top_row, text="E", **button_style,
                                 command=lambda: self._handle_movement('turn_right'))
        self.e_button.pack(side=tk.LEFT, padx=2)
        
        # Add button click debugging
        def debug_button_click(direction):
            print(f"🖱️ Button clicked: {direction}")
            print(f"📊 Button states check:")
            print(f"   - W button state: {self.w_button['state'] if hasattr(self, 'w_button') else 'N/A'}")
            print(f"   - A button state: {self.a_button['state'] if hasattr(self, 'a_button') else 'N/A'}")
            print(f"   - S button state: {self.s_button['state'] if hasattr(self, 's_button') else 'N/A'}")
            print(f"   - D button state: {self.d_button['state'] if hasattr(self, 'd_button') else 'N/A'}")
            print(f"   - Q button state: {self.q_button['state'] if hasattr(self, 'q_button') else 'N/A'}")
            print(f"   - E button state: {self.e_button['state'] if hasattr(self, 'e_button') else 'N/A'}")
            
            # Provide visual feedback by briefly changing button color
            button_map = {
                'forward': 'w_button', 'backward': 's_button', 
                'left': 'a_button', 'right': 'd_button',
                'turn_left': 'q_button', 'turn_right': 'e_button'
            }
            
            if direction in button_map:
                button_attr = button_map[direction]
                if hasattr(self, button_attr):
                    button = getattr(self, button_attr)
                    # Store the original color as a property of the button
                    if not hasattr(button, '_original_bg'):
                        button._original_bg = button['bg']
                    
                    # Flash bright green to show button press
                    button.config(bg='#00FF00')
                    # Reset color after 200ms using the stored original color
                    def reset_button_color(btn=button):
                        btn.config(bg=btn._original_bg)
                    # Get root window from parent widget
                    root_window = self.frame.winfo_toplevel()
                    root_window.after(200, reset_button_color)
            
            self._handle_movement(direction)
        
        # Update button commands for debugging
        self.q_button.config(command=lambda: debug_button_click('turn_left'))
        self.w_button.config(command=lambda: debug_button_click('forward'))
        self.e_button.config(command=lambda: debug_button_click('turn_right'))
        
        # Middle row: A - S - D
        middle_row = tk.Frame(movement_frame, bg=Colors.PANEL_BG)
        middle_row.pack(pady=2)
        
        self.a_button = tk.Button(middle_row, text="A", **button_style,
                                 command=lambda: self._handle_movement('left'))
        self.a_button.pack(side=tk.LEFT, padx=2)
        
        self.s_button = tk.Button(middle_row, text="S", **button_style,
                                 command=lambda: self._handle_movement('backward'))
        self.s_button.pack(side=tk.LEFT, padx=2)
        
        self.d_button = tk.Button(middle_row, text="D", **button_style,
                                 command=lambda: self._handle_movement('right'))
        self.d_button.pack(side=tk.LEFT, padx=2)
        
        # Update middle row buttons for debugging
        self.a_button.config(command=lambda: debug_button_click('left'))
        self.s_button.config(command=lambda: debug_button_click('backward'))
        self.d_button.config(command=lambda: debug_button_click('right'))
        
        # Labels
        label_frame = tk.Frame(movement_frame, bg=Colors.PANEL_BG)
        label_frame.pack(pady=2)
        
        tk.Label(label_frame, text="WASD: Move  •  QE: Turn", 
                bg=Colors.PANEL_BG, fg=Colors.TEXT_SECONDARY,
                font=("Arial", 8)).pack()
    
    def _create_action_controls(self):
        """Create action control buttons"""
        action_frame = tk.LabelFrame(self.frame, text="Actions", 
                                   bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY,
                                   font=("Arial", 10, "bold"))
        action_frame.pack(fill=tk.X, padx=10, pady=5)
        
        button_style = {
            'bg': Colors.BUTTON_NORMAL,
            'fg': Colors.TEXT_PRIMARY,
            'activebackground': Colors.BUTTON_ACTIVE,
            'activeforeground': Colors.TEXT_PRIMARY,
            'relief': tk.RAISED,
            'bd': 2,
            'font': ("Arial", 9),
            'width': 12,
            'height': 1
        }
        
        # Gait selection
        gait_frame = tk.Frame(action_frame, bg=Colors.PANEL_BG)
        gait_frame.pack(fill=tk.X, pady=2)
        
        tk.Label(gait_frame, text="Gait:", bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY,
                font=("Arial", 9)).pack(side=tk.LEFT)
        
        self.gait_var = tk.StringVar(value="tripod")
        self.gait_combo = ttk.Combobox(gait_frame, textvariable=self.gait_var,
                                      values=["tripod", "wave", "ripple"],
                                      state="readonly", width=10)
        self.gait_combo.pack(side=tk.RIGHT, padx=5)
        self.gait_combo.bind('<<ComboboxSelected>>', self._handle_gait_change)
        
        # Demo selection
        demo_frame = tk.Frame(action_frame, bg=Colors.PANEL_BG)
        demo_frame.pack(fill=tk.X, pady=2)
        
        tk.Label(demo_frame, text="Demo:", bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY,
                font=("Arial", 9)).pack(side=tk.LEFT)
        
        self.demo_var = tk.StringVar(value="basic_walk")
        self.demo_combo = ttk.Combobox(demo_frame, textvariable=self.demo_var,
                                      values=["basic_walk", "circle", "figure_eight", "dance"],
                                      state="readonly", width=10)
        self.demo_combo.pack(side=tk.RIGHT, padx=5)
        
        # Action buttons
        buttons_frame = tk.Frame(action_frame, bg=Colors.PANEL_BG)
        buttons_frame.pack(fill=tk.X, pady=5)
        
        start_style = button_style.copy()
        start_style['bg'] = Colors.ACCENT_1
        self.start_button = tk.Button(buttons_frame, text="START", **start_style,
                                     command=lambda: self._handle_action('start'))
        self.start_button.pack(side=tk.LEFT, padx=2)
        
        stop_style = button_style.copy()
        stop_style['bg'] = Colors.ACCENT_6
        self.stop_button = tk.Button(buttons_frame, text="STOP", **stop_style,
                                    command=lambda: self._handle_action('stop'))
        self.stop_button.pack(side=tk.RIGHT, padx=2)
        
        # Add action button debugging
        def debug_action_click(action):
            print(f"🖱️ Action button clicked: {action}")
            self._handle_action(action)
        
        # Update action button commands for debugging
        self.start_button.config(command=lambda: debug_action_click('start'))
        self.stop_button.config(command=lambda: debug_action_click('stop'))
        
        # Quick actions
        quick_frame = tk.Frame(action_frame, bg=Colors.PANEL_BG)
        quick_frame.pack(fill=tk.X, pady=2)
        
        self.crouch_button = tk.Button(quick_frame, text="CROUCH", **button_style,
                                      command=lambda: self._handle_action('crouch'))
        self.crouch_button.pack(side=tk.LEFT, padx=1)
        
        self.stand_button = tk.Button(quick_frame, text="STAND", **button_style,
                                     command=lambda: self._handle_action('stand'))
        self.stand_button.pack(side=tk.RIGHT, padx=1)
    
    def _create_system_controls(self):
        """Create system control buttons"""
        system_frame = tk.LabelFrame(self.frame, text="System", 
                                   bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY,
                                   font=("Arial", 10, "bold"))
        system_frame.pack(fill=tk.X, padx=10, pady=5)
        
        button_style = {
            'bg': Colors.BUTTON_NORMAL,
            'fg': Colors.TEXT_PRIMARY,
            'activebackground': Colors.BUTTON_ACTIVE,
            'activeforeground': Colors.TEXT_PRIMARY,
            'relief': tk.RAISED,
            'bd': 2,
            'font': ("Arial", 9),
            'width': 12,
            'height': 1
        }
        
        # System buttons
        sys_buttons_frame = tk.Frame(system_frame, bg=Colors.PANEL_BG)
        sys_buttons_frame.pack(fill=tk.X, pady=2)
        
        self.reset_button = tk.Button(sys_buttons_frame, text="RESET", **button_style,
                                     command=lambda: self._handle_system('reset'))
        self.reset_button.pack(side=tk.LEFT, padx=1)
        
        self.settings_button = tk.Button(sys_buttons_frame, text="SETTINGS", **button_style,
                                        command=lambda: self._handle_system('settings'))
        self.settings_button.pack(side=tk.RIGHT, padx=1)
        
        # Add system button debugging
        def debug_system_click(command):
            print(f"🖱️ System button clicked: {command}")
            self._handle_system(command)
        
        # Update system button commands for debugging
        self.reset_button.config(command=lambda: debug_system_click('reset'))
        self.settings_button.config(command=lambda: debug_system_click('settings'))
    
    def _create_status_display(self):
        """Create status display area"""
        status_frame = tk.LabelFrame(self.frame, text="Status", 
                                   bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY,
                                   font=("Arial", 10, "bold"))
        status_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Status text
        self.status_text = tk.Text(status_frame, bg=Colors.BACKGROUND, fg=Colors.TEXT_PRIMARY,
                                  font=("Courier", 8), height=8, width=30,
                                  relief=tk.SUNKEN, bd=1)
        self.status_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Scrollbar
        scrollbar = tk.Scrollbar(status_frame, command=self.status_text.yview)
        self.status_text.config(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Initialize status
        self.update_status("System initialized\nReady for operation\n")
    
    def _handle_movement(self, direction: str):
        """Handle movement button press"""
        print(f"🎛️ ControlPanel._handle_movement called with direction: {direction}")
        if direction in self.movement_callbacks:
            print(f"✅ Found callback for {direction}, executing...")
            self.movement_callbacks[direction]()
            # Show visual feedback
            self.update_status(f"🎮 MOVING {direction.upper()}\n")
        else:
            print(f"❌ No callback found for direction: {direction}")
            print(f"Available callbacks: {list(self.movement_callbacks.keys())}")
            self.update_status(f"❌ No callback for movement: {direction}\n")
    
    def _handle_action(self, action: str):
        """Handle action button press"""
        print(f"🎬 Action button pressed: {action}")
        
        # Provide visual feedback
        button_map = {'start': 'start_button', 'stop': 'stop_button'}
        if action in button_map and hasattr(self, button_map[action]):
            button = getattr(self, button_map[action])
            if not hasattr(button, '_original_bg'):
                button._original_bg = button['bg']
            button.config(bg='#00FF00')  # Flash green
            def reset_action_button_color(btn=button):
                btn.config(bg=btn._original_bg)
            # Get root window from parent widget
            root_window = self.frame.winfo_toplevel()
            root_window.after(200, reset_action_button_color)
        
        if action in self.action_callbacks:
            self.action_callbacks[action]()
            # Show visual feedback
            self.update_status(f"✅ {action.upper()} executed successfully\n")
        else:
            self.update_status(f"❌ No callback for action: {action}\n")
        
    def _handle_system(self, command: str):
        """Handle system command"""
        print(f"⚙️ System button pressed: {command}")
        
        # Provide visual feedback
        button_map = {'reset': 'reset_button', 'settings': 'settings_button'}
        if command in button_map and hasattr(self, button_map[command]):
            button = getattr(self, button_map[command])
            if not hasattr(button, '_original_bg'):
                button._original_bg = button['bg']
            button.config(bg='#FF6600')  # Flash orange for system commands
            def reset_system_button_color(btn=button):
                btn.config(bg=btn._original_bg)
            # Get root window from parent widget
            root_window = self.frame.winfo_toplevel()
            root_window.after(200, reset_system_button_color)
        
        if command in self.system_callbacks:
            self.system_callbacks[command]()
            # Show visual feedback  
            self.update_status(f"✅ {command.upper()} executed successfully\n")
        else:
            self.update_status(f"❌ No callback for system: {command}\n")
    
    def _handle_gait_change(self, event):
        """Handle gait selection change"""
        gait = self.gait_var.get()
        if 'gait_change' in self.action_callbacks:
            self.action_callbacks['gait_change'](gait)
        self.update_status(f"Gait changed to: {gait}\n")
    
    def update_status(self, message: str):
        """Update status display"""
        self.status_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}")
        self.status_text.see(tk.END)
        
    def reset_all_button_colors(self):
        """Reset all buttons to their original colors"""
        button_names = ['w_button', 'a_button', 's_button', 'd_button', 'q_button', 'e_button',
                       'start_button', 'stop_button', 'reset_button', 'settings_button']
        
        for button_name in button_names:
            if hasattr(self, button_name):
                button = getattr(self, button_name)
                if hasattr(button, '_original_bg'):
                    button.config(bg=button._original_bg)
                else:
                    # Fallback to default button color
                    button.config(bg=Colors.BUTTON_NORMAL)
        
        # Limit text length
        if int(self.status_text.index('end-1c').split('.')[0]) > 100:
            self.status_text.delete('1.0', '10.0')
    
    def set_callbacks(self, movement_callbacks=None, action_callbacks=None, system_callbacks=None):
        """Set callback functions for button presses"""
        if movement_callbacks:
            self.movement_callbacks.update(movement_callbacks)
        if action_callbacks:
            self.action_callbacks.update(action_callbacks)
        if system_callbacks:
            self.system_callbacks.update(system_callbacks)
    
    def pack(self, **kwargs):
        """Pack the frame"""
        self.frame.pack(**kwargs)
    
    def grid(self, **kwargs):
        """Grid the frame"""
        self.frame.grid(**kwargs)


class JointAnglePanel:
    """Panel for displaying joint angles and positions"""
    
    def __init__(self, parent, width=400, height=300):
        self.parent = parent
        self.width = width
        self.height = height
        
        # Create frame
        self.frame = tk.Frame(parent, bg=Colors.PANEL_BG, relief=tk.RAISED, bd=2)
        
        # Joint angle displays
        self.angle_labels = {}
        self.position_labels = {}
        
        self._create_joint_display()
        
        logger.info("Joint angle panel initialized")
    
    def _create_joint_display(self):
        """Create joint angle display widgets"""
        # Title
        title_label = tk.Label(self.frame, text="JOINT ANGLES (°)", 
                              bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY,
                              font=("Arial", 12, "bold"))
        title_label.pack(pady=5)
        
        # Create two columns: Left legs and Right legs
        columns_frame = tk.Frame(self.frame, bg=Colors.PANEL_BG)
        columns_frame.pack(fill=tk.BOTH, expand=True, padx=10)
        
        # Left legs column
        left_frame = tk.LabelFrame(columns_frame, text="LEFT LEGS", 
                                 bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY,
                                 font=("Arial", 10, "bold"))
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        # Right legs column
        right_frame = tk.LabelFrame(columns_frame, text="RIGHT LEGS", 
                                  bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY,
                                  font=("Arial", 10, "bold"))
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        
        # Create displays for each leg
        left_legs = ['L1', 'L2', 'L3']
        right_legs = ['R1', 'R2', 'R3']
        
        for leg in left_legs:
            self._create_leg_display(left_frame, leg)
        
        for leg in right_legs:
            self._create_leg_display(right_frame, leg)
        
        # Position display
        self._create_position_display()
    
    def _create_leg_display(self, parent, leg_name: str):
        """Create display for a single leg"""
        leg_frame = tk.Frame(parent, bg=Colors.PANEL_BG, relief=tk.GROOVE, bd=1)
        leg_frame.pack(fill=tk.X, pady=2, padx=5)
        
        # Leg name with color
        leg_color = LEG_CONFIG.LEG_COLORS[leg_name]
        leg_label = tk.Label(leg_frame, text=leg_name, 
                            bg=Colors.PANEL_BG, fg=leg_color,
                            font=("Arial", 10, "bold"))
        leg_label.pack(side=tk.LEFT)
        
        # Joint angle labels
        angles_frame = tk.Frame(leg_frame, bg=Colors.PANEL_BG)
        angles_frame.pack(side=tk.RIGHT, fill=tk.X, expand=True)
        
        angle_text = tk.Label(angles_frame, text="C:0° F:0° T:0°", 
                             bg=Colors.PANEL_BG, fg=Colors.TEXT_SECONDARY,
                             font=("Courier", 9))
        angle_text.pack()
        
        self.angle_labels[leg_name] = angle_text
    
    def _create_position_display(self):
        """Create position and orientation display"""
        pos_frame = tk.LabelFrame(self.frame, text="ROBOT POSITION", 
                                bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY,
                                font=("Arial", 10, "bold"))
        pos_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Position
        self.position_label = tk.Label(pos_frame, text="X:0.00m Y:0.00m Z:0.15m", 
                                      bg=Colors.PANEL_BG, fg=Colors.TEXT_SECONDARY,
                                      font=("Courier", 10))
        self.position_label.pack()
        
        # Orientation
        self.orientation_label = tk.Label(pos_frame, text="Orientation: 0.0°", 
                                         bg=Colors.PANEL_BG, fg=Colors.TEXT_SECONDARY,
                                         font=("Courier", 10))
        self.orientation_label.pack()
        
        # Velocity
        self.velocity_label = tk.Label(pos_frame, text="Velocity: 0.0 m/s", 
                                      bg=Colors.PANEL_BG, fg=Colors.TEXT_SECONDARY,
                                      font=("Courier", 10))
        self.velocity_label.pack()
    
    def update_joint_angles(self, joint_angles: Dict[str, Tuple[float, float, float]]):
        """Update joint angle displays"""
        for leg_name, (coxa, femur, tibia) in joint_angles.items():
            if leg_name in self.angle_labels:
                angle_text = f"C:{coxa:+4.0f}° F:{femur:+4.0f}° T:{tibia:+4.0f}°"
                self.angle_labels[leg_name].config(text=angle_text)
    
    def update_position(self, position: Tuple[float, float, float], 
                       orientation: float, velocity: Tuple[float, float]):
        """Update position and orientation displays"""
        x, y, z = position
        vx, vy = velocity
        v_mag = np.sqrt(vx**2 + vy**2)
        
        pos_text = f"X:{x:+5.2f}m Y:{y:+5.2f}m Z:{z:+5.2f}m"
        self.position_label.config(text=pos_text)
        
        orient_text = f"Orientation: {orientation:+6.1f}°"
        self.orientation_label.config(text=orient_text)
        
        vel_text = f"Velocity: {v_mag:4.2f} m/s"
        self.velocity_label.config(text=vel_text)
    
    def pack(self, **kwargs):
        """Pack the frame"""
        self.frame.pack(**kwargs)
    
    def grid(self, **kwargs):
        """Grid the frame"""
        self.frame.grid(**kwargs)


class HexaPodSimGUI:
    """Main GUI application for HexaPodSim 2.0"""
    
    def __init__(self):
        # Initialize main window with error handling
        try:
            self.root = tk.Tk()
            self.root.title("🤖 HexaPodSim 2.0 - Hexapod Robot Simulation")
            self.root.configure(bg=Colors.BACKGROUND)
            self.root.geometry("1400x900")
            
            # Set window icon and properties
            self.root.resizable(True, True)
            
            # Force window to front and focus
            self.root.lift()
            self.root.attributes('-topmost', True)
            self.root.after_idle(lambda: self.root.attributes('-topmost', False))
            
            logger.info("Main window created successfully")
            
        except Exception as e:
            logger.error(f"Failed to create main window: {e}")
            raise
        
        # Initialize robot systems
        self.robot_state = RobotState()
        self.is_running = False
        self.simulation_thread = None
        
        # Initialize robot components
        try:
            logger.info("Initializing robot systems...")
            from .kinematics import HexapodKinematics
            from .gait import GaitGenerator, GaitType
            from .motion import MotionController
            
            logger.info("Creating kinematics system...")
            self.kinematics = HexapodKinematics()
            
            logger.info("Creating gait generator...")
            self.gait_generator = GaitGenerator(self.kinematics, GaitType.TRIPOD)
            
            logger.info("Creating motion controller...")
            self.motion_controller = MotionController(self.kinematics)
            
            # Start motion controller
            logger.info("Starting motion controller...")
            self.motion_controller.start()
            
            logger.info("Robot systems initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize robot systems: {e}")
            import traceback
            logger.error(f"Full traceback: {traceback.format_exc()}")
            # Continue with limited functionality
            self.kinematics = None
            self.gait_generator = None
            self.motion_controller = None
        
        # Create GUI panels
        try:
            logger.info("Creating GUI layout...")
            self._create_gui_layout()
            logger.info("GUI layout created successfully")
        except Exception as e:
            logger.error(f"Failed to create GUI layout: {e}")
            import traceback
            logger.error(f"Full traceback: {traceback.format_exc()}")
            raise
        
        # Set up callbacks
        try:
            logger.info("Setting up callbacks...")
            self._setup_callbacks()
            logger.info("Callbacks set up successfully")
        except Exception as e:
            logger.error(f"Failed to set up callbacks: {e}")
            # Continue without callbacks
        
        # Start update loop
        try:
            logger.info("Starting update loop...")
            self._start_update_loop()
            logger.info("Update loop started successfully")
        except Exception as e:
            logger.error(f"Failed to start update loop: {e}")
            # Continue without updates
        
        logger.info("HexaPodSim 2.0 GUI initialized successfully")
    
    def _create_gui_layout(self):
        """Create the main GUI layout"""
        # Main container with better layout management
        main_frame = tk.Frame(self.root, bg=Colors.BACKGROUND)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Title section
        title_label = tk.Label(
            main_frame,
            text="🤖 HexaPodSim 2.0 - Hexapod Robot Simulation",
            font=('Consolas', 16, 'bold'),
            fg=Colors.ACCENT_1,
            bg=Colors.BACKGROUND
        )
        title_label.pack(pady=(0, 10))
        
        # Control panel (most important - make it prominent)
        logger.info("Creating control panel...")
        self.control_panel = ControlPanel(main_frame, width=1200, height=300)
        self.control_panel.pack(fill=tk.X, pady=10)
        
        # Secondary panels container
        panels_frame = tk.Frame(main_frame, bg=Colors.BACKGROUND)
        panels_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Left side: Gait and joint displays
        left_panel = tk.Frame(panels_frame, bg=Colors.BACKGROUND)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # Gait visualization
        logger.info("Creating gait visualization panel...")
        try:
            self.gait_panel = GaitVisualizationPanel(left_panel, width=600, height=200)
            self.gait_panel.pack(fill=tk.X, pady=(0, 5))
        except Exception as e:
            logger.error(f"Failed to create gait panel: {e}")
            # Create fallback
            fallback_gait = tk.Label(left_panel, text="Gait Visualization (Loading...)", 
                                   bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY, height=8)
            fallback_gait.pack(fill=tk.X, pady=(0, 5))
        
        # Joint angles panel
        logger.info("Creating joint angle panel...")
        try:
            self.joint_panel = JointAnglePanel(left_panel, width=600, height=250)
            self.joint_panel.pack(fill=tk.BOTH, expand=True)
        except Exception as e:
            logger.error(f"Failed to create joint panel: {e}")
            # Create fallback
            fallback_joint = tk.Label(left_panel, text="Joint Angles (Loading...)", 
                                    bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY, height=10)
            fallback_joint.pack(fill=tk.BOTH, expand=True)
        
        # Right side: 3D visualization
        right_panel = tk.Frame(panels_frame, bg=Colors.BACKGROUND)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        logger.info("Creating 3D robot visualization...")
        try:
            self.robot_3d = Robot3DVisualization(right_panel, width=400, height=500)
            self.robot_3d.pack(fill=tk.BOTH, expand=True)
        except Exception as e:
            logger.error(f"Failed to create 3D visualization: {e}")
            # Create fallback
            fallback_3d = tk.Label(right_panel, text="3D Robot View (Loading...)", 
                                 bg=Colors.PANEL_BG, fg=Colors.TEXT_PRIMARY, width=50, height=25)
            fallback_3d.pack(fill=tk.BOTH, expand=True)
    
    def _setup_callbacks(self):
        """Set up callback functions for controls"""
        movement_callbacks = {
            'forward': lambda: self._handle_movement(1.0, 0.0, 0.0),
            'backward': lambda: self._handle_movement(-1.0, 0.0, 0.0),
            'left': lambda: self._handle_movement(0.0, 1.0, 0.0),
            'right': lambda: self._handle_movement(0.0, -1.0, 0.0),
            'turn_left': lambda: self._handle_movement(0.0, 0.0, 30.0),
            'turn_right': lambda: self._handle_movement(0.0, 0.0, -30.0)
        }
        
        action_callbacks = {
            'start': self._start_simulation,
            'stop': self._stop_simulation,
            'crouch': self._crouch_robot,
            'stand': self._stand_robot,
            'gait_change': self._change_gait
        }
        
        system_callbacks = {
            'reset': self._reset_robot,
            'settings': self._open_settings
        }
        
        self.control_panel.set_callbacks(movement_callbacks, action_callbacks, system_callbacks)
    
    def _handle_movement(self, linear_x: float, linear_y: float, angular_z: float):
        """Handle movement commands"""
        if self.motion_controller and self.gait_generator:
            try:
                # Scale velocities to reasonable ranges
                max_linear = 0.2  # m/s
                max_angular = 30.0  # deg/s
                
                vx = linear_x * max_linear
                vy = linear_y * max_linear
                wz = angular_z * max_angular
                
                # Set velocity on gait generator
                self.gait_generator.set_velocity([vx, vy, 0.0], wz * np.pi / 180.0)
                
                # Update robot state
                self.robot_state.velocity = (vx, vy)
                self.robot_state.angular_velocity = wz
                
                # Start simulation if movement commanded
                if not self.is_running and (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.1):
                    self._start_simulation()
                
                # Send movement command to motion controller
                if abs(vx) > 0.01:
                    if vx > 0:
                        success = self.motion_controller.walk_forward(abs(vx))
                    else:
                        success = self.motion_controller.walk_backward(abs(vx))
                elif abs(wz) > 0.1:
                    if wz > 0:
                        success = self.motion_controller.turn_left(abs(wz) * np.pi / 180.0)
                    else:
                        success = self.motion_controller.turn_right(abs(wz) * np.pi / 180.0)
                else:
                    success = self.motion_controller.stop_motion()
                
                if success:
                    self.control_panel.update_status(f"Movement: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.1f}°/s\\n")
                
            except Exception as e:
                logger.error(f"Movement error: {e}")
                self.control_panel.update_status(f"Movement error: {e}\\n")
    
    def _start_simulation(self):
        """Start robot simulation"""
        if not self.is_running:
            self.is_running = True
            self.robot_state.status = "RUNNING"
            self.robot_state.is_moving = True
            self.control_panel.update_status("Simulation started\n")
            logger.info("Simulation started")
    
    def _stop_simulation(self):
        """Stop robot simulation"""
        if self.is_running:
            self.is_running = False
            self.robot_state.status = "STOPPED"
            self.robot_state.is_moving = False
            self.robot_state.velocity = (0.0, 0.0)
            self.robot_state.angular_velocity = 0.0
            self.control_panel.update_status("Simulation stopped\n")
            logger.info("Simulation stopped")
    
    def _change_gait(self, gait_type: str):
        """Change gait pattern"""
        if self.gait_generator and self.motion_controller:
            try:
                from .gait import GaitType
                
                # Map gait names to GaitType enum
                gait_map = {
                    'tripod': GaitType.TRIPOD,
                    'wave': GaitType.WAVE,
                    'ripple': GaitType.RIPPLE
                }
                
                if gait_type in gait_map:
                    # Set gait on motion controller
                    success = self.motion_controller.set_gait_type(gait_map[gait_type])
                    
                    if success:
                        self.robot_state.gait_type = gait_type
                        self.control_panel.update_status(f"Gait changed to {gait_type}\\n")
                        logger.info(f"Gait changed to {gait_type}")
                    else:
                        self.control_panel.update_status(f"Failed to change gait to {gait_type}\\n")
                
            except Exception as e:
                logger.error(f"Gait change error: {e}")
                self.control_panel.update_status(f"Gait change error: {e}\\n")
    
    def _crouch_robot(self):
        """Make robot crouch"""
        self.robot_state.position = (self.robot_state.position[0], 
                                   self.robot_state.position[1], 
                                   0.08)  # Lower body
        self.control_panel.update_status("Robot crouched\n")
    
    def _stand_robot(self):
        """Make robot stand"""
        self.robot_state.position = (self.robot_state.position[0], 
                                   self.robot_state.position[1], 
                                   0.15)  # Normal height
        self.control_panel.update_status("Robot standing\n")
    
    def _reset_robot(self):
        """Reset robot to initial state"""
        self._stop_simulation()
        self.robot_state = RobotState()
        self.control_panel.update_status("Robot reset to initial state\n")
    
    def _open_settings(self):
        """Open settings dialog"""
        self.control_panel.update_status("Settings dialog not implemented yet\n")
    
    def _start_update_loop(self):
        """Start the GUI update loop"""
        self._update_displays()
        self.root.after(50, self._start_update_loop)  # 20 FPS update rate
    
    def _update_displays(self):
        """Update all display panels"""
        try:
            # Update robot state with real simulation data
            if self.motion_controller:
                # Get current robot pose from motion controller
                if hasattr(self.motion_controller, 'get_current_pose'):
                    current_pose = self.motion_controller.get_current_pose()
                    if current_pose:
                        self.robot_state.position = current_pose.get('position', self.robot_state.position)
                        self.robot_state.orientation = current_pose.get('orientation', self.robot_state.orientation)
                
                # Get current motion state
                if hasattr(self.motion_controller, 'get_motion_state'):
                    motion_state = self.motion_controller.get_motion_state()
                    if motion_state:
                        self.robot_state.linear_velocity = motion_state.get('linear_velocity', (0, 0, 0))
                        self.robot_state.angular_velocity = motion_state.get('angular_velocity', (0, 0, 0))
            
            # Update gait visualization
            if self.is_running and self.gait_generator:
                # Update gait phase
                self.robot_state.gait_phase = (self.robot_state.gait_phase + 2.0) % 100.0
                
            # Update joint angles from real kinematics
            self._update_joint_angles()
            
            # Update all display panels
            self._update_joint_displays()
            self._update_3d_display()
            self._update_gait_display()
            
            # Get current leg states for gait visualization
            if self.is_running and self.gait_generator:
                leg_states = self._get_current_leg_states()
                
                self.gait_panel.update_gait_pattern(
                    self.robot_state.gait_type,
                    self.robot_state.gait_phase,
                    leg_states
                )
            
        except Exception as e:
            logger.error(f"Display update error: {e}")
            self.control_panel.update_status(f"Display update error: {e}\n")
    
    def _update_joint_displays(self):
        """Update joint angle displays"""
        try:
            self.joint_panel.update_joint_angles(self.robot_state.joint_angles)
            self.joint_panel.update_position(
                self.robot_state.position,
                self.robot_state.orientation,
                self.robot_state.velocity
            )
        except Exception as e:
            logger.error(f"Joint display update error: {e}")
    
    def _update_3d_display(self):
        """Update 3D robot display"""
        try:
            self.robot_3d.update_robot_display(self.robot_state)
        except Exception as e:
            logger.error(f"3D display update error: {e}")
    
    def _update_gait_display(self):
        """Update gait pattern display"""
        try:
            if self.is_running and self.gait_generator:
                leg_states = self._get_current_leg_states()
                self.gait_panel.update_gait_pattern(
                    self.robot_state.gait_type,
                    self.robot_state.gait_phase,
                    leg_states
                )
        except Exception as e:
            logger.error(f"Gait display update error: {e}")
    
    def _update_system_status(self):
        """Update system status display"""
        try:
            # Update system monitoring
            if hasattr(self, 'control_panel'):
                status_text = f"Robot Status: {'Running' if self.is_running else 'Stopped'}\n"
                status_text += f"Gait: {self.robot_state.gait_type}\n"
                status_text += f"Phase: {self.robot_state.gait_phase:.1f}%\n"
                if self.motion_controller:
                    status_text += "Motion Controller: Active\n"
                else:
                    status_text += "Motion Controller: Inactive\n"
                self.control_panel.update_status(status_text)
        except Exception as e:
            logger.error(f"Status update error: {e}")
            if self.is_running:
                self.robot_state.cpu_load = np.random.uniform(40, 80)
                self.robot_state.memory_usage = np.random.uniform(30, 60)
                self.robot_state.battery_level = max(0, self.robot_state.battery_level - 0.01)
            
        except Exception as e:
            logger.error(f"Display update error: {e}")
    
    def _get_current_leg_states(self) -> Dict[str, bool]:
        """Get current leg states for gait visualization"""
        # Simplified gait state calculation
        phase = self.robot_state.gait_phase / 100.0
        
        if self.robot_state.gait_type == "tripod":
            # Tripod gait: L1,R2,L3 vs R1,L2,R3
            return {
                'L1': phase < 0.5,
                'R1': phase >= 0.5,
                'L2': phase >= 0.5,
                'R2': phase < 0.5,
                'L3': phase < 0.5,
                'R3': phase >= 0.5
            }
        elif self.robot_state.gait_type == "wave":
            # Wave gait: sequential leg movement
            wave_phase = (phase * 6) % 6
            return {
                'L1': 0 <= wave_phase < 1,
                'L2': 1 <= wave_phase < 2,
                'L3': 2 <= wave_phase < 3,
                'R3': 3 <= wave_phase < 4,
                'R2': 4 <= wave_phase < 5,
                'R1': 5 <= wave_phase < 6
            }
        else:  # ripple
            return {leg: np.random.random() > 0.7 for leg in LEG_CONFIG.LEG_NAMES}
    
    def _update_joint_angles(self):
        """Update joint angles using real kinematics"""
        if self.gait_generator and self.kinematics:
            try:
                # Update gait generator
                dt = 0.05  # 20 FPS update rate
                leg_positions = self.gait_generator.update(dt)
                
                # Calculate joint angles for each leg using inverse kinematics
                for i, leg_name in enumerate(['L1', 'R1', 'L2', 'R2', 'L3', 'R3']):
                    if i < len(leg_positions):
                        # Get target position for this leg
                        target_pos = leg_positions[i]
                        
                        try:
                            # Calculate joint angles using inverse kinematics
                            joint_angles, success = self.kinematics.inverse_kinematics(target_pos, leg_name)
                            
                            if success:
                                # Convert to degrees and store
                                joint_angles_deg = np.degrees(joint_angles)
                                self.robot_state.joint_angles[leg_name] = tuple(joint_angles_deg)
                            else:
                                # Keep previous angles if IK fails
                                if leg_name not in self.robot_state.joint_angles:
                                    self.robot_state.joint_angles[leg_name] = (0.0, -20.0, 40.0)
                                    
                        except Exception as e:
                            # Fallback to default angles
                            if leg_name not in self.robot_state.joint_angles:
                                self.robot_state.joint_angles[leg_name] = (0.0, -20.0, 40.0)
                
            except Exception as e:
                logger.error(f"Joint angle update error: {e}")
                # Fallback to animated angles
                self._update_animated_joint_angles()
        else:
            # Fallback to animated angles when no real kinematics
            self._update_animated_joint_angles()
    
    def _update_animated_joint_angles(self):
        """Update joint angles with simple animation (fallback)"""
        import time
        
        # Generate test joint angles for animation
        time_factor = time.time() * 2  # Animation speed
        
        for i, leg in enumerate(['L1', 'R1', 'L2', 'R2', 'L3', 'R3']):
            phase_offset = i * np.pi / 3  # Phase offset for each leg
            
            # Generate sinusoidal joint movements
            coxa_angle = 15 * np.sin(time_factor + phase_offset)
            femur_angle = -20 + 10 * np.sin(time_factor * 2 + phase_offset)
            tibia_angle = 30 + 15 * np.sin(time_factor * 1.5 + phase_offset)
            
            self.robot_state.joint_angles[leg] = (coxa_angle, femur_angle, tibia_angle)
    
    def run(self):
        """Start the GUI application"""
        try:
            logger.info("Starting GUI mainloop...")
            
            # Make sure window is visible and properly sized
            self.root.deiconify()  # Make sure window is not minimized
            self.root.lift()       # Bring to front
            self.root.focus_force() # Force focus
            
            # Force geometry update
            self.root.update_idletasks()
            self.root.geometry("1400x900")  # Ensure size is set
            
            # Update the display before starting mainloop
            self.root.update()
            
            logger.info("GUI window should now be visible with controls")
            logger.info("Window geometry: 1400x900")
            logger.info("Starting tkinter mainloop...")
            
            self.root.mainloop()
            
            logger.info("GUI mainloop ended")
            
        except KeyboardInterrupt:
            logger.info("GUI application interrupted by user")
        except Exception as e:
            logger.error(f"GUI application error: {e}")
            import traceback
            logger.error(f"Full traceback: {traceback.format_exc()}")
        finally:
            self._cleanup()
    
    def _cleanup(self):
        """Clean up resources"""
        self.is_running = False
        if self.simulation_thread:
            self.simulation_thread.join()
        logger.info("GUI application cleaned up")


if __name__ == "__main__":
    # Test the GUI components
    print("Testing HexaPodSim 2.0 GUI Components...")
    
    root = tk.Tk()
    root.title("HexaPodSim 2.0 - Component Test")
    root.configure(bg=Colors.BACKGROUND)
    root.geometry("1200x800")
    
    # Test gait visualization
    gait_panel = GaitVisualizationPanel(root, width=600, height=200)
    gait_panel.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)
    
    # Test 3D visualization
    robot_3d = Robot3DVisualization(root, width=400, height=400)
    robot_3d.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=5)
    
    # Initialize with test data
    test_state = RobotState()
    test_state.joint_angles = {
        'L1': (15.0, -20.0, 45.0),
        'R1': (-15.0, -20.0, 45.0),
        'L2': (90.0, -15.0, 30.0),
        'R2': (-90.0, -15.0, 30.0),
        'L3': (165.0, -10.0, 25.0),
        'R3': (-165.0, -10.0, 25.0)
    }
    
    # Update displays with test data
    gait_panel.update_gait_pattern("tripod", 25.0, {leg: False for leg in LEG_CONFIG.LEG_NAMES})
    robot_3d.update_robot_display(test_state)
    
    print("GUI components loaded. Close window to exit.")
    root.mainloop()