"""
Gait Generation System for Hexapod Robot

This module implements comprehensive gait pattern generation for hexapod locomotion,
including tripod, wave, and ripple gaits with configurable parameters and real-time switching.

Gait Patterns Implemented:
- Tripod Gait: Fast locomotion (50% duty factor)
- Wave Gait: Maximum stability (83% duty factor)  
- Ripple Gait: Balanced performance (67% duty factor)
- Custom gaits with configurable timing

Key Features:
- Real-time gait switching
- Configurable step height and stride length
- Phase offset calculation for each leg
- Smooth transitions between stance and swing phases
- Support for variable speed and direction

Author: Hexapod Simulation Team
Date: October 2024
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any
from enum import Enum
import time
import math

from .kinematics import HexapodKinematics


class GaitType(Enum):
    """Supported gait patterns"""
    TRIPOD = "tripod"
    WAVE = "wave"
    RIPPLE = "ripple"
    STATIONARY = "stationary"
    CUSTOM = "custom"


class LegPhase(Enum):
    """Leg movement phases"""
    STANCE = "stance"    # Leg on ground, supporting body
    SWING = "swing"      # Leg in air, moving to next position
    TRANSITION = "transition"  # Transitioning between phases


@dataclass
class GaitParameters:
    """Gait configuration parameters"""
    # Basic gait properties
    cycle_time: float = 2.0          # Total gait cycle time (seconds)
    duty_factor: float = 0.5         # Fraction of cycle in stance phase
    step_height: float = 0.03        # Maximum foot lift height (meters)
    stride_length: float = 0.08      # Forward step distance (meters)
    
    # Movement parameters
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [vx, vy, vz] m/s
    angular_velocity: float = 0.0    # Body rotation rate (rad/s)
    
    # Leg sequencing (phase offsets for each leg 0-5)
    phase_offsets: np.ndarray = field(default_factory=lambda: np.zeros(6))
    
    # Transition parameters
    transition_time: float = 0.1     # Time for smooth transitions (seconds)
    lift_velocity: float = 0.2       # Foot lift/lower velocity (m/s)
    
    def __post_init__(self):
        """Validate and initialize default values"""
        if len(self.velocity) != 3:
            self.velocity = np.array([0.0, 0.0, 0.0])
        if len(self.phase_offsets) != 6:
            self.phase_offsets = np.zeros(6)


@dataclass
class LegTrajectory:
    """Trajectory data for a single leg"""
    # Current state
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    phase: LegPhase = LegPhase.STANCE
    phase_time: float = 0.0          # Time within current phase
    
    # Target positions
    stance_start: np.ndarray = field(default_factory=lambda: np.zeros(3))
    stance_end: np.ndarray = field(default_factory=lambda: np.zeros(3))
    swing_start: np.ndarray = field(default_factory=lambda: np.zeros(3))
    swing_end: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    # Trajectory parameters
    is_support_leg: bool = True      # Currently supporting body weight
    ground_clearance: float = 0.0    # Current height above ground
    contact_force: float = 0.0       # Estimated ground contact force


class GaitGenerator:
    """Main gait generation system"""
    
    def __init__(self, kinematics: HexapodKinematics, gait_type: GaitType = GaitType.TRIPOD):
        """
        Initialize gait generator
        
        Args:
            kinematics: Hexapod kinematics system
            gait_type: Initial gait pattern
        """
        self.kinematics = kinematics
        self.current_gait = gait_type
        self.parameters = self._get_default_parameters(gait_type)
        
        # Gait state
        self.cycle_time = 0.0           # Current time in gait cycle
        self.is_active = False          # Gait generation active
        self.is_transitioning = False   # Currently transitioning between gaits
        
        # Leg trajectories
        self.leg_trajectories = [LegTrajectory() for _ in range(6)]
        
        # Default foot positions (standing configuration)
        self.default_positions = self._calculate_default_positions()
        
        # Performance tracking
        self.computation_times = []
        self.last_update_time = time.perf_counter()
        
    def _get_default_parameters(self, gait_type: GaitType) -> GaitParameters:
        """Get default parameters for specific gait type"""
        if gait_type == GaitType.TRIPOD:
            return GaitParameters(
                cycle_time=1.0,
                duty_factor=0.5,
                step_height=0.04,
                stride_length=0.10,
                phase_offsets=np.array([0.0, 0.5, 0.0, 0.5, 0.0, 0.5])  # Alternating tripods
            )
        elif gait_type == GaitType.WAVE:
            return GaitParameters(
                cycle_time=3.0,
                duty_factor=0.83,  # 5/6 of cycle in stance
                step_height=0.03,
                stride_length=0.06,
                phase_offsets=np.array([0.0, 1/6, 2/6, 3/6, 4/6, 5/6])  # Sequential wave
            )
        elif gait_type == GaitType.RIPPLE:
            return GaitParameters(
                cycle_time=2.0,
                duty_factor=0.67,  # 2/3 of cycle in stance
                step_height=0.035,
                stride_length=0.08,
                phase_offsets=np.array([0.0, 2/3, 1/3, 0.0, 2/3, 1/3])  # Ripple pattern
            )
        else:  # STATIONARY or CUSTOM
            return GaitParameters(
                cycle_time=1.0,
                duty_factor=1.0,  # Always in stance
                step_height=0.0,
                stride_length=0.0,
                phase_offsets=np.zeros(6)
            )
            
    def _calculate_default_positions(self) -> List[np.ndarray]:
        """Calculate default foot positions for standing pose"""
        default_positions = []
        
        # Use a comfortable standing pose
        default_angles = np.array([
            [0.0, -0.3, 0.8],  # L1
            [0.0, -0.3, 0.8],  # R1
            [0.0, -0.3, 0.8],  # L2
            [0.0, -0.3, 0.8],  # R2
            [0.0, -0.3, 0.8],  # L3
            [0.0, -0.3, 0.8]   # R3
        ])
        
        for i, leg_name in enumerate(self.kinematics.LEG_NAMES):
            joint_angles_deg = np.degrees(default_angles[i])
            foot_pos = self.kinematics.forward_kinematics(joint_angles_deg, leg_name)
            default_positions.append(foot_pos)
            
        return default_positions
        
    def set_gait_type(self, gait_type: GaitType, smooth_transition: bool = True):
        """
        Switch to a new gait pattern
        
        Args:
            gait_type: New gait pattern to use
            smooth_transition: Whether to transition smoothly
        """
        if gait_type == self.current_gait:
            return
            
        old_gait = self.current_gait
        self.current_gait = gait_type
        self.parameters = self._get_default_parameters(gait_type)
        
        if smooth_transition and self.is_active:
            self._initiate_transition(old_gait, gait_type)
        else:
            self._reset_gait_state()
            
    def _initiate_transition(self, old_gait: GaitType, new_gait: GaitType):
        """Initiate smooth transition between gaits"""
        self.is_transitioning = True
        
        # Calculate transition timing to minimize disruption
        # For now, implement immediate transition
        # TODO: Implement sophisticated transition planning
        self._reset_gait_state()
        self.is_transitioning = False
        
    def _reset_gait_state(self):
        """Reset gait state for new pattern"""
        self.cycle_time = 0.0
        
        # Reset all legs to stance phase initially
        for i, trajectory in enumerate(self.leg_trajectories):
            trajectory.phase = LegPhase.STANCE
            trajectory.phase_time = 0.0
            trajectory.position = self.default_positions[i].copy()
            trajectory.velocity = np.zeros(3)
            trajectory.is_support_leg = True
            
    def start_gait(self):
        """Start gait generation"""
        if not self.is_active:
            self.is_active = True
            self._reset_gait_state()
            self.last_update_time = time.perf_counter()
            
    def stop_gait(self):
        """Stop gait generation and return to default pose"""
        self.is_active = False
        self.is_transitioning = False
        
        # Move all legs back to default positions
        for i, trajectory in enumerate(self.leg_trajectories):
            trajectory.phase = LegPhase.STANCE
            trajectory.position = self.default_positions[i].copy()
            trajectory.velocity = np.zeros(3)
            trajectory.is_support_leg = True
            
    def update(self, dt: float) -> List[np.ndarray]:
        """
        Update gait generation for one time step
        
        Args:
            dt: Time step duration (seconds)
            
        Returns:
            List of target foot positions for each leg
        """
        start_time = time.perf_counter()
        
        if not self.is_active:
            # Return default positions when not active
            return [pos.copy() for pos in self.default_positions]
            
        # Update cycle time
        self.cycle_time += dt
        if self.cycle_time >= self.parameters.cycle_time:
            self.cycle_time -= self.parameters.cycle_time
            
        # Calculate target positions for each leg
        target_positions = []
        
        for leg_id in range(6):
            # Calculate leg phase based on cycle time and phase offset
            leg_phase_time = (self.cycle_time + 
                            self.parameters.phase_offsets[leg_id] * self.parameters.cycle_time)
            leg_phase_time = leg_phase_time % self.parameters.cycle_time
            
            # Determine if leg should be in stance or swing
            stance_duration = self.parameters.cycle_time * self.parameters.duty_factor
            
            if leg_phase_time < stance_duration:
                # Stance phase
                target_pos = self._calculate_stance_position(leg_id, leg_phase_time, stance_duration)
                self.leg_trajectories[leg_id].phase = LegPhase.STANCE
                self.leg_trajectories[leg_id].is_support_leg = True
            else:
                # Swing phase
                swing_time = leg_phase_time - stance_duration
                swing_duration = self.parameters.cycle_time * (1.0 - self.parameters.duty_factor)
                target_pos = self._calculate_swing_position(leg_id, swing_time, swing_duration)
                self.leg_trajectories[leg_id].phase = LegPhase.SWING
                self.leg_trajectories[leg_id].is_support_leg = False
                
            # Update trajectory state
            self.leg_trajectories[leg_id].position = target_pos
            self.leg_trajectories[leg_id].phase_time = leg_phase_time
            
            target_positions.append(target_pos)
            
        # Performance tracking
        computation_time = (time.perf_counter() - start_time) * 1000
        self.computation_times.append(computation_time)
        if len(self.computation_times) > 1000:
            self.computation_times.pop(0)
            
        return target_positions
        
    def _calculate_stance_position(self, leg_id: int, phase_time: float, 
                                 stance_duration: float) -> np.ndarray:
        """Calculate foot position during stance phase"""
        # Normalize phase time (0 to 1)
        t = phase_time / stance_duration
        
        # Get default position for this leg
        default_pos = self.default_positions[leg_id].copy()
        
        # Calculate displacement based on body velocity
        # During stance, foot moves backward relative to body
        displacement = -self.parameters.velocity * self.parameters.cycle_time * t
        
        # Add stride pattern
        if self.parameters.stride_length > 0:
            # Move from front of stride to back of stride
            stride_progress = t
            forward_displacement = (0.5 - stride_progress) * self.parameters.stride_length
            displacement[0] += forward_displacement
            
        return default_pos + displacement
        
    def _calculate_swing_position(self, leg_id: int, swing_time: float, 
                                swing_duration: float) -> np.ndarray:
        """Calculate foot position during swing phase"""
        # Normalize swing time (0 to 1)
        t = swing_time / swing_duration
        
        # Get default position
        default_pos = self.default_positions[leg_id].copy()
        
        # Calculate start and end positions for swing
        swing_start = default_pos + np.array([-self.parameters.stride_length/2, 0, 0])
        swing_end = default_pos + np.array([self.parameters.stride_length/2, 0, 0])
        
        # Add body velocity compensation
        velocity_compensation = self.parameters.velocity * swing_duration
        swing_end += velocity_compensation
        
        # Linear interpolation for x and y
        swing_pos = swing_start + t * (swing_end - swing_start)
        
        # Parabolic trajectory for z (step height)
        # Maximum height at t = 0.5
        height_factor = 4 * t * (1 - t)  # Parabolic curve, max at t=0.5
        swing_pos[2] = default_pos[2] + height_factor * self.parameters.step_height
        
        return swing_pos
        
    def set_velocity(self, linear_velocity: np.ndarray, angular_velocity: float = 0.0):
        """
        Set desired movement velocity
        
        Args:
            linear_velocity: Desired linear velocity [vx, vy, vz] in m/s
            angular_velocity: Desired angular velocity in rad/s
        """
        self.parameters.velocity = np.array(linear_velocity)
        self.parameters.angular_velocity = angular_velocity
        
        # Adjust gait parameters based on velocity
        speed = np.linalg.norm(linear_velocity[:2])  # Horizontal speed
        
        if speed > 0.1:  # Fast movement
            if self.current_gait == GaitType.WAVE:
                # Switch to tripod for faster movement
                self.set_gait_type(GaitType.TRIPOD)
        elif speed < 0.05:  # Slow movement
            if self.current_gait == GaitType.TRIPOD:
                # Switch to wave for stability
                self.set_gait_type(GaitType.WAVE)
                
    def get_support_pattern(self) -> List[bool]:
        """
        Get current support pattern (which legs are supporting)
        
        Returns:
            List of boolean values indicating support status for each leg
        """
        return [traj.is_support_leg for traj in self.leg_trajectories]
        
    def get_gait_info(self) -> Dict[str, Any]:
        """Get comprehensive gait information"""
        support_count = sum(self.get_support_pattern())
        
        return {
            'gait_type': self.current_gait.value,
            'cycle_time': self.cycle_time,
            'total_cycle_time': self.parameters.cycle_time,
            'duty_factor': self.parameters.duty_factor,
            'step_height': self.parameters.step_height,
            'stride_length': self.parameters.stride_length,
            'velocity': self.parameters.velocity,
            'angular_velocity': self.parameters.angular_velocity,
            'is_active': self.is_active,
            'is_transitioning': self.is_transitioning,
            'support_legs': support_count,
            'support_pattern': self.get_support_pattern(),
            'leg_phases': [traj.phase.value for traj in self.leg_trajectories]
        }
        
    def get_performance_metrics(self) -> Dict[str, float]:
        """Get performance metrics"""
        if not self.computation_times:
            return {}
            
        return {
            'avg_computation_ms': np.mean(self.computation_times),
            'max_computation_ms': np.max(self.computation_times),
            'min_computation_ms': np.min(self.computation_times),
            'std_computation_ms': np.std(self.computation_times),
            'updates_per_second': 1000.0 / np.mean(self.computation_times) if np.mean(self.computation_times) > 0 else 0
        }
        
    def reset_performance_tracking(self):
        """Reset performance tracking data"""
        self.computation_times.clear()
        
    def update_parameters(self, **kwargs):
        """Update gait parameters"""
        for key, value in kwargs.items():
            if hasattr(self.parameters, key):
                setattr(self.parameters, key, value)
                
    def calculate_foot_velocities(self, target_positions: List[np.ndarray]) -> List[np.ndarray]:
        """
        Calculate foot velocities for current trajectories
        
        Args:
            target_positions: Current target foot positions
            
        Returns:
            List of foot velocities for each leg
        """
        velocities = []
        
        for leg_id in range(6):
            trajectory = self.leg_trajectories[leg_id]
            
            if trajectory.phase == LegPhase.STANCE:
                # During stance, foot velocity should match negative body velocity
                velocity = -self.parameters.velocity
            else:
                # During swing, calculate velocity based on swing trajectory
                # This is a simplified calculation
                swing_duration = self.parameters.cycle_time * (1.0 - self.parameters.duty_factor)
                
                if swing_duration > 0:
                    # Estimate velocity based on remaining swing distance and time
                    velocity = np.array([
                        self.parameters.stride_length / swing_duration,
                        0.0,
                        0.0  # Vertical velocity depends on swing phase
                    ])
                else:
                    velocity = np.zeros(3)
                    
            trajectory.velocity = velocity
            velocities.append(velocity)
            
        return velocities


class GaitAnalyzer:
    """Utility class for gait analysis and optimization"""
    
    @staticmethod
    def calculate_stability_margin(support_pattern: List[bool], 
                                 foot_positions: List[np.ndarray],
                                 com_position: np.ndarray) -> float:
        """
        Calculate stability margin for current support pattern
        
        Args:
            support_pattern: Which legs are currently supporting
            foot_positions: Current foot positions
            com_position: Center of mass position
            
        Returns:
            Stability margin in meters
        """
        # Get support leg positions
        support_positions = []
        for i, is_support in enumerate(support_pattern):
            if is_support:
                support_positions.append(foot_positions[i][:2])  # Only x,y
                
        if len(support_positions) < 3:
            return 0.0  # Unstable with less than 3 support points
            
        # Calculate distance from COM to support polygon edges
        support_array = np.array(support_positions)
        com_2d = com_position[:2]
        
        # Simple check: is COM inside the convex hull of support points?
        # For a triangle or more complex polygon, we'll use a simplified approach
        
        # Calculate centroid of support polygon
        centroid = np.mean(support_array, axis=0)
        
        # Calculate distances from COM to each support point
        distances_to_points = []
        for support_pos in support_array:
            dist = np.linalg.norm(com_2d - support_pos)
            distances_to_points.append(dist)
            
        # Calculate distance from COM to centroid
        dist_to_centroid = np.linalg.norm(com_2d - centroid)
        
        # Simple stability metric: 
        # If COM is close to centroid, use minimum distance to support points
        # If COM is far from centroid, return negative margin
        avg_support_radius = np.mean(distances_to_points)
        
        if dist_to_centroid < avg_support_radius * 0.7:
            # COM is reasonably within support polygon
            min_distance = min(distances_to_points) * 0.5  # Conservative estimate
            return min_distance
        else:
            # COM is likely outside support polygon
            return -dist_to_centroid * 0.1
        
    @staticmethod
    def _point_to_line_distance(point: np.ndarray, line_start: np.ndarray, 
                               line_end: np.ndarray) -> float:
        """Calculate distance from point to line segment"""
        # Vector from line start to end
        line_vec = line_end - line_start
        line_len_sq = np.dot(line_vec, line_vec)
        
        if line_len_sq == 0:
            return np.linalg.norm(point - line_start)
            
        # Project point onto line
        t = max(0, min(1, np.dot(point - line_start, line_vec) / line_len_sq))
        projection = line_start + t * line_vec
        
        return np.linalg.norm(point - projection)
        
    @staticmethod
    def evaluate_gait_quality(gait_info: Dict[str, Any], 
                            stability_margins: List[float]) -> Dict[str, float]:
        """
        Evaluate overall gait quality metrics
        
        Args:
            gait_info: Gait information dictionary
            stability_margins: List of stability margins over time
            
        Returns:
            Dictionary of quality metrics
        """
        metrics = {}
        
        if stability_margins:
            metrics['avg_stability'] = np.mean(stability_margins)
            metrics['min_stability'] = np.min(stability_margins)
            metrics['stability_variance'] = np.var(stability_margins)
            
        # Support pattern analysis
        support_pattern = gait_info.get('support_pattern', [])
        if support_pattern:
            support_count = sum(support_pattern)
            metrics['avg_support_legs'] = support_count
            metrics['support_ratio'] = support_count / len(support_pattern)
            
        # Velocity efficiency
        velocity_magnitude = np.linalg.norm(gait_info.get('velocity', [0, 0, 0]))
        metrics['velocity_efficiency'] = velocity_magnitude / max(0.1, gait_info.get('duty_factor', 1.0))
        
        return metrics