#!/usr/bin/env python3
"""
PID Controller System for Hexapod Robot

This module implements individual PID controllers for all 18 joints of the hexapod robot.
Each joint has its own controller with tunable gains and degree-based setpoint interface.

Key Features:
- Individual PID controllers for 18 joints (6 legs × 3 DOF)
- Degree-based setpoint and feedback interface
- Configurable PID gains per joint type (coxa, femur, tibia)
- Real-time control loop ready (100Hz+ operation)
- Servo motor simulation with realistic dynamics
- Comprehensive control system monitoring

Author: HexaPodSim 2.0
Date: October 2025
"""

import numpy as np
import time
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass, field
from enum import Enum
import warnings


class JointType(Enum):
    """Joint type enumeration for different PID tuning."""
    COXA = "coxa"
    FEMUR = "femur" 
    TIBIA = "tibia"


@dataclass
class PIDGains:
    """PID controller gain parameters."""
    kp: float = 1.0     # Proportional gain
    ki: float = 0.1     # Integral gain  
    kd: float = 0.05    # Derivative gain
    
    # Output limits
    output_min: float = -90.0  # Minimum output (degrees/sec)
    output_max: float = 90.0   # Maximum output (degrees/sec)
    
    # Integral limits (anti-windup)
    integral_min: float = -10.0
    integral_max: float = 10.0


@dataclass
class ServoParameters:
    """Servo motor simulation parameters."""
    max_speed: float = 60.0      # Maximum speed (degrees/sec)
    max_acceleration: float = 300.0  # Maximum acceleration (degrees/sec²)
    position_deadband: float = 0.1   # Position deadband (degrees)
    
    # Realistic servo dynamics
    damping_ratio: float = 0.7
    natural_frequency: float = 20.0  # rad/s
    
    # Physical limits
    min_angle: float = -90.0
    max_angle: float = 90.0


@dataclass
class ControllerConfig:
    """Configuration for all PID controllers."""
    
    # Default PID gains by joint type
    coxa_gains: PIDGains = field(default_factory=lambda: PIDGains(
        kp=2.0, ki=0.2, kd=0.1,
        output_min=-60.0, output_max=60.0
    ))
    
    femur_gains: PIDGains = field(default_factory=lambda: PIDGains(
        kp=3.0, ki=0.3, kd=0.15,
        output_min=-90.0, output_max=90.0
    ))
    
    tibia_gains: PIDGains = field(default_factory=lambda: PIDGains(
        kp=2.5, ki=0.25, kd=0.12,
        output_min=-90.0, output_max=90.0
    ))
    
    # Servo parameters (same for all joints)
    servo_params: ServoParameters = field(default_factory=ServoParameters)
    
    # Control loop settings
    control_frequency: float = 100.0  # Hz
    dt: float = field(init=False)
    
    def __post_init__(self):
        self.dt = 1.0 / self.control_frequency


class PIDController:
    """
    Individual PID controller for a single joint.
    
    Implements a discrete-time PID controller with anti-windup,
    output limiting, and derivative kick prevention.
    """
    
    def __init__(self, gains: PIDGains, dt: float, joint_id: str):
        """
        Initialize PID controller.
        
        Args:
            gains: PID gain parameters
            dt: Control loop time step (seconds)
            joint_id: Unique joint identifier for debugging
        """
        self.gains = gains
        self.dt = dt
        self.joint_id = joint_id
        
        # Controller state
        self.setpoint = 0.0  # degrees
        self.last_feedback = 0.0  # degrees
        self.integral = 0.0
        self.last_error = 0.0
        
        # Timing
        self.last_update_time = 0.0
        self.first_update = True  # Flag to track first update
        
        # Statistics
        self.reset_statistics()
    
    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_feedback = 0.0
        self.last_update_time = 0.0
        self.first_update = True
        self.reset_statistics()
    
    def reset_statistics(self):
        """Reset performance statistics."""
        self.max_error = 0.0
        self.rms_error = 0.0
        self.settle_time = 0.0
        self.overshoot = 0.0
        self._error_history = []
    
    def set_setpoint(self, setpoint_deg: float):
        """
        Set controller setpoint.
        
        Args:
            setpoint_deg: Target position in degrees
        """
        self.setpoint = np.clip(setpoint_deg, 
                               self.gains.output_min, 
                               self.gains.output_max)
    
    def update(self, feedback_deg: float, current_time: float = None) -> float:
        """
        Update PID controller and return control output.
        
        Args:
            feedback_deg: Current joint position in degrees
            current_time: Current time (seconds), uses time.time() if None
            
        Returns:
            Control output (velocity command in degrees/sec)
        """
        if current_time is None:
            current_time = time.time()
        
        # Calculate actual dt from timing
        if self.last_update_time > 0:
            actual_dt = current_time - self.last_update_time
            # Use configured dt if timing is reasonable, otherwise use actual
            if 0.005 <= actual_dt <= 0.05:  # 20Hz to 200Hz range
                dt = actual_dt
            else:
                dt = self.dt
        else:
            dt = self.dt
        
        self.last_update_time = current_time
        
        # Calculate error
        error = self.setpoint - feedback_deg
        
        # Proportional term
        proportional = self.gains.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, 
                               self.gains.integral_min, 
                               self.gains.integral_max)
        integral_term = self.gains.ki * self.integral
        
        # Derivative term (derivative on measurement to avoid derivative kick)
        if dt > 0 and not self.first_update:  # Only calculate derivative after first update
            derivative = -self.gains.kd * (feedback_deg - self.last_feedback) / dt
        else:
            derivative = 0.0
        
        # Calculate output
        output = proportional + integral_term + derivative
        
        # Apply output limits
        output = np.clip(output, self.gains.output_min, self.gains.output_max)
        
        # Update state
        self.last_feedback = feedback_deg
        self.last_error = error
        
        # Update statistics
        self._update_statistics(error, feedback_deg)
        
        return output
    
    def _update_statistics(self, error: float, feedback: float):
        """Update performance statistics."""
        abs_error = abs(error)
        self.max_error = max(self.max_error, abs_error)
        
        self._error_history.append(error)
        if len(self._error_history) > 1000:  # Keep last 1000 samples
            self._error_history.pop(0)
        
        # Calculate RMS error
        if self._error_history:
            self.rms_error = np.sqrt(np.mean(np.array(self._error_history)**2))
    
    def get_status(self) -> Dict[str, float]:
        """Get controller status and statistics."""
        return {
            'setpoint': self.setpoint,
            'feedback': self.last_feedback,
            'error': self.last_error,
            'integral': self.integral,
            'max_error': self.max_error,
            'rms_error': self.rms_error,
            'output_range': [self.gains.output_min, self.gains.output_max]
        }


class ServoMotor:
    """
    Servo motor simulation with realistic dynamics.
    
    Simulates a real servo motor with speed/acceleration limits,
    second-order dynamics, and position feedback.
    """
    
    def __init__(self, params: ServoParameters, joint_id: str):
        """
        Initialize servo motor simulation.
        
        Args:
            params: Servo motor parameters
            joint_id: Joint identifier
        """
        self.params = params
        self.joint_id = joint_id
        
        # State variables
        self.position = 0.0  # degrees
        self.velocity = 0.0  # degrees/sec
        self.acceleration = 0.0  # degrees/sec²
        
        # Command input
        self.velocity_command = 0.0  # degrees/sec
        
        # Timing
        self.last_update_time = 0.0
    
    def reset(self, initial_position: float = 0.0):
        """Reset servo to initial state."""
        self.position = np.clip(initial_position, 
                               self.params.min_angle, 
                               self.params.max_angle)
        self.velocity = 0.0
        self.acceleration = 0.0
        self.velocity_command = 0.0
        self.last_update_time = 0.0
    
    def set_velocity_command(self, velocity_cmd: float):
        """Set velocity command (from PID controller)."""
        self.velocity_command = np.clip(velocity_cmd,
                                       -self.params.max_speed,
                                       self.params.max_speed)
    
    def update(self, dt: float) -> float:
        """
        Update servo motor dynamics.
        
        Args:
            dt: Time step (seconds)
            
        Returns:
            Current position (degrees)
        """
        # Calculate required acceleration to reach commanded velocity
        velocity_error = self.velocity_command - self.velocity
        
        # Simple first-order velocity control with acceleration limits
        if abs(velocity_error) > 0.1:  # Velocity deadband
            desired_accel = velocity_error * self.params.natural_frequency
            
            # Apply acceleration limits
            self.acceleration = np.clip(desired_accel,
                                       -self.params.max_acceleration,
                                       self.params.max_acceleration)
        else:
            self.acceleration = 0.0
        
        # Integrate dynamics
        self.velocity += self.acceleration * dt
        self.velocity = np.clip(self.velocity,
                               -self.params.max_speed,
                               self.params.max_speed)
        
        # Update position
        old_position = self.position
        self.position += self.velocity * dt
        
        # Apply position limits
        if self.position <= self.params.min_angle:
            self.position = self.params.min_angle
            self.velocity = max(0.0, self.velocity)  # Allow only positive velocity
        elif self.position >= self.params.max_angle:
            self.position = self.params.max_angle
            self.velocity = min(0.0, self.velocity)  # Allow only negative velocity
        
        return self.position
    
    def get_status(self) -> Dict[str, float]:
        """Get servo status."""
        return {
            'position': self.position,
            'velocity': self.velocity,
            'acceleration': self.acceleration,
            'velocity_command': self.velocity_command,
            'at_limit': (self.position <= self.params.min_angle + 0.1 or 
                        self.position >= self.params.max_angle - 0.1)
        }


class HexapodControllerSystem:
    """
    Complete controller system for hexapod robot.
    
    Manages 18 individual PID controllers and servo motors for all joints.
    Provides high-level interface for setting joint positions and monitoring
    control system performance.
    """
    
    def __init__(self, config: ControllerConfig = None):
        """
        Initialize controller system.
        
        Args:
            config: Controller configuration, uses defaults if None
        """
        self.config = config or ControllerConfig()
        
        # Joint naming (consistent with kinematics)
        self.LEG_NAMES = ['L1', 'R1', 'L2', 'R2', 'L3', 'R3']
        self.JOINT_NAMES = ['coxa', 'femur', 'tibia']
        
        # Create all joint identifiers
        self.joint_ids = []
        for leg in self.LEG_NAMES:
            for joint in self.JOINT_NAMES:
                self.joint_ids.append(f"{leg}_{joint}")
        
        # Initialize controllers and servos
        self.controllers: Dict[str, PIDController] = {}
        self.servos: Dict[str, ServoMotor] = {}
        
        self._create_controllers()
        
        # System state
        self.is_running = False
        self.control_loop_time = 0.0
        self.last_control_time = 0.0
        
        # Performance monitoring
        self.loop_times = []
        self.max_loop_time = 0.0
        self.control_frequency_actual = 0.0
    
    def _create_controllers(self):
        """Create all PID controllers and servo motors."""
        for joint_id in self.joint_ids:
            leg, joint_name = joint_id.split('_')
            
            # Select appropriate gains based on joint type
            if joint_name == 'coxa':
                gains = self.config.coxa_gains
            elif joint_name == 'femur':
                gains = self.config.femur_gains
            else:  # tibia
                gains = self.config.tibia_gains
            
            # Create controller and servo
            self.controllers[joint_id] = PIDController(gains, self.config.dt, joint_id)
            self.servos[joint_id] = ServoMotor(self.config.servo_params, joint_id)
    
    def reset_all(self, initial_positions: Dict[str, float] = None):
        """
        Reset all controllers and servos.
        
        Args:
            initial_positions: Initial joint positions {joint_id: position_deg}
                              Uses 0.0 for all joints if None
        """
        initial_positions = initial_positions or {}
        
        for joint_id in self.joint_ids:
            initial_pos = initial_positions.get(joint_id, 0.0)
            
            self.controllers[joint_id].reset()
            self.controllers[joint_id].set_setpoint(initial_pos)
            self.servos[joint_id].reset(initial_pos)
        
        # Reset system statistics
        self.loop_times = []
        self.max_loop_time = 0.0
        self.last_control_time = 0.0
    
    def set_joint_angles(self, joint_angles: Dict[str, float]):
        """
        Set target joint angles for specified joints.
        
        Args:
            joint_angles: Dictionary mapping joint_id to target angle (degrees)
        """
        for joint_id, angle in joint_angles.items():
            if joint_id in self.controllers:
                self.controllers[joint_id].set_setpoint(angle)
            else:
                warnings.warn(f"Unknown joint_id: {joint_id}")
    
    def set_leg_angles(self, leg_id: str, coxa: float, femur: float, tibia: float):
        """
        Set joint angles for a complete leg.
        
        Args:
            leg_id: Leg identifier ('L1', 'R1', etc.)
            coxa: Coxa joint angle (degrees)
            femur: Femur joint angle (degrees)  
            tibia: Tibia joint angle (degrees)
        """
        if leg_id not in self.LEG_NAMES:
            raise ValueError(f"Invalid leg_id: {leg_id}")
        
        joint_angles = {
            f"{leg_id}_coxa": coxa,
            f"{leg_id}_femur": femur,
            f"{leg_id}_tibia": tibia
        }
        
        self.set_joint_angles(joint_angles)
    
    def set_all_leg_angles(self, leg_angles: Dict[str, np.ndarray]):
        """
        Set joint angles for all legs simultaneously.
        
        Args:
            leg_angles: Dictionary mapping leg_id to [coxa, femur, tibia] angles
        """
        for leg_id, angles in leg_angles.items():
            if len(angles) != 3:
                raise ValueError(f"Expected 3 angles for {leg_id}, got {len(angles)}")
            
            self.set_leg_angles(leg_id, angles[0], angles[1], angles[2])
    
    def update_control_loop(self, current_time: float = None) -> Dict[str, float]:
        """
        Execute one control loop iteration.
        
        Args:
            current_time: Current time (seconds), uses time.time() if None
            
        Returns:
            Dictionary of current joint positions {joint_id: position_deg}
        """
        if current_time is None:
            current_time = time.time()
        
        loop_start_time = current_time
        
        # Calculate dt from last update
        if self.last_control_time > 0:
            dt = current_time - self.last_control_time
        else:
            dt = self.config.dt
        
        self.last_control_time = current_time
        
        # Update all controllers and servos
        joint_positions = {}
        
        for joint_id in self.joint_ids:
            # Get current position from servo
            current_position = self.servos[joint_id].position
            
            # Update PID controller
            velocity_command = self.controllers[joint_id].update(current_position, current_time)
            
            # Send command to servo
            self.servos[joint_id].set_velocity_command(velocity_command)
            
            # Update servo dynamics
            new_position = self.servos[joint_id].update(dt)
            
            joint_positions[joint_id] = new_position
        
        # Update performance statistics
        loop_time = time.time() - loop_start_time
        self.loop_times.append(loop_time)
        
        if len(self.loop_times) > 1000:  # Keep last 1000 samples
            self.loop_times.pop(0)
        
        self.max_loop_time = max(self.max_loop_time, loop_time)
        
        # Calculate actual control frequency
        if len(self.loop_times) > 10:
            avg_loop_time = np.mean(self.loop_times[-10:])
            self.control_frequency_actual = 1.0 / avg_loop_time if avg_loop_time > 0 else 0.0
        
        return joint_positions
    
    def get_joint_positions(self) -> Dict[str, float]:
        """Get current joint positions."""
        return {joint_id: servo.position for joint_id, servo in self.servos.items()}
    
    def get_leg_positions(self, leg_id: str) -> np.ndarray:
        """
        Get joint positions for a specific leg.
        
        Args:
            leg_id: Leg identifier
            
        Returns:
            Array of [coxa, femur, tibia] angles in degrees
        """
        if leg_id not in self.LEG_NAMES:
            raise ValueError(f"Invalid leg_id: {leg_id}")
        
        return np.array([
            self.servos[f"{leg_id}_coxa"].position,
            self.servos[f"{leg_id}_femur"].position,
            self.servos[f"{leg_id}_tibia"].position
        ])
    
    def get_all_leg_positions(self) -> Dict[str, np.ndarray]:
        """Get joint positions for all legs."""
        return {leg_id: self.get_leg_positions(leg_id) for leg_id in self.LEG_NAMES}
    
    def get_control_system_status(self) -> Dict[str, Any]:
        """Get comprehensive control system status."""
        status = {
            'system': {
                'is_running': self.is_running,
                'control_frequency_target': self.config.control_frequency,
                'control_frequency_actual': self.control_frequency_actual,
                'max_loop_time': self.max_loop_time * 1000,  # Convert to ms
                'avg_loop_time': np.mean(self.loop_times[-100:]) * 1000 if self.loop_times else 0.0
            },
            'joints': {},
            'performance': {
                'total_joints': len(self.joint_ids),
                'joints_at_setpoint': 0,
                'joints_at_limits': 0,
                'max_position_error': 0.0,
                'avg_position_error': 0.0
            }
        }
        
        # Collect joint status
        position_errors = []
        for joint_id in self.joint_ids:
            controller_status = self.controllers[joint_id].get_status()
            servo_status = self.servos[joint_id].get_status()
            
            joint_status = {
                'controller': controller_status,
                'servo': servo_status
            }
            
            status['joints'][joint_id] = joint_status
            
            # Update performance metrics
            error = abs(controller_status['error'])
            position_errors.append(error)
            
            if error < 1.0:  # Within 1 degree
                status['performance']['joints_at_setpoint'] += 1
            
            if servo_status['at_limit']:
                status['performance']['joints_at_limits'] += 1
        
        # Calculate aggregate performance metrics
        if position_errors:
            status['performance']['max_position_error'] = max(position_errors)
            status['performance']['avg_position_error'] = np.mean(position_errors)
        
        return status


def validate_controller_system():
    """Validate the controller system implementation."""
    print("🎮 Validating HexaPodSim 2.0 Controller System")
    print("=" * 60)
    
    # Create controller system
    config = ControllerConfig()
    controller_system = HexapodControllerSystem(config)
    
    # Test initialization
    print(f"✅ Initialized {len(controller_system.joint_ids)} joint controllers")
    print(f"   Target control frequency: {config.control_frequency}Hz")
    print(f"   Control loop dt: {config.dt*1000:.1f}ms")
    
    # Test joint configuration
    print(f"\n🦿 Joint Configuration:")
    for leg in controller_system.LEG_NAMES:
        positions = controller_system.get_leg_positions(leg)
        print(f"   {leg}: [{positions[0]:+6.1f}, {positions[1]:+6.1f}, {positions[2]:+6.1f}]°")
    
    # Test setpoint setting
    print(f"\n📍 Testing Setpoint Commands:")
    test_angles = {
        'L1_coxa': 15.0,
        'L1_femur': -20.0, 
        'L1_tibia': 30.0
    }
    
    controller_system.set_joint_angles(test_angles)
    
    for joint_id, target in test_angles.items():
        controller = controller_system.controllers[joint_id]
        print(f"   {joint_id}: setpoint = {controller.setpoint:.1f}° (target: {target:.1f}°)")
    
    print(f"\n✅ Controller system validation complete!")


if __name__ == "__main__":
    validate_controller_system()