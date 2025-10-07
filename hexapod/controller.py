"""
PID Controller System for Hexapod Robot

This module implements comprehensive PID control for all 18 joints of the hexapod robot,
providing individual controllers with tunable parameters, output limiting, and
real-time performance monitoring.

Controller Features:
- Individual PID controllers for each joint (18 total)
- Degree-based angle interface for intuitive control
- Configurable PID gains (Kp, Ki, Kd) per joint type
- Output limiting and saturation handling
- Derivative filtering for noise reduction
- Integral windup protection

Key Features:
- Real-time control at 100Hz minimum
- Joint-specific tuning (coxa, femur, tibia)
- Smooth setpoint tracking
- Disturbance rejection
- Hardware-ready output values

Author: Hexapod Simulation Team
Date: October 2024
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any
from enum import Enum
import time

from .kinematics import HexapodKinematics


class JointType(Enum):
    """Joint type enumeration for type-specific tuning"""
    COXA = "coxa"       # Hip joint (rotation around vertical axis)
    FEMUR = "femur"     # Upper leg joint (forward/backward)
    TIBIA = "tibia"     # Lower leg joint (knee)


@dataclass
class PIDGains:
    """PID controller gains"""
    kp: float = 1.0     # Proportional gain
    ki: float = 0.0     # Integral gain  
    kd: float = 0.0     # Derivative gain
    
    # Tuning parameters
    output_limit: float = 180.0     # Maximum output magnitude (degrees/sec)
    integral_limit: float = 50.0    # Maximum integral accumulation
    derivative_filter: float = 0.1  # Derivative filter coefficient (0-1)
    
    def __post_init__(self):
        """Validate gain values"""
        self.kp = max(0.0, self.kp)
        self.ki = max(0.0, self.ki)  
        self.kd = max(0.0, self.kd)
        self.output_limit = max(0.1, self.output_limit)
        self.integral_limit = max(0.1, self.integral_limit)
        self.derivative_filter = max(0.0, min(1.0, self.derivative_filter))


@dataclass
class ControllerState:
    """Internal state of PID controller"""
    # Control variables
    setpoint: float = 0.0           # Target angle (degrees)
    process_value: float = 0.0      # Current angle (degrees)
    output: float = 0.0             # Controller output (degrees/sec)
    
    # Error tracking
    error: float = 0.0              # Current error
    previous_error: float = 0.0     # Previous error for derivative
    integral: float = 0.0           # Integral accumulation
    derivative: float = 0.0         # Derivative term
    filtered_derivative: float = 0.0  # Filtered derivative
    
    # Timing
    last_update_time: float = 0.0   # Last update timestamp
    dt: float = 0.01               # Time step
    
    # Status
    is_enabled: bool = True         # Controller enabled
    is_saturated: bool = False      # Output saturation status
    
    def reset(self):
        """Reset controller state"""
        self.error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.filtered_derivative = 0.0
        self.output = 0.0
        self.is_saturated = False


class PIDController:
    """Individual PID controller for one joint"""
    
    def __init__(self, joint_id: str, joint_type: JointType, gains: PIDGains = None):
        """
        Initialize PID controller
        
        Args:
            joint_id: Unique identifier for the joint (e.g., "L1_coxa")
            joint_type: Type of joint for appropriate tuning
            gains: PID gains and parameters
        """
        self.joint_id = joint_id
        self.joint_type = joint_type
        self.gains = gains or self._get_default_gains(joint_type)
        self.state = ControllerState()
        
        # Performance tracking
        self.performance_history = {
            'error': [],
            'output': [],
            'computation_time': []
        }
        
    def _get_default_gains(self, joint_type: JointType) -> PIDGains:
        """Get default gains based on joint type"""
        if joint_type == JointType.COXA:
            # Coxa joints - rotational, need precise positioning
            return PIDGains(
                kp=8.0,           # High proportional gain for responsiveness
                ki=0.1,           # Small integral for steady-state accuracy
                kd=0.5,           # Moderate derivative for damping
                output_limit=120.0,
                integral_limit=20.0,
                derivative_filter=0.1
            )
        elif joint_type == JointType.FEMUR:
            # Femur joints - load bearing, need stability
            return PIDGains(
                kp=10.0,          # High proportional gain
                ki=0.2,           # Moderate integral for load compensation
                kd=0.8,           # Higher derivative for stability
                output_limit=150.0,
                integral_limit=30.0,
                derivative_filter=0.15
            )
        else:  # TIBIA
            # Tibia joints - fine positioning, fastest response
            return PIDGains(
                kp=12.0,          # Highest proportional gain
                ki=0.05,          # Low integral to avoid oscillation
                kd=1.0,           # High derivative for damping
                output_limit=180.0,
                integral_limit=15.0,
                derivative_filter=0.2
            )
            
    def update(self, setpoint: float, process_value: float, dt: float = None) -> float:
        """
        Update PID controller
        
        Args:
            setpoint: Target angle in degrees
            process_value: Current angle in degrees
            dt: Time step in seconds (auto-calculated if None)
            
        Returns:
            Controller output in degrees/second
        """
        start_time = time.perf_counter()
        
        # Update state
        self.state.setpoint = setpoint
        self.state.process_value = process_value
        
        # Calculate time step
        current_time = time.perf_counter()
        if dt is None:
            if self.state.last_update_time > 0:
                dt = current_time - self.state.last_update_time
            else:
                dt = 0.01  # Default 10ms
        self.state.dt = dt
        self.state.last_update_time = current_time
        
        if not self.state.is_enabled or dt <= 0:
            return self.state.output
            
        # Calculate error
        self.state.previous_error = self.state.error
        self.state.error = setpoint - process_value
        
        # Handle angle wraparound (e.g., -180 to +180 transition)
        if abs(self.state.error) > 180:
            if self.state.error > 0:
                self.state.error -= 360
            else:
                self.state.error += 360
                
        # Proportional term
        proportional = self.gains.kp * self.state.error
        
        # Integral term with windup protection
        if not self.state.is_saturated or (self.state.error * self.state.integral) <= 0:
            self.state.integral += self.state.error * dt
            # Clamp integral
            self.state.integral = max(-self.gains.integral_limit, 
                                    min(self.gains.integral_limit, self.state.integral))
        integral = self.gains.ki * self.state.integral
        
        # Derivative term with filtering
        if dt > 0:
            raw_derivative = (self.state.error - self.state.previous_error) / dt
            # Apply low-pass filter to reduce noise
            alpha = self.gains.derivative_filter
            self.state.filtered_derivative = (alpha * raw_derivative + 
                                            (1 - alpha) * self.state.filtered_derivative)
            self.state.derivative = self.state.filtered_derivative
        derivative = self.gains.kd * self.state.derivative
        
        # Calculate output
        output = proportional + integral + derivative
        
        # Apply output limits
        self.state.is_saturated = False
        if abs(output) > self.gains.output_limit:
            self.state.is_saturated = True
            output = np.sign(output) * self.gains.output_limit
            
        self.state.output = output
        
        # Performance tracking
        computation_time = (time.perf_counter() - start_time) * 1000
        self._update_performance_history(computation_time)
        
        return output
        
    def _update_performance_history(self, computation_time: float):
        """Update performance tracking history"""
        max_history = 1000
        
        self.performance_history['error'].append(abs(self.state.error))
        self.performance_history['output'].append(abs(self.state.output))
        self.performance_history['computation_time'].append(computation_time)
        
        # Limit history size
        for key in self.performance_history:
            if len(self.performance_history[key]) > max_history:
                self.performance_history[key].pop(0)
                
    def set_gains(self, kp: float = None, ki: float = None, kd: float = None, **kwargs):
        """Update PID gains"""
        if kp is not None:
            self.gains.kp = max(0.0, kp)
        if ki is not None:
            self.gains.ki = max(0.0, ki)
        if kd is not None:
            self.gains.kd = max(0.0, kd)
            
        # Update other parameters
        for key, value in kwargs.items():
            if hasattr(self.gains, key):
                setattr(self.gains, key, value)
                
    def reset(self):
        """Reset controller state"""
        self.state.reset()
        
    def enable(self):
        """Enable controller"""
        self.state.is_enabled = True
        
    def disable(self):
        """Disable controller"""
        self.state.is_enabled = False
        self.state.output = 0.0
        
    def get_status(self) -> Dict[str, Any]:
        """Get controller status information"""
        return {
            'joint_id': self.joint_id,
            'joint_type': self.joint_type.value,
            'enabled': self.state.is_enabled,
            'setpoint': self.state.setpoint,
            'process_value': self.state.process_value,
            'error': self.state.error,
            'output': self.state.output,
            'saturated': self.state.is_saturated,
            'gains': {
                'kp': self.gains.kp,
                'ki': self.gains.ki,
                'kd': self.gains.kd
            }
        }
        
    def get_performance_metrics(self) -> Dict[str, float]:
        """Get performance metrics"""
        metrics = {}
        
        for key, values in self.performance_history.items():
            if values:
                metrics[f'{key}_avg'] = np.mean(values)
                metrics[f'{key}_max'] = np.max(values)
                metrics[f'{key}_std'] = np.std(values)
                
        return metrics


class JointControllerSystem:
    """Complete joint controller system for hexapod robot"""
    
    def __init__(self, kinematics: HexapodKinematics):
        """
        Initialize joint controller system
        
        Args:
            kinematics: Hexapod kinematics system
        """
        self.kinematics = kinematics
        
        # Create controllers for all joints
        self.controllers: Dict[str, PIDController] = {}
        self._create_controllers()
        
        # System state
        self.current_angles = np.zeros((6, 3))      # Current joint angles (degrees)
        self.target_angles = np.zeros((6, 3))       # Target joint angles (degrees)
        self.angular_velocities = np.zeros((6, 3))  # Joint velocities (degrees/sec)
        
        # Control mode
        self.control_frequency = 100.0  # Hz
        self.is_active = False
        self.last_update_time = time.perf_counter()
        
        # Performance tracking
        self.system_performance = {
            'update_times': [],
            'tracking_errors': [],
            'total_outputs': []
        }
        
    def _create_controllers(self):
        """Create PID controllers for all joints"""
        joint_types = [JointType.COXA, JointType.FEMUR, JointType.TIBIA]
        joint_names = ['coxa', 'femur', 'tibia']
        
        for i, leg_name in enumerate(self.kinematics.LEG_NAMES):
            for j, (joint_type, joint_name) in enumerate(zip(joint_types, joint_names)):
                controller_id = f"{leg_name}_{joint_name}"
                self.controllers[controller_id] = PIDController(
                    controller_id, joint_type
                )
                
    def start_control(self):
        """Start the control system"""
        self.is_active = True
        self.last_update_time = time.perf_counter()
        
        # Reset all controllers
        for controller in self.controllers.values():
            controller.reset()
            controller.enable()
            
    def stop_control(self):
        """Stop the control system"""
        self.is_active = False
        
        # Disable all controllers
        for controller in self.controllers.values():
            controller.disable()
            
    def update(self, target_positions: List[np.ndarray] = None, 
               target_joint_angles: np.ndarray = None) -> np.ndarray:
        """
        Update all joint controllers
        
        Args:
            target_positions: Target foot positions for inverse kinematics
            target_joint_angles: Direct target joint angles (6x3 array in degrees)
            
        Returns:
            Joint angular velocities (6x3 array in degrees/sec)
        """
        start_time = time.perf_counter()
        
        if not self.is_active:
            return np.zeros((6, 3))
            
        # Calculate time step
        current_time = time.perf_counter()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        # Get target angles
        if target_joint_angles is not None:
            self.target_angles = target_joint_angles.copy()
        elif target_positions is not None:
            # Use inverse kinematics to get target angles
            self.target_angles = self._calculate_target_angles(target_positions)
        else:
            # Hold current position
            pass
            
        # Update each controller
        total_error = 0.0
        total_output = 0.0
        
        for i, leg_name in enumerate(self.kinematics.LEG_NAMES):
            for j, joint_name in enumerate(['coxa', 'femur', 'tibia']):
                controller_id = f"{leg_name}_{joint_name}"
                controller = self.controllers[controller_id]
                
                # Update controller
                setpoint = self.target_angles[i, j]
                process_value = self.current_angles[i, j]
                
                output = controller.update(setpoint, process_value, dt)
                self.angular_velocities[i, j] = output
                
                # Track performance
                total_error += abs(controller.state.error)
                total_output += abs(output)
                
        # Update system performance tracking
        system_time = (time.perf_counter() - start_time) * 1000
        self._update_system_performance(system_time, total_error, total_output)
        
        return self.angular_velocities
        
    def _calculate_target_angles(self, target_positions: List[np.ndarray]) -> np.ndarray:
        """Calculate target joint angles from foot positions"""
        target_angles = np.zeros((6, 3))
        
        for i, (leg_name, target_pos) in enumerate(zip(self.kinematics.LEG_NAMES, target_positions)):
            try:
                # Use inverse kinematics
                joint_angles_rad = self.kinematics.inverse_kinematics(target_pos, leg_name)
                target_angles[i] = np.degrees(joint_angles_rad)
            except Exception as e:
                # Use current angles if IK fails
                target_angles[i] = self.current_angles[i]
                
        return target_angles
        
    def _update_system_performance(self, system_time: float, total_error: float, total_output: float):
        """Update system performance tracking"""
        max_history = 1000
        
        self.system_performance['update_times'].append(system_time)
        self.system_performance['tracking_errors'].append(total_error)
        self.system_performance['total_outputs'].append(total_output)
        
        # Limit history size
        for key in self.system_performance:
            if len(self.system_performance[key]) > max_history:
                self.system_performance[key].pop(0)
                
    def set_current_angles(self, joint_angles: np.ndarray):
        """
        Update current joint angles (feedback from sensors/simulation)
        
        Args:
            joint_angles: Current joint angles (6x3 array in degrees)
        """
        self.current_angles = joint_angles.copy()
        
    def set_target_angles(self, joint_angles: np.ndarray):
        """
        Set target joint angles directly
        
        Args:
            joint_angles: Target joint angles (6x3 array in degrees)
        """
        self.target_angles = joint_angles.copy()
        
    def get_controller(self, leg_name: str, joint_name: str) -> PIDController:
        """Get specific controller"""
        controller_id = f"{leg_name}_{joint_name}"
        return self.controllers.get(controller_id)
        
    def tune_controller(self, leg_name: str, joint_name: str, **gains):
        """Tune specific controller gains"""
        controller = self.get_controller(leg_name, joint_name)
        if controller:
            controller.set_gains(**gains)
            
    def tune_joint_type(self, joint_type: JointType, **gains):
        """Tune all controllers of a specific joint type"""
        for controller in self.controllers.values():
            if controller.joint_type == joint_type:
                controller.set_gains(**gains)
                
    def reset_all_controllers(self):
        """Reset all controllers"""
        for controller in self.controllers.values():
            controller.reset()
            
    def get_system_status(self) -> Dict[str, Any]:
        """Get complete system status"""
        # Calculate summary statistics
        total_error = 0.0
        saturated_count = 0
        max_output = 0.0
        
        controller_status = []
        
        for controller in self.controllers.values():
            status = controller.get_status()
            controller_status.append(status)
            
            total_error += abs(status['error'])
            if status['saturated']:
                saturated_count += 1
            max_output = max(max_output, abs(status['output']))
            
        return {
            'is_active': self.is_active,
            'control_frequency': self.control_frequency,
            'total_controllers': len(self.controllers),
            'total_error': total_error,
            'avg_error': total_error / len(self.controllers),
            'saturated_controllers': saturated_count,
            'max_output': max_output,
            'current_angles': self.current_angles.tolist(),
            'target_angles': self.target_angles.tolist(),
            'angular_velocities': self.angular_velocities.tolist(),
            'controllers': controller_status
        }
        
    def get_performance_summary(self) -> Dict[str, Any]:
        """Get performance summary"""
        summary = {}
        
        # System-level metrics
        for key, values in self.system_performance.items():
            if values:
                summary[f'system_{key}_avg'] = np.mean(values)
                summary[f'system_{key}_max'] = np.max(values)
                summary[f'system_{key}_std'] = np.std(values)
                
        # Individual controller metrics
        controller_metrics = {}
        for controller_id, controller in self.controllers.items():
            controller_metrics[controller_id] = controller.get_performance_metrics()
            
        summary['controllers'] = controller_metrics
        
        # Calculate control frequency
        if self.system_performance['update_times']:
            avg_update_time = np.mean(self.system_performance['update_times'])
            summary['actual_frequency'] = 1000.0 / avg_update_time if avg_update_time > 0 else 0
            
        return summary
        
    def emergency_stop(self):
        """Emergency stop - disable all controllers immediately"""
        self.stop_control()
        self.angular_velocities.fill(0.0)
        
        for controller in self.controllers.values():
            controller.state.output = 0.0
            
    def set_control_frequency(self, frequency: float):
        """Set desired control frequency"""
        self.control_frequency = max(1.0, frequency)
        
    def apply_safety_limits(self, max_angular_velocity: float = 180.0):
        """Apply safety limits to all outputs"""
        # Limit angular velocities
        self.angular_velocities = np.clip(
            self.angular_velocities, 
            -max_angular_velocity, 
            max_angular_velocity
        )
        
        # Update controller output limits
        for controller in self.controllers.values():
            controller.gains.output_limit = min(
                controller.gains.output_limit, 
                max_angular_velocity
            )