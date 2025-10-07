"""
Contact Force Estimation System for Hexapod Robot Simulation

This module implements comprehensive contact force modeling and estimation for the hexapod robot,
including ground interaction, force sensors, and contact detection algorithms.

Physics Model:
- Spring-damper ground contact with configurable stiffness and damping
- Coulomb friction model with static/kinetic friction coefficients
- Contact detection with penetration depth calculation
- Force sensor simulation with realistic noise characteristics

Key Features:
- Real-time contact force estimation
- Ground compliance modeling
- Friction force calculation
- Force sensor data processing
- Contact event detection and tracking

Author: Hexapod Simulation Team
Date: 2024
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any
from enum import Enum
import time
from abc import ABC, abstractmethod

from .kinematics import HexapodKinematics
from .dynamics import LegDynamics, BodyDynamics


class ContactState(Enum):
    """Contact state enumeration"""
    NO_CONTACT = 0
    TOUCHING = 1
    SLIDING = 2
    STICKING = 3


@dataclass
class ContactProperties:
    """Ground contact properties"""
    stiffness: float = 50000.0  # N/m - ground stiffness
    damping: float = 1000.0     # Ns/m - ground damping
    static_friction: float = 0.8 # Static friction coefficient
    kinetic_friction: float = 0.6 # Kinetic friction coefficient
    restitution: float = 0.1    # Coefficient of restitution
    penetration_threshold: float = 0.001  # m - minimum penetration for contact


@dataclass
class ForceReading:
    """Force sensor reading"""
    force: np.ndarray = field(default_factory=lambda: np.zeros(3))  # N
    torque: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Nm
    timestamp: float = 0.0
    contact_state: ContactState = ContactState.NO_CONTACT
    penetration_depth: float = 0.0  # m
    contact_normal: np.ndarray = field(default_factory=lambda: np.array([0, 0, 1]))


@dataclass
class ContactPoint:
    """Individual contact point data"""
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))  # m
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # m/s
    normal_force: float = 0.0     # N
    friction_force: np.ndarray = field(default_factory=lambda: np.zeros(2))  # N (x,y)
    penetration: float = 0.0      # m
    contact_state: ContactState = ContactState.NO_CONTACT
    sticking_threshold: float = 0.001  # m/s - velocity threshold for sticking


class GroundModel(ABC):
    """Abstract base class for ground models"""
    
    @abstractmethod
    def calculate_contact_force(self, position: np.ndarray, velocity: np.ndarray, 
                              properties: ContactProperties) -> Tuple[np.ndarray, ContactState]:
        """Calculate contact force and state"""
        pass


class SpringDamperGround(GroundModel):
    """Spring-damper ground model with friction"""
    
    def __init__(self, ground_height: float = 0.0):
        self.ground_height = ground_height
        
    def calculate_contact_force(self, position: np.ndarray, velocity: np.ndarray,
                              properties: ContactProperties) -> Tuple[np.ndarray, ContactState]:
        """
        Calculate contact force using spring-damper model
        
        Args:
            position: Contact point position [x, y, z] in meters
            velocity: Contact point velocity [vx, vy, vz] in m/s
            properties: Contact properties
            
        Returns:
            Tuple of (contact_force, contact_state)
        """
        # Calculate penetration depth
        penetration = self.ground_height - position[2]
        
        if penetration <= properties.penetration_threshold:
            return np.zeros(3), ContactState.NO_CONTACT
            
        # Normal force (spring-damper)
        normal_velocity = -velocity[2]  # Velocity into ground
        normal_force = (properties.stiffness * penetration + 
                       properties.damping * normal_velocity)
        normal_force = max(0.0, normal_force)  # No pulling forces
        
        # Tangential velocity
        tangential_velocity = velocity[:2]
        tangential_speed = np.linalg.norm(tangential_velocity)
        
        # Friction force calculation
        if tangential_speed < properties.penetration_threshold:  # Sticking
            friction_force = np.zeros(2)
            contact_state = ContactState.STICKING
        else:
            # Sliding friction
            friction_direction = -tangential_velocity / tangential_speed
            friction_magnitude = properties.kinetic_friction * normal_force
            friction_force = friction_magnitude * friction_direction
            contact_state = ContactState.SLIDING
            
        # Combine forces
        total_force = np.array([friction_force[0], friction_force[1], normal_force])
        
        return total_force, contact_state


class ForceSensor:
    """Force sensor simulation with noise and filtering"""
    
    def __init__(self, noise_std: float = 0.1, filter_cutoff: float = 50.0):
        """
        Initialize force sensor
        
        Args:
            noise_std: Standard deviation of sensor noise (N)
            filter_cutoff: Low-pass filter cutoff frequency (Hz)
        """
        self.noise_std = noise_std
        self.filter_cutoff = filter_cutoff
        self.last_reading = ForceReading()
        self.filter_alpha = 0.1  # Low-pass filter coefficient
        
    def add_noise(self, force: np.ndarray) -> np.ndarray:
        """Add realistic sensor noise"""
        noise = np.random.normal(0, self.noise_std, size=force.shape)
        return force + noise
        
    def apply_filter(self, new_force: np.ndarray, last_force: np.ndarray) -> np.ndarray:
        """Apply low-pass filter to reduce noise"""
        return self.filter_alpha * new_force + (1 - self.filter_alpha) * last_force
        
    def read_force(self, true_force: np.ndarray, contact_state: ContactState,
                   penetration: float = 0.0) -> ForceReading:
        """
        Simulate force sensor reading
        
        Args:
            true_force: Actual contact force
            contact_state: Current contact state
            penetration: Penetration depth
            
        Returns:
            Force sensor reading with noise and filtering
        """
        # Add noise
        noisy_force = self.add_noise(true_force)
        
        # Apply filtering
        filtered_force = self.apply_filter(noisy_force, self.last_reading.force)
        
        # Create reading
        reading = ForceReading(
            force=filtered_force,
            torque=np.zeros(3),  # Simplified - no torque sensing for now
            timestamp=time.time(),
            contact_state=contact_state,
            penetration_depth=penetration,
            contact_normal=np.array([0, 0, 1])
        )
        
        self.last_reading = reading
        return reading


class ContactForceEstimator:
    """Main contact force estimation system"""
    
    def __init__(self, kinematics: HexapodKinematics, body_dynamics: BodyDynamics,
                 contact_properties: ContactProperties = None):
        """
        Initialize contact force estimator
        
        Args:
            kinematics: Hexapod kinematics system
            body_dynamics: Body dynamics system
            contact_properties: Ground contact properties
        """
        self.kinematics = kinematics
        self.body_dynamics = body_dynamics
        self.contact_properties = contact_properties or ContactProperties()
        
        # Ground model
        self.ground_model = SpringDamperGround()
        
        # Force sensors for each leg
        self.force_sensors = [ForceSensor() for _ in range(6)]
        
        # Contact tracking
        self.contact_points = [ContactPoint() for _ in range(6)]
        self.contact_history = []
        
        # Performance tracking
        self.computation_times = {
            'total': [],
            'contact_detection': [],
            'force_calculation': [],
            'sensor_simulation': []
        }
        
    def detect_contact(self, leg_id: int, foot_position: np.ndarray,
                      foot_velocity: np.ndarray) -> Tuple[bool, float]:
        """
        Detect contact for a specific leg
        
        Args:
            leg_id: Leg identifier (0-5)
            foot_position: Foot position in world coordinates
            foot_velocity: Foot velocity in world coordinates
            
        Returns:
            Tuple of (is_in_contact, penetration_depth)
        """
        start_time = time.perf_counter()
        
        # Check if foot is below ground level
        penetration = self.ground_model.ground_height - foot_position[2]
        is_in_contact = penetration > self.contact_properties.penetration_threshold
        
        # Update contact point
        self.contact_points[leg_id].position = foot_position.copy()
        self.contact_points[leg_id].velocity = foot_velocity.copy()
        self.contact_points[leg_id].penetration = max(0.0, penetration)
        
        self.computation_times['contact_detection'].append(
            (time.perf_counter() - start_time) * 1000
        )
        
        return is_in_contact, max(0.0, penetration)
        
    def calculate_contact_forces(self, leg_id: int) -> np.ndarray:
        """
        Calculate contact forces for a specific leg
        
        Args:
            leg_id: Leg identifier (0-5)
            
        Returns:
            Contact force vector [fx, fy, fz] in Newtons
        """
        start_time = time.perf_counter()
        
        contact_point = self.contact_points[leg_id]
        
        # Calculate force using ground model
        force, contact_state = self.ground_model.calculate_contact_force(
            contact_point.position,
            contact_point.velocity,
            self.contact_properties
        )
        
        # Update contact point state
        contact_point.normal_force = force[2]
        contact_point.friction_force = force[:2]
        contact_point.contact_state = contact_state
        
        self.computation_times['force_calculation'].append(
            (time.perf_counter() - start_time) * 1000
        )
        
        return force
        
    def simulate_force_sensor(self, leg_id: int, true_force: np.ndarray) -> ForceReading:
        """
        Simulate force sensor reading for a leg
        
        Args:
            leg_id: Leg identifier (0-5)
            true_force: True contact force
            
        Returns:
            Simulated force sensor reading
        """
        start_time = time.perf_counter()
        
        contact_point = self.contact_points[leg_id]
        
        # Get sensor reading
        reading = self.force_sensors[leg_id].read_force(
            true_force,
            contact_point.contact_state,
            contact_point.penetration
        )
        
        self.computation_times['sensor_simulation'].append(
            (time.perf_counter() - start_time) * 1000
        )
        
        return reading
        
    def estimate_all_contact_forces(self, joint_angles: np.ndarray,
                                  joint_velocities: np.ndarray) -> Tuple[List[np.ndarray], List[ForceReading]]:
        """
        Estimate contact forces for all legs
        
        Args:
            joint_angles: All joint angles [6x3] in radians
            joint_velocities: All joint velocities [6x3] in rad/s
            
        Returns:
            Tuple of (contact_forces, sensor_readings)
        """
        start_time = time.perf_counter()
        
        contact_forces = []
        sensor_readings = []
        
        for leg_id in range(6):
            # Get foot position and velocity
            leg_id_str = self.kinematics.LEG_NAMES[leg_id]  # Use correct leg naming
            joint_angles_deg = np.degrees(joint_angles[leg_id])
            foot_pos = self.kinematics.forward_kinematics(joint_angles_deg, leg_id_str)
            
            # Calculate foot velocity using simple approximation
            # For now, we'll use a simple linear approximation based on joint velocities
            # This would ideally use the Jacobian matrix
            foot_vel = np.zeros(3)  # Simplified for initial implementation
            
            # Detect contact
            is_in_contact, penetration = self.detect_contact(leg_id, foot_pos, foot_vel)
            
            if is_in_contact:
                # Calculate contact forces
                force = self.calculate_contact_forces(leg_id)
                
                # Simulate sensor reading
                reading = self.simulate_force_sensor(leg_id, force)
            else:
                # No contact
                force = np.zeros(3)
                reading = ForceReading(
                    timestamp=time.time(),
                    contact_state=ContactState.NO_CONTACT
                )
                
            contact_forces.append(force)
            sensor_readings.append(reading)
            
        # Update performance tracking
        total_time = (time.perf_counter() - start_time) * 1000
        self.computation_times['total'].append(total_time)
        
        return contact_forces, sensor_readings
        
    def get_stance_legs(self, sensor_readings: List[ForceReading],
                       force_threshold: float = 5.0) -> List[int]:
        """
        Determine which legs are in stance phase based on force readings
        
        Args:
            sensor_readings: Force sensor readings for all legs
            force_threshold: Minimum normal force to consider leg in stance (N)
            
        Returns:
            List of leg IDs in stance phase
        """
        stance_legs = []
        
        for leg_id, reading in enumerate(sensor_readings):
            if (reading.contact_state != ContactState.NO_CONTACT and 
                reading.force[2] > force_threshold):
                stance_legs.append(leg_id)
                
        return stance_legs
        
    def calculate_ground_reaction_force(self, contact_forces: List[np.ndarray]) -> np.ndarray:
        """
        Calculate total ground reaction force
        
        Args:
            contact_forces: Contact forces for all legs
            
        Returns:
            Total ground reaction force [fx, fy, fz] in Newtons
        """
        return np.sum(contact_forces, axis=0)
        
    def estimate_friction_coefficients(self, sensor_readings: List[ForceReading]) -> Dict[str, float]:
        """
        Estimate friction coefficients from sensor data
        
        Args:
            sensor_readings: Force sensor readings
            
        Returns:
            Dictionary with estimated friction coefficients
        """
        tangential_forces = []
        normal_forces = []
        
        for reading in sensor_readings:
            if reading.contact_state in [ContactState.SLIDING, ContactState.STICKING]:
                normal_force = reading.force[2]
                tangential_force = np.linalg.norm(reading.force[:2])
                
                if normal_force > 1.0:  # Avoid division by small numbers
                    tangential_forces.append(tangential_force)
                    normal_forces.append(normal_force)
                    
        if len(tangential_forces) > 0:
            friction_ratios = np.array(tangential_forces) / np.array(normal_forces)
            estimated_friction = np.mean(friction_ratios)
        else:
            estimated_friction = self.contact_properties.kinetic_friction
            
        return {
            'estimated_kinetic': estimated_friction,
            'configured_kinetic': self.contact_properties.kinetic_friction,
            'configured_static': self.contact_properties.static_friction
        }
        
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get performance metrics for contact force estimation"""
        metrics = {}
        
        for key, times in self.computation_times.items():
            if times:
                metrics[f'{key}_avg_ms'] = np.mean(times)
                metrics[f'{key}_max_ms'] = np.max(times)
                metrics[f'{key}_std_ms'] = np.std(times)
                
        return metrics
        
    def reset_performance_tracking(self):
        """Reset performance tracking data"""
        for key in self.computation_times:
            self.computation_times[key].clear()
            
    def update_contact_properties(self, **kwargs):
        """Update contact properties"""
        for key, value in kwargs.items():
            if hasattr(self.contact_properties, key):
                setattr(self.contact_properties, key, value)
                
    def get_contact_summary(self) -> Dict[str, Any]:
        """Get summary of current contact state"""
        summary = {
            'total_legs': 6,
            'legs_in_contact': 0,
            'total_normal_force': 0.0,
            'total_friction_force': 0.0,
            'contact_states': {},
            'average_penetration': 0.0
        }
        
        penetrations = []
        
        for leg_id, contact_point in enumerate(self.contact_points):
            state = contact_point.contact_state.name
            if state in summary['contact_states']:
                summary['contact_states'][state] += 1
            else:
                summary['contact_states'][state] = 1
                
            if contact_point.contact_state != ContactState.NO_CONTACT:
                summary['legs_in_contact'] += 1
                summary['total_normal_force'] += contact_point.normal_force
                summary['total_friction_force'] += np.linalg.norm(contact_point.friction_force)
                penetrations.append(contact_point.penetration)
                
        if penetrations:
            summary['average_penetration'] = np.mean(penetrations)
            
        return summary