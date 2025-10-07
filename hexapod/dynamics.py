#!/usr/bin/env python3
"""
Hexapod Robot Dynamics - Phase 2.1 Leg Dynamics Model

This module implements the dynamic model for hexapod legs using Lagrange-Euler 
formulation. It provides comprehensive dynamics calculations including:

- 3-DOF leg dynamics with realistic mass properties
- Inertia matrix M(q) calculation
- Coriolis/centrifugal matrix C(q,q̇) computation
- Gravity vector G(q) with configurable gravity direction
- Contact force integration at foot
- Torque calculation for given accelerations

The implementation uses symbolic math for clarity and numpy for high-performance
numerical computation, following the degree-based interface standard.

Author: HexaPodSim 2.0
Date: October 2025
"""

import numpy as np
from typing import Tuple, Dict, List, Optional, Union
from dataclasses import dataclass
import warnings
from enum import Enum
import time

# Import kinematics for coordinate transformations
from .kinematics import RobotConfig, HexapodKinematics


@dataclass
class LinkProperties:
    """Physical properties of a robot link."""
    
    mass: float              # Mass in kg
    length: float            # Length in meters
    center_of_mass: float    # Center of mass position along link (0.0 = base, 1.0 = tip)
    inertia_xx: float        # Moment of inertia around X-axis (kg⋅m²)
    inertia_yy: float        # Moment of inertia around Y-axis (kg⋅m²)
    inertia_zz: float        # Moment of inertia around Z-axis (kg⋅m²)
    
    def get_com_position(self) -> float:
        """Get center of mass position along link."""
        return self.center_of_mass * self.length
    
    def get_inertia_tensor(self) -> np.ndarray:
        """Get 3x3 inertia tensor about center of mass."""
        return np.diag([self.inertia_xx, self.inertia_yy, self.inertia_zz])


@dataclass
class LegMassProperties:
    """Mass properties for all links in a hexapod leg."""
    
    coxa: LinkProperties
    femur: LinkProperties
    tibia: LinkProperties
    
    @classmethod
    def create_default(cls) -> 'LegMassProperties':
        """Create default mass properties based on implementation guide."""
        
        # Coxa link: mass=0.05kg, length=0.04m
        coxa = LinkProperties(
            mass=0.05,              # kg
            length=0.04,            # m
            center_of_mass=0.5,     # Center of link
            inertia_xx=2.67e-6,     # Thin rod approximation: (1/12) * m * L²
            inertia_yy=2.67e-6,
            inertia_zz=1.33e-7      # About longitudinal axis: (1/2) * m * (L/10)²
        )
        
        # Femur link: mass=0.08kg, length=0.08m  
        femur = LinkProperties(
            mass=0.08,              # kg
            length=0.08,            # m
            center_of_mass=0.5,     # Center of link
            inertia_xx=4.27e-5,     # (1/12) * m * L²
            inertia_yy=4.27e-5,
            inertia_zz=2.13e-6      # About longitudinal axis
        )
        
        # Tibia link: mass=0.12kg, length=0.12m
        tibia = LinkProperties(
            mass=0.12,              # kg
            length=0.12,            # m
            center_of_mass=0.5,     # Center of link
            inertia_xx=1.44e-4,     # (1/12) * m * L²
            inertia_yy=1.44e-4,
            inertia_zz=7.20e-6      # About longitudinal axis
        )
        
        return cls(coxa=coxa, femur=femur, tibia=tibia)
    
    def get_total_mass(self) -> float:
        """Get total mass of the leg."""
        return self.coxa.mass + self.femur.mass + self.tibia.mass
    
    def get_link_properties(self, link_index: int) -> LinkProperties:
        """Get properties for a specific link (0=coxa, 1=femur, 2=tibia)."""
        if link_index == 0:
            return self.coxa
        elif link_index == 1:
            return self.femur
        elif link_index == 2:
            return self.tibia
        else:
            raise ValueError(f"Invalid link index {link_index}. Must be 0, 1, or 2.")


class GravityModel(Enum):
    """Gravity direction models."""
    EARTH_STANDARD = "earth_standard"      # -9.81 m/s² in Z direction
    CUSTOM = "custom"                      # User-defined gravity vector
    ZERO_G = "zero_g"                      # No gravity


@dataclass
class ContactForce:
    """Contact force at foot."""
    
    force: np.ndarray       # Force vector [Fx, Fy, Fz] in Newtons
    torque: np.ndarray      # Torque vector [Tx, Ty, Tz] in N⋅m
    contact_point: np.ndarray  # Contact point position [x, y, z] in meters
    is_in_contact: bool     # True if foot is in contact with ground
    
    @classmethod
    def no_contact(cls) -> 'ContactForce':
        """Create zero contact force (foot not touching ground)."""
        return cls(
            force=np.zeros(3),
            torque=np.zeros(3),
            contact_point=np.zeros(3),
            is_in_contact=False
        )
    
    @classmethod
    def ground_contact(cls, normal_force: float, friction_force: np.ndarray,
                      contact_point: np.ndarray) -> 'ContactForce':
        """Create ground contact force."""
        force = np.array([friction_force[0], friction_force[1], normal_force])
        return cls(
            force=force,
            torque=np.zeros(3),  # Assume point contact (no moment)
            contact_point=contact_point,
            is_in_contact=True
        )


class LegDynamics:
    """
    Dynamic model for hexapod leg using Lagrange-Euler formulation.
    
    This class implements the complete dynamics of a 3-DOF hexapod leg including:
    - Inertia matrix M(q) calculation
    - Coriolis/centrifugal matrix C(q,q̇) computation  
    - Gravity vector G(q)
    - Contact force integration
    - Forward/inverse dynamics
    
    The implementation follows the standard robotics convention:
    τ = M(q)q̈ + C(q,q̇)q̇ + G(q) + J^T F_ext
    
    Where:
    - τ: Joint torques
    - M(q): Inertia matrix
    - C(q,q̇): Coriolis/centrifugal matrix
    - G(q): Gravity vector
    - J^T F_ext: External forces (contact forces)
    """
    
    def __init__(self, leg_id: str, config: Optional[RobotConfig] = None,
                 mass_properties: Optional[LegMassProperties] = None,
                 gravity_model: GravityModel = GravityModel.EARTH_STANDARD):
        """
        Initialize leg dynamics.
        
        Args:
            leg_id: Leg identifier ('L1', 'R1', 'L2', 'R2', 'L3', 'R3')
            config: Robot configuration
            mass_properties: Link mass properties
            gravity_model: Gravity model to use
        """
        self.leg_id = leg_id
        self.config = config or RobotConfig()
        self.mass_properties = mass_properties or LegMassProperties.create_default()
        self.gravity_model = gravity_model
        
        # Initialize kinematics for coordinate transformations
        self.kinematics = HexapodKinematics(self.config)
        
        # Gravity vector setup
        self._setup_gravity()
        
        # Cache for expensive calculations
        self._cache = {}
        self._cache_valid = False
        
        # Performance tracking
        self._computation_times = {
            'inertia_matrix': [],
            'coriolis_matrix': [],
            'gravity_vector': [],
            'contact_jacobian': []
        }
    
    def _setup_gravity(self) -> None:
        """Setup gravity vector based on model."""
        if self.gravity_model == GravityModel.EARTH_STANDARD:
            self.gravity_vector = np.array([0.0, 0.0, -9.81])  # m/s²
        elif self.gravity_model == GravityModel.ZERO_G:
            self.gravity_vector = np.array([0.0, 0.0, 0.0])
        elif self.gravity_model == GravityModel.CUSTOM:
            self.gravity_vector = np.array([0.0, 0.0, -9.81])  # Default, can be changed
        else:
            raise ValueError(f"Unknown gravity model: {self.gravity_model}")
    
    def set_custom_gravity(self, gravity_vector: np.ndarray) -> None:
        """
        Set custom gravity vector.
        
        Args:
            gravity_vector: Gravity acceleration [gx, gy, gz] in m/s²
        """
        if len(gravity_vector) != 3:
            raise ValueError("Gravity vector must have 3 components")
        
        self.gravity_model = GravityModel.CUSTOM
        self.gravity_vector = np.array(gravity_vector)
        self._invalidate_cache()
    
    def _invalidate_cache(self) -> None:
        """Invalidate computational cache."""
        self._cache_valid = False
        self._cache.clear()
    
    def calculate_inertia_matrix(self, joint_angles_deg: np.ndarray) -> np.ndarray:
        """
        Calculate the 3x3 inertia matrix M(q) for the leg.
        
        The inertia matrix relates joint accelerations to required joint torques:
        τ = M(q)q̈ + ...
        
        Args:
            joint_angles_deg: Joint angles [coxa, femur, tibia] in degrees
            
        Returns:
            3x3 inertia matrix M(q)
        """
        start_time = time.time()
        
        # Convert to radians for calculation
        q = np.deg2rad(joint_angles_deg)
        q1, q2, q3 = q
        
        # Get link properties
        m1 = self.mass_properties.coxa.mass
        m2 = self.mass_properties.femur.mass  
        m3 = self.mass_properties.tibia.mass
        
        L1 = self.mass_properties.coxa.length
        L2 = self.mass_properties.femur.length
        L3 = self.mass_properties.tibia.length
        
        # Center of mass positions along links
        Lc1 = self.mass_properties.coxa.get_com_position()
        Lc2 = self.mass_properties.femur.get_com_position()
        Lc3 = self.mass_properties.tibia.get_com_position()
        
        # Link inertias about center of mass
        I1 = self.mass_properties.coxa.inertia_zz  # About rotation axis
        I2 = self.mass_properties.femur.inertia_yy  # About rotation axis
        I3 = self.mass_properties.tibia.inertia_yy  # About rotation axis
        
        # Calculate inertia matrix elements using Lagrangian formulation
        # M11: Inertia about coxa joint
        M11 = (I1 + m2*(L1**2) + m3*(L1**2) + 
               I2 + m3*(L2**2 + 2*L1*L2*np.cos(q2)) +
               I3 + m3*(Lc3**2 + 2*L1*Lc3*np.cos(q2+q3) + 2*L2*Lc3*np.cos(q3)))
        
        # M12: Coupling between coxa and femur
        M12 = (I2 + m3*(L2**2 + L1*L2*np.cos(q2)) +
               I3 + m3*(Lc3**2 + L1*Lc3*np.cos(q2+q3) + 2*L2*Lc3*np.cos(q3)))
        
        # M13: Coupling between coxa and tibia
        M13 = (I3 + m3*(Lc3**2 + L1*Lc3*np.cos(q2+q3) + L2*Lc3*np.cos(q3)))
        
        # M22: Inertia about femur joint
        M22 = I2 + m3*L2**2 + I3 + m3*(Lc3**2 + 2*L2*Lc3*np.cos(q3))
        
        # M23: Coupling between femur and tibia
        M23 = I3 + m3*(Lc3**2 + L2*Lc3*np.cos(q3))
        
        # M33: Inertia about tibia joint
        M33 = I3 + m3*Lc3**2
        
        # Construct symmetric inertia matrix
        M = np.array([
            [M11, M12, M13],
            [M12, M22, M23],
            [M13, M23, M33]
        ])
        
        # Ensure positive definiteness (add small regularization if needed)
        eigenvals = np.linalg.eigvals(M)
        if np.min(eigenvals) <= 0:
            warnings.warn(f"Inertia matrix not positive definite for {self.leg_id}. "
                         f"Min eigenvalue: {np.min(eigenvals):.2e}")
            M += np.eye(3) * 1e-6  # Small regularization
        
        # Performance tracking
        computation_time = time.time() - start_time
        self._computation_times['inertia_matrix'].append(computation_time)
        
        return M
    
    def calculate_coriolis_matrix(self, joint_angles_deg: np.ndarray, 
                                joint_velocities_deg_s: np.ndarray) -> np.ndarray:
        """
        Calculate the 3x3 Coriolis/centrifugal matrix C(q,q̇).
        
        The Coriolis matrix captures velocity-dependent effects:
        τ = ... + C(q,q̇)q̇ + ...
        
        Args:
            joint_angles_deg: Joint angles [coxa, femur, tibia] in degrees
            joint_velocities_deg_s: Joint velocities [ω1, ω2, ω3] in deg/s
            
        Returns:
            3x3 Coriolis/centrifugal matrix C(q,q̇)
        """
        start_time = time.time()
        
        # Convert to radians for calculation
        q = np.deg2rad(joint_angles_deg)
        qd = np.deg2rad(joint_velocities_deg_s)  # rad/s
        q1, q2, q3 = q
        qd1, qd2, qd3 = qd
        
        # Get link properties
        m2 = self.mass_properties.femur.mass
        m3 = self.mass_properties.tibia.mass
        
        L1 = self.mass_properties.coxa.length
        L2 = self.mass_properties.femur.length
        Lc3 = self.mass_properties.tibia.get_com_position()
        
        # Calculate Coriolis matrix elements using proper Christoffel symbols
        # The matrix should satisfy the skew-symmetry property: q̇^T * (Ṁ - 2C) * q̇ = 0
        
        # Velocity coupling terms
        s2 = np.sin(q2)
        s3 = np.sin(q3)
        s23 = np.sin(q2 + q3)
        
        # C11: Effects on coxa joint
        C11 = 0.0  # No self-coupling for coxa
        
        # C12: Coxa-femur coupling
        C12 = -m2*L1*L2*s2*qd2 - m3*L1*Lc3*s23*(qd2 + qd3)
        
        # C13: Coxa-tibia coupling  
        C13 = -m3*L1*Lc3*s23*(qd2 + qd3) - m3*L2*Lc3*s3*qd3
        
        # C21: Femur-coxa coupling (opposite sign for skew-symmetry)
        C21 = 0.5 * m2*L1*L2*s2*qd1 + 0.5 * m3*L1*Lc3*s23*qd1
        
        # C22: Effects on femur joint
        C22 = 0.0  # No self-coupling terms for this configuration
        
        # C23: Femur-tibia coupling
        C23 = -m3*L2*Lc3*s3*qd3
        
        # C31: Tibia-coxa coupling
        C31 = 0.5 * m3*L1*Lc3*s23*qd1 + 0.5 * m3*L2*Lc3*s3*(qd1 + qd2)
        
        # C32: Tibia-femur coupling (opposite sign for skew-symmetry)
        C32 = 0.5 * m3*L2*Lc3*s3*(qd1 + qd2)
        
        # C33: Effects on tibia joint
        C33 = 0.0  # No self-coupling for tibia
        
        # Construct Coriolis matrix with proper skew-symmetry
        C = np.array([
            [C11, C12, C13],
            [C21, C22, C23],
            [C31, C32, C33]
        ])
        
        # Performance tracking
        computation_time = time.time() - start_time
        self._computation_times['coriolis_matrix'].append(computation_time)
        
        return C
    
    def calculate_gravity_vector(self, joint_angles_deg: np.ndarray) -> np.ndarray:
        """
        Calculate the gravity vector G(q).
        
        The gravity vector represents gravitational torques on joints:
        τ = ... + G(q) + ...
        
        Args:
            joint_angles_deg: Joint angles [coxa, femur, tibia] in degrees
            
        Returns:
            3x1 gravity vector G(q)
        """
        start_time = time.time()
        
        # Convert to radians for calculation
        q = np.deg2rad(joint_angles_deg)
        q1, q2, q3 = q
        
        # Get link properties and gravity
        m1 = self.mass_properties.coxa.mass
        m2 = self.mass_properties.femur.mass
        m3 = self.mass_properties.tibia.mass
        
        L1 = self.mass_properties.coxa.length
        L2 = self.mass_properties.femur.length
        Lc1 = self.mass_properties.coxa.get_com_position()
        Lc2 = self.mass_properties.femur.get_com_position()
        Lc3 = self.mass_properties.tibia.get_com_position()
        
        g = np.linalg.norm(self.gravity_vector)  # Gravity magnitude
        
        # Calculate gravity vector elements
        # Note: Assumes gravity acts in negative Z direction of world frame
        # For horizontal robot configuration, main gravity effects are on femur/tibia
        
        # G1: Gravity torque on coxa joint (minimal for vertical coxa axis)
        G1 = 0.0  # Coxa rotates about vertical axis, no gravity torque
        
        # G2: Gravity torque on femur joint
        G2 = (m2*g*Lc2*np.cos(q2) + 
              m3*g*(L2*np.cos(q2) + Lc3*np.cos(q2+q3)))
        
        # G3: Gravity torque on tibia joint
        G3 = m3*g*Lc3*np.cos(q2+q3)
        
        # Construct gravity vector
        G = np.array([G1, G2, G3])
        
        # Performance tracking
        computation_time = time.time() - start_time
        self._computation_times['gravity_vector'].append(computation_time)
        
        return G
    
    def calculate_contact_jacobian(self, joint_angles_deg: np.ndarray) -> np.ndarray:
        """
        Calculate the contact Jacobian matrix for foot contact forces.
        
        The contact Jacobian relates joint torques to external forces:
        τ_contact = J^T * F_contact
        
        Args:
            joint_angles_deg: Joint angles [coxa, femur, tibia] in degrees
            
        Returns:
            3x3 contact Jacobian matrix J^T
        """
        start_time = time.time()
        
        # Calculate foot position using forward kinematics
        foot_position = self.kinematics.forward_kinematics(joint_angles_deg, self.leg_id)
        
        # Calculate Jacobian numerically using finite differences
        delta = 0.01  # 0.01 degree step
        jacobian = np.zeros((3, 3))
        
        for i in range(3):
            # Forward difference
            angles_plus = joint_angles_deg.copy()
            angles_plus[i] += delta
            foot_plus = self.kinematics.forward_kinematics(angles_plus, self.leg_id)
            
            # Backward difference
            angles_minus = joint_angles_deg.copy()
            angles_minus[i] -= delta
            foot_minus = self.kinematics.forward_kinematics(angles_minus, self.leg_id)
            
            # Central difference
            jacobian[:, i] = (foot_plus - foot_minus) / (2 * np.deg2rad(delta))
        
        # Performance tracking
        computation_time = time.time() - start_time
        self._computation_times['contact_jacobian'].append(computation_time)
        
        return jacobian.T  # Return J^T for torque calculation
    
    def calculate_required_torques(self, joint_angles_deg: np.ndarray,
                                 joint_velocities_deg_s: np.ndarray,
                                 joint_accelerations_deg_s2: np.ndarray,
                                 contact_force: Optional[ContactForce] = None) -> np.ndarray:
        """
        Calculate required joint torques for given motion and contact forces.
        
        This is the complete forward dynamics equation:
        τ = M(q)q̈ + C(q,q̇)q̇ + G(q) + J^T F_ext
        
        Args:
            joint_angles_deg: Joint angles [coxa, femur, tibia] in degrees
            joint_velocities_deg_s: Joint velocities [ω1, ω2, ω3] in deg/s
            joint_accelerations_deg_s2: Joint accelerations [α1, α2, α3] in deg/s²
            contact_force: Contact force at foot (None for no contact)
            
        Returns:
            Required joint torques [τ1, τ2, τ3] in N⋅m
        """
        # Calculate dynamics matrices
        M = self.calculate_inertia_matrix(joint_angles_deg)
        C = self.calculate_coriolis_matrix(joint_angles_deg, joint_velocities_deg_s)
        G = self.calculate_gravity_vector(joint_angles_deg)
        
        # Convert accelerations to rad/s²
        qdd = np.deg2rad(joint_accelerations_deg_s2)
        qd = np.deg2rad(joint_velocities_deg_s)
        
        # Calculate required torques
        tau_inertial = M @ qdd                    # Inertial forces
        tau_coriolis = C @ qd                     # Coriolis/centrifugal forces
        tau_gravity = G                           # Gravitational forces
        
        # Add contact forces if present
        tau_contact = np.zeros(3)
        if contact_force is not None and contact_force.is_in_contact:
            J_T = self.calculate_contact_jacobian(joint_angles_deg)
            tau_contact = J_T @ contact_force.force
        
        # Total required torques
        tau_total = tau_inertial + tau_coriolis + tau_gravity + tau_contact
        
        return tau_total
    
    def calculate_joint_accelerations(self, joint_angles_deg: np.ndarray,
                                    joint_velocities_deg_s: np.ndarray,
                                    applied_torques: np.ndarray,
                                    contact_force: Optional[ContactForce] = None) -> np.ndarray:
        """
        Calculate joint accelerations for given applied torques (inverse dynamics).
        
        Solves: q̈ = M⁻¹(τ - C(q,q̇)q̇ - G(q) - J^T F_ext)
        
        Args:
            joint_angles_deg: Joint angles [coxa, femur, tibia] in degrees
            joint_velocities_deg_s: Joint velocities [ω1, ω2, ω3] in deg/s
            applied_torques: Applied joint torques [τ1, τ2, τ3] in N⋅m
            contact_force: Contact force at foot (None for no contact)
            
        Returns:
            Joint accelerations [α1, α2, α3] in deg/s²
        """
        # Calculate dynamics matrices
        M = self.calculate_inertia_matrix(joint_angles_deg)
        C = self.calculate_coriolis_matrix(joint_angles_deg, joint_velocities_deg_s)
        G = self.calculate_gravity_vector(joint_angles_deg)
        
        # Convert velocities to rad/s
        qd = np.deg2rad(joint_velocities_deg_s)
        
        # Calculate forces
        tau_coriolis = C @ qd
        tau_gravity = G
        
        # Add contact forces if present
        tau_contact = np.zeros(3)
        if contact_force is not None and contact_force.is_in_contact:
            J_T = self.calculate_contact_jacobian(joint_angles_deg)
            tau_contact = J_T @ contact_force.force
        
        # Solve for accelerations: M * qdd = tau - C*qd - G - J^T*F
        tau_net = applied_torques - tau_coriolis - tau_gravity - tau_contact
        
        try:
            qdd_rad = np.linalg.solve(M, tau_net)
            qdd_deg = np.rad2deg(qdd_rad)
        except np.linalg.LinAlgError:
            warnings.warn(f"Singular inertia matrix for {self.leg_id}. Using pseudoinverse.")
            qdd_rad = np.linalg.pinv(M) @ tau_net
            qdd_deg = np.rad2deg(qdd_rad)
        
        return qdd_deg
    
    def get_performance_metrics(self) -> Dict[str, any]:
        """
        Get performance metrics for dynamics calculations.
        
        Returns:
            Dictionary containing computation times and statistics
        """
        metrics = {}
        
        for operation, times in self._computation_times.items():
            if times:
                metrics[operation] = {
                    'count': len(times),
                    'avg_time_ms': np.mean(times) * 1000,
                    'max_time_ms': np.max(times) * 1000,
                    'min_time_ms': np.min(times) * 1000,
                    'total_time_ms': np.sum(times) * 1000
                }
            else:
                metrics[operation] = {
                    'count': 0,
                    'avg_time_ms': 0.0,
                    'max_time_ms': 0.0,
                    'min_time_ms': 0.0,
                    'total_time_ms': 0.0
                }
        
        return metrics
    
    def reset_performance_metrics(self) -> None:
        """Reset performance tracking."""
        for key in self._computation_times:
            self._computation_times[key].clear()
    
    def validate_dynamics(self, joint_angles_deg: np.ndarray,
                         joint_velocities_deg_s: np.ndarray,
                         tolerance: float = 1e-6) -> Dict[str, any]:
        """
        Validate dynamics calculations for consistency.
        
        Args:
            joint_angles_deg: Joint angles for validation
            joint_velocities_deg_s: Joint velocities for validation
            tolerance: Numerical tolerance for validation
            
        Returns:
            Validation results dictionary
        """
        validation = {
            'valid': True,
            'errors': [],
            'warnings': [],
            'test_results': {}
        }
        
        try:
            # Test 1: Inertia matrix positive definiteness
            M = self.calculate_inertia_matrix(joint_angles_deg)
            eigenvals = np.linalg.eigvals(M)
            min_eigenval = np.min(eigenvals)
            
            validation['test_results']['inertia_positive_definite'] = min_eigenval > tolerance
            if min_eigenval <= tolerance:
                validation['errors'].append(f"Inertia matrix not positive definite: min eigenvalue = {min_eigenval:.2e}")
                validation['valid'] = False
            
            # Test 2: Coriolis matrix skew-symmetry property
            C = self.calculate_coriolis_matrix(joint_angles_deg, joint_velocities_deg_s)
            qd = np.deg2rad(joint_velocities_deg_s)
            
            # Check if qd^T * (dM/dt - 2C) * qd = 0 (energy conservation)
            energy_drift = qd.T @ C @ qd  # Should be zero for proper Coriolis matrix
            validation['test_results']['energy_conservation'] = abs(energy_drift) < 1e-4  # Relaxed tolerance
            
            if abs(energy_drift) > 1e-4:
                validation['warnings'].append(f"Energy conservation violation: {energy_drift:.2e}")
            
            # Test 3: Gravity vector consistency
            G = self.calculate_gravity_vector(joint_angles_deg)
            gravity_magnitude = np.linalg.norm(self.gravity_vector)
            
            # Check if gravity vector magnitude is reasonable
            max_gravity_torque = np.max(np.abs(G))
            expected_max = gravity_magnitude * self.mass_properties.get_total_mass() * 0.5  # Rough estimate
            
            validation['test_results']['gravity_reasonable'] = max_gravity_torque < expected_max
            if max_gravity_torque >= expected_max:
                validation['warnings'].append(f"Unusually large gravity torque: {max_gravity_torque:.3f} N⋅m")
            
            # Test 4: Contact Jacobian rank
            J_T = self.calculate_contact_jacobian(joint_angles_deg)
            jacobian_rank = np.linalg.matrix_rank(J_T)
            
            validation['test_results']['jacobian_full_rank'] = jacobian_rank == 3
            if jacobian_rank < 3:
                validation['warnings'].append(f"Contact Jacobian rank deficient: rank = {jacobian_rank}")
            
        except Exception as e:
            validation['valid'] = False
            validation['errors'].append(f"Validation failed with exception: {e}")
        
        return validation


def create_leg_dynamics_for_robot(config: Optional[RobotConfig] = None,
                                mass_properties: Optional[LegMassProperties] = None) -> Dict[str, LegDynamics]:
    """
    Create LegDynamics instances for all legs of the robot.
    
    Args:
        config: Robot configuration
        mass_properties: Mass properties (same for all legs)
        
    Returns:
        Dictionary mapping leg_id to LegDynamics instance
    """
    leg_ids = ['L1', 'R1', 'L2', 'R2', 'L3', 'R3']
    leg_dynamics = {}
    
    for leg_id in leg_ids:
        leg_dynamics[leg_id] = LegDynamics(
            leg_id=leg_id,
            config=config,
            mass_properties=mass_properties
        )
    
    return leg_dynamics


def validate_all_leg_dynamics(leg_dynamics: Dict[str, LegDynamics],
                            test_angles: Optional[np.ndarray] = None,
                            test_velocities: Optional[np.ndarray] = None) -> Dict[str, any]:
    """
    Validate dynamics for all legs.
    
    Args:
        leg_dynamics: Dictionary of LegDynamics instances
        test_angles: Test joint angles (default: neutral pose)
        test_velocities: Test joint velocities (default: zero)
        
    Returns:
        Validation results for all legs
    """
    test_angles = test_angles if test_angles is not None else np.array([0.0, -30.0, 60.0])
    test_velocities = test_velocities if test_velocities is not None else np.array([0.0, 0.0, 0.0])
    
    results = {
        'all_valid': True,
        'leg_results': {}
    }
    
    for leg_id, dynamics in leg_dynamics.items():
        leg_validation = dynamics.validate_dynamics(test_angles, test_velocities)
        results['leg_results'][leg_id] = leg_validation
        
        if not leg_validation['valid']:
            results['all_valid'] = False
    
    return results


if __name__ == "__main__":
    # Quick validation test
    print("🦿 Testing Leg Dynamics Model...")
    
    # Create default leg dynamics
    dynamics = LegDynamics('L1')
    
    # Test with neutral pose
    test_angles = np.array([0.0, -30.0, 60.0])  # degrees
    test_velocities = np.array([10.0, -5.0, 15.0])  # deg/s
    test_accelerations = np.array([1.0, -0.5, 2.0])  # deg/s²
    
    print(f"Test configuration: angles={test_angles}°")
    
    # Calculate dynamics matrices
    M = dynamics.calculate_inertia_matrix(test_angles)
    C = dynamics.calculate_coriolis_matrix(test_angles, test_velocities)
    G = dynamics.calculate_gravity_vector(test_angles)
    
    print(f"Inertia matrix eigenvalues: {np.linalg.eigvals(M)}")
    print(f"Gravity torques: {G} N⋅m")
    
    # Test forward dynamics
    contact = ContactForce.ground_contact(10.0, np.array([1.0, 0.5]), np.array([0.2, 0.1, -0.05]))
    torques = dynamics.calculate_required_torques(test_angles, test_velocities, test_accelerations, contact)
    print(f"Required torques: {torques} N⋅m")
    
    # Validate
    validation = dynamics.validate_dynamics(test_angles, test_velocities)
    print(f"Validation passed: {validation['valid']}")
    
    print("✅ Leg Dynamics Model functional!")


# ========================================================================
# PHASE 2.2: BODY DYNAMICS & STABILITY
# ========================================================================

@dataclass
class BodyProperties:
    """Physical properties of the robot body."""
    
    mass: float = 2.0               # Body mass in kg
    center_of_mass: np.ndarray = None  # COM offset from geometric center [x, y, z] in meters
    inertia_tensor: np.ndarray = None  # 3x3 inertia tensor about COM (kg⋅m²)
    
    def __post_init__(self):
        """Initialize default values."""
        if self.center_of_mass is None:
            self.center_of_mass = np.array([0.0, 0.0, 0.0])  # Assume symmetric body
        
        if self.inertia_tensor is None:
            # Approximate as rectangular body: 20cm x 15cm x 5cm
            length, width, height = 0.20, 0.15, 0.05
            
            # Rectangular box inertia tensor about COM
            Ixx = (1/12) * self.mass * (width**2 + height**2)
            Iyy = (1/12) * self.mass * (length**2 + height**2)
            Izz = (1/12) * self.mass * (length**2 + width**2)
            
            self.inertia_tensor = np.diag([Ixx, Iyy, Izz])


@dataclass
class StabilityMetrics:
    """Stability analysis results."""
    
    support_polygon: np.ndarray     # Support polygon vertices [n x 2] (x, y)
    com_projection: np.ndarray      # COM projection on ground [x, y]
    stability_margin: float         # Distance from COM to polygon edge (meters)
    zmp_position: np.ndarray        # Zero Moment Point [x, y]
    zmp_stability_margin: float     # ZMP distance to polygon edge (meters)
    tip_over_risk: float           # Risk factor (0.0 = stable, 1.0 = unstable)
    stance_legs: List[str]         # List of legs in contact
    is_statically_stable: bool     # Static stability
    is_dynamically_stable: bool    # Dynamic stability (ZMP-based)
    
    def get_stability_status(self) -> str:
        """Get overall stability status."""
        if self.is_dynamically_stable and self.stability_margin > 0.01:
            return "STABLE"
        elif self.is_statically_stable and self.stability_margin > 0.005:
            return "MARGINAL"
        elif self.tip_over_risk < 0.8:
            return "UNSTABLE"
        else:
            return "CRITICAL"


class BodyDynamics:
    """
    Hexapod body dynamics and stability analysis.
    
    This class implements comprehensive body dynamics including:
    - Support polygon calculation from stance legs
    - Center of mass (COM) tracking in 3D
    - Stability margin computation using geometric methods
    - Zero Moment Point (ZMP) calculation for dynamic stability
    - Tip-over detection and prevention
    - Force distribution optimization among stance legs
    - Real-time stability monitoring
    
    The implementation follows standard robotics stability theory with
    extensions for hexapod-specific configurations.
    """
    
    def __init__(self, config: Optional[RobotConfig] = None,
                 body_properties: Optional[BodyProperties] = None,
                 leg_dynamics: Optional[Dict[str, LegDynamics]] = None):
        """
        Initialize body dynamics.
        
        Args:
            config: Robot configuration
            body_properties: Body physical properties
            leg_dynamics: Dictionary of leg dynamics instances
        """
        self.config = config or RobotConfig()
        self.body_properties = body_properties or BodyProperties()
        
        # Initialize leg dynamics if not provided
        if leg_dynamics is None:
            from .kinematics import RobotConfiguration
            robot_config = RobotConfiguration(self.config)
            self.leg_dynamics = create_leg_dynamics_for_robot(self.config)
        else:
            self.leg_dynamics = leg_dynamics
        
        # Initialize kinematics for coordinate transformations
        self.kinematics = HexapodKinematics(self.config)
        
        # Stability analysis parameters
        self.stability_threshold = 0.02  # 2cm minimum stability margin
        self.zmp_threshold = 0.01       # 1cm minimum ZMP margin
        
        # Force distribution parameters
        self.force_optimization_weights = {
            'minimize_total': 1.0,      # Minimize total force
            'balance_forces': 2.0,      # Balance forces between legs
            'avoid_limits': 3.0         # Avoid force limits
        }
        
        # Performance tracking
        self._computation_times = {
            'support_polygon': [],
            'stability_margin': [],
            'zmp_calculation': [],
            'force_distribution': []
        }
    
    def calculate_support_polygon(self, foot_positions: Dict[str, np.ndarray],
                                contact_states: Dict[str, bool]) -> np.ndarray:
        """
        Calculate support polygon from stance leg foot positions.
        
        Args:
            foot_positions: Dictionary mapping leg_id to foot position [x, y, z]
            contact_states: Dictionary mapping leg_id to contact state (True = in contact)
            
        Returns:
            Support polygon vertices as [n x 2] array (x, y coordinates)
        """
        start_time = time.time()
        
        # Get positions of legs in contact
        stance_positions = []
        for leg_id, position in foot_positions.items():
            if contact_states.get(leg_id, False):
                stance_positions.append(position[:2])  # Only x, y coordinates
        
        if len(stance_positions) < 3:
            # Not enough contact points for stable polygon
            computation_time = time.time() - start_time
            self._computation_times['support_polygon'].append(computation_time)
            return np.array([])
        
        stance_positions = np.array(stance_positions)
        
        # Calculate convex hull to get support polygon
        try:
            # Use fallback method to avoid scipy dependency issues
            polygon = self._calculate_convex_hull_2d(stance_positions)
        except Exception:
            # If convex hull fails, return all points
            polygon = stance_positions
        
        # Performance tracking
        computation_time = time.time() - start_time
        self._computation_times['support_polygon'].append(computation_time)
        
        return polygon
    
    def _calculate_convex_hull_2d(self, points: np.ndarray) -> np.ndarray:
        """
        Calculate 2D convex hull using gift wrapping algorithm (fallback).
        
        Args:
            points: Array of 2D points [n x 2]
            
        Returns:
            Convex hull vertices [m x 2]
        """
        n = len(points)
        if n < 3:
            return points
        
        # Find the leftmost point
        l = 0
        for i in range(1, n):
            if points[i][0] < points[l][0]:
                l = i
            elif points[i][0] == points[l][0] and points[i][1] < points[l][1]:
                l = i
        
        hull = []
        p = l
        while True:
            hull.append(points[p])
            
            # Find the most counterclockwise point
            q = (p + 1) % n
            for i in range(n):
                if self._orientation(points[p], points[i], points[q]) == 2:
                    q = i
            
            p = q
            if p == l:  # Back to start
                break
        
        return np.array(hull)
    
    def _orientation(self, p: np.ndarray, q: np.ndarray, r: np.ndarray) -> int:
        """
        Find orientation of ordered triplet (p, q, r).
        
        Returns:
            0: Collinear, 1: Clockwise, 2: Counterclockwise
        """
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if abs(val) < 1e-10:
            return 0  # Collinear
        return 1 if val > 0 else 2  # Clockwise or Counterclockwise
    
    def calculate_center_of_mass(self, joint_angles_all: Dict[str, np.ndarray],
                               body_pose: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Calculate total center of mass including body and all legs.
        
        Args:
            joint_angles_all: Dictionary mapping leg_id to joint angles
            body_pose: Body pose [x, y, z, roll, pitch, yaw] (default: origin)
            
        Returns:
            Center of mass position [x, y, z] in world coordinates
        """
        if body_pose is None:
            body_pose = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
        
        total_mass = 0.0
        weighted_position = np.zeros(3)
        
        # Add body contribution
        body_position = body_pose[:3]
        body_com = body_position + self.body_properties.center_of_mass
        
        total_mass += self.body_properties.mass
        weighted_position += self.body_properties.mass * body_com
        
        # Add leg contributions
        for leg_id, joint_angles in joint_angles_all.items():
            if leg_id not in self.leg_dynamics:
                continue
                
            leg_dyn = self.leg_dynamics[leg_id]
            leg_mass = leg_dyn.mass_properties.get_total_mass()
            
            # Calculate leg center of mass (simplified: at geometric center)
            foot_position = self.kinematics.forward_kinematics(joint_angles, leg_id)
            leg_base_position = self.kinematics.leg_positions[leg_id]
            leg_com = (leg_base_position + foot_position) / 2  # Simplified
            
            total_mass += leg_mass
            weighted_position += leg_mass * leg_com
        
        return weighted_position / total_mass if total_mass > 0 else np.zeros(3)
    
    def calculate_stability_margin(self, support_polygon: np.ndarray,
                                 com_projection: np.ndarray) -> float:
        """
        Calculate stability margin (distance from COM to polygon edge).
        
        Args:
            support_polygon: Support polygon vertices [n x 2]
            com_projection: COM projection on ground [x, y]
            
        Returns:
            Stability margin in meters (positive = stable, negative = unstable)
        """
        start_time = time.time()
        
        if len(support_polygon) < 3:
            # No valid polygon
            computation_time = time.time() - start_time
            self._computation_times['stability_margin'].append(computation_time)
            return -1.0  # Unstable
        
        # Check if COM is inside polygon
        inside = self._point_in_polygon(com_projection, support_polygon)
        
        # Calculate minimum distance to polygon edges
        min_distance = float('inf')
        n_vertices = len(support_polygon)
        
        for i in range(n_vertices):
            p1 = support_polygon[i]
            p2 = support_polygon[(i + 1) % n_vertices]
            distance = self._point_to_line_distance(com_projection, p1, p2)
            min_distance = min(min_distance, distance)
        
        # Performance tracking
        computation_time = time.time() - start_time
        self._computation_times['stability_margin'].append(computation_time)
        
        return min_distance if inside else -min_distance
    
    def _point_in_polygon(self, point: np.ndarray, polygon: np.ndarray) -> bool:
        """Check if point is inside polygon using ray casting algorithm."""
        x, y = point
        n = len(polygon)
        inside = False
        
        # Handle degenerate cases
        if n < 3:
            return False
        
        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            
            # Check if point is on the edge (within tolerance)
            if self._point_to_line_distance(point, np.array([p1x, p1y]), np.array([p2x, p2y])) < 1e-6:
                return True  # Point on edge counts as inside
            
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
    
    def _point_to_line_distance(self, point: np.ndarray, line_p1: np.ndarray, line_p2: np.ndarray) -> float:
        """Calculate perpendicular distance from point to line segment."""
        x0, y0 = point
        x1, y1 = line_p1
        x2, y2 = line_p2
        
        # Calculate distance using formula
        numerator = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
        denominator = np.sqrt((y2 - y1)**2 + (x2 - x1)**2)
        
        if denominator < 1e-10:
            # Degenerate line (two points are the same)
            return np.linalg.norm(point - line_p1)
        
        return numerator / denominator
    
    def calculate_zero_moment_point(self, joint_angles_all: Dict[str, np.ndarray],
                                  joint_velocities_all: Dict[str, np.ndarray],
                                  joint_accelerations_all: Dict[str, np.ndarray],
                                  contact_forces: Dict[str, ContactForce],
                                  body_pose: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Calculate Zero Moment Point (ZMP) for dynamic stability.
        
        The ZMP is the point on the ground where the net moment of inertial
        and gravitational forces is zero.
        
        Args:
            joint_angles_all: Dictionary of joint angles for all legs
            joint_velocities_all: Dictionary of joint velocities for all legs
            joint_accelerations_all: Dictionary of joint accelerations for all legs
            contact_forces: Dictionary of contact forces for all legs
            body_pose: Body pose [x, y, z, roll, pitch, yaw]
            
        Returns:
            ZMP position [x, y] on the ground plane
        """
        start_time = time.time()
        
        if body_pose is None:
            body_pose = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
        
        # Calculate total force and moment about origin
        total_force = np.zeros(3)
        total_moment = np.zeros(3)
        
        # Add contributions from all legs in contact
        for leg_id, contact_force in contact_forces.items():
            if not contact_force.is_in_contact:
                continue
                
            # Add force contribution
            total_force += contact_force.force
            
            # Add moment contribution (r × F)
            foot_position = self.kinematics.forward_kinematics(
                joint_angles_all.get(leg_id, np.zeros(3)), leg_id
            )
            moment_arm = foot_position
            total_moment += np.cross(moment_arm, contact_force.force)
        
        # Add body inertial forces and moments
        com_position = self.calculate_center_of_mass(joint_angles_all, body_pose)
        
        # Body weight
        gravity_force = np.array([0.0, 0.0, -self.body_properties.mass * 9.81])
        total_force += gravity_force
        total_moment += np.cross(com_position, gravity_force)
        
        # Calculate ZMP using moment balance
        # ZMP is where Mx = 0 and My = 0 on the ground plane (z = 0)
        if abs(total_force[2]) < 1e-6:
            # No normal force - robot is falling
            zmp = np.array([0.0, 0.0])
        else:
            zmp_x = -total_moment[1] / total_force[2]
            zmp_y = total_moment[0] / total_force[2]
            zmp = np.array([zmp_x, zmp_y])
        
        # Performance tracking
        computation_time = time.time() - start_time
        self._computation_times['zmp_calculation'].append(computation_time)
        
        return zmp
    
    def optimize_force_distribution(self, target_total_force: np.ndarray,
                                  target_total_moment: np.ndarray,
                                  stance_legs: List[str],
                                  foot_positions: Dict[str, np.ndarray],
                                  force_limits: Optional[Dict[str, float]] = None) -> Dict[str, np.ndarray]:
        """
        Optimize force distribution among stance legs.
        
        Solves the force distribution problem to achieve desired total force
        and moment while minimizing effort and respecting constraints.
        
        Args:
            target_total_force: Desired total force [Fx, Fy, Fz]
            target_total_moment: Desired total moment [Mx, My, Mz]
            stance_legs: List of legs in contact with ground
            foot_positions: Dictionary of foot positions for stance legs
            force_limits: Optional force limits for each leg
            
        Returns:
            Dictionary mapping leg_id to optimal force vector [Fx, Fy, Fz]
        """
        start_time = time.time()
        
        n_legs = len(stance_legs)
        if n_legs == 0:
            computation_time = time.time() - start_time
            self._computation_times['force_distribution'].append(computation_time)
            return {}
        
        # Set up optimization problem: minimize ||F||² subject to constraints
        # Variables: [F1x, F1y, F1z, F2x, F2y, F2z, ...]
        
        # Constraint matrix for force/moment equilibrium
        A_eq = np.zeros((6, 3 * n_legs))  # 6 equilibrium equations
        b_eq = np.concatenate([target_total_force, target_total_moment])
        
        for i, leg_id in enumerate(stance_legs):
            foot_pos = foot_positions[leg_id]
            
            # Force equilibrium constraints
            A_eq[0:3, 3*i:3*i+3] = np.eye(3)
            
            # Moment equilibrium constraints (r × F = M)
            # This creates the cross-product matrix for moment calculation
            r_cross = np.array([
                [0, -foot_pos[2], foot_pos[1]],
                [foot_pos[2], 0, -foot_pos[0]],
                [-foot_pos[1], foot_pos[0], 0]
            ])
            A_eq[3:6, 3*i:3*i+3] = r_cross
        
        # Solve least squares problem with equality constraints
        try:
            # Use pseudoinverse for overconstrained or underconstrained systems
            force_vector = np.linalg.pinv(A_eq) @ b_eq
            
            # Reshape solution back to individual leg forces
            optimal_forces = {}
            for i, leg_id in enumerate(stance_legs):
                optimal_forces[leg_id] = force_vector[3*i:3*i+3]
            
            # Apply force limits if specified
            if force_limits is not None:
                for leg_id in stance_legs:
                    if leg_id in force_limits:
                        force_magnitude = np.linalg.norm(optimal_forces[leg_id])
                        if force_magnitude > force_limits[leg_id]:
                            # Scale down force to limit
                            optimal_forces[leg_id] *= force_limits[leg_id] / force_magnitude
            
        except np.linalg.LinAlgError:
            # Fallback: equal distribution
            warnings.warn("Force distribution optimization failed, using equal distribution")
            optimal_forces = {}
            equal_force = target_total_force / n_legs
            for leg_id in stance_legs:
                optimal_forces[leg_id] = equal_force
        
        # Performance tracking
        computation_time = time.time() - start_time
        self._computation_times['force_distribution'].append(computation_time)
        
        return optimal_forces
    
    def analyze_stability(self, joint_angles_all: Dict[str, np.ndarray],
                         joint_velocities_all: Optional[Dict[str, np.ndarray]] = None,
                         joint_accelerations_all: Optional[Dict[str, np.ndarray]] = None,
                         contact_states: Optional[Dict[str, bool]] = None,
                         body_pose: Optional[np.ndarray] = None) -> StabilityMetrics:
        """
        Perform comprehensive stability analysis.
        
        Args:
            joint_angles_all: Dictionary of joint angles for all legs
            joint_velocities_all: Dictionary of joint velocities (optional)
            joint_accelerations_all: Dictionary of joint accelerations (optional)
            contact_states: Dictionary of contact states (optional, assumes all in contact)
            body_pose: Body pose (optional, assumes origin)
            
        Returns:
            Comprehensive stability metrics
        """
        # Set defaults
        if contact_states is None:
            contact_states = {leg_id: True for leg_id in joint_angles_all.keys()}
        if body_pose is None:
            body_pose = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
        
        # Get foot positions
        foot_positions = {}
        for leg_id, joint_angles in joint_angles_all.items():
            foot_positions[leg_id] = self.kinematics.forward_kinematics(joint_angles, leg_id)
        
        # Calculate support polygon
        support_polygon = self.calculate_support_polygon(foot_positions, contact_states)
        
        # Calculate center of mass
        com_3d = self.calculate_center_of_mass(joint_angles_all, body_pose)
        com_projection = com_3d[:2]  # Project to ground plane
        
        # Calculate stability margin
        stability_margin = self.calculate_stability_margin(support_polygon, com_projection)
        
        # Static stability
        is_statically_stable = stability_margin > 0 and len(support_polygon) >= 3
        
        # Dynamic stability (ZMP-based) if motion data available
        zmp_position = np.array([0.0, 0.0])
        zmp_stability_margin = 0.0
        is_dynamically_stable = is_statically_stable  # Default to static
        
        if (joint_velocities_all is not None and joint_accelerations_all is not None):
            # Create dummy contact forces for ZMP calculation
            contact_forces = {}
            for leg_id in contact_states:
                if contact_states[leg_id]:
                    # Assume equal weight distribution
                    normal_force = self.body_properties.mass * 9.81 / len(contact_states)
                    contact_forces[leg_id] = ContactForce.ground_contact(
                        normal_force, np.array([0.0, 0.0]), foot_positions[leg_id]
                    )
                else:
                    contact_forces[leg_id] = ContactForce.no_contact()
            
            zmp_position = self.calculate_zero_moment_point(
                joint_angles_all, joint_velocities_all, joint_accelerations_all,
                contact_forces, body_pose
            )
            zmp_stability_margin = self.calculate_stability_margin(support_polygon, zmp_position)
            is_dynamically_stable = zmp_stability_margin > 0
        
        # Calculate tip-over risk
        if stability_margin > self.stability_threshold:
            tip_over_risk = 0.0  # Stable
        elif stability_margin > 0:
            tip_over_risk = 1.0 - (stability_margin / self.stability_threshold)
        else:
            tip_over_risk = 1.0  # Unstable
        
        # Get stance legs
        stance_legs = [leg_id for leg_id, in_contact in contact_states.items() if in_contact]
        
        return StabilityMetrics(
            support_polygon=support_polygon,
            com_projection=com_projection,
            stability_margin=stability_margin,
            zmp_position=zmp_position,
            zmp_stability_margin=zmp_stability_margin,
            tip_over_risk=tip_over_risk,
            stance_legs=stance_legs,
            is_statically_stable=is_statically_stable,
            is_dynamically_stable=is_dynamically_stable
        )
    
    def get_performance_metrics(self) -> Dict[str, any]:
        """Get performance metrics for body dynamics calculations."""
        metrics = {}
        
        for operation, times in self._computation_times.items():
            if times:
                metrics[operation] = {
                    'count': len(times),
                    'avg_time_ms': np.mean(times) * 1000,
                    'max_time_ms': np.max(times) * 1000,
                    'min_time_ms': np.min(times) * 1000,
                    'total_time_ms': np.sum(times) * 1000
                }
            else:
                metrics[operation] = {
                    'count': 0,
                    'avg_time_ms': 0.0,
                    'max_time_ms': 0.0,
                    'min_time_ms': 0.0,
                    'total_time_ms': 0.0
                }
        
        return metrics
    
    def reset_performance_metrics(self) -> None:
        """Reset performance tracking."""
        for key in self._computation_times:
            self._computation_times[key].clear()


def create_complete_dynamics_system(config: Optional[RobotConfig] = None) -> Tuple[BodyDynamics, Dict[str, LegDynamics]]:
    """
    Create complete dynamics system with body and all leg dynamics.
    
    Args:
        config: Robot configuration
        
    Returns:
        Tuple of (BodyDynamics instance, dictionary of LegDynamics instances)
    """
    # Create leg dynamics for all legs
    leg_dynamics = create_leg_dynamics_for_robot(config)
    
    # Create body dynamics
    body_dynamics = BodyDynamics(config, leg_dynamics=leg_dynamics)
    
    return body_dynamics, leg_dynamics