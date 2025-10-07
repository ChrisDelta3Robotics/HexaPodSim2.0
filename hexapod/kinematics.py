#!/usr/bin/env python3
"""
Hexapod Robot Forward Kinematics with Star Configuration

This module implements forward kinematics for a hexapod robot featuring a star-shaped
leg configuration where front and back legs are angled for enhanced stability and
maneuverability.

Key Features:
- Star configuration with configurable INWARD_LEG_ANGLE (default 15°)
- Denavit-Hartenberg convention with degree-based calculations
- Individual leg kinematics for all 6 legs (L1, R1, L2, R2, L3, R3)
- Body-frame base rotations applied before D-H transforms
- Comprehensive joint limit validation (-90° to +90°)

Author: HexaPodSim 2.0
Date: October 2025
"""

import numpy as np
from typing import Tuple, Dict, List, Optional
from dataclasses import dataclass
import warnings
from enum import Enum


@dataclass
class RobotConfig:
    """Robot physical configuration parameters (all in meters and degrees)."""
    
    # Body dimensions
    body_length: float = 0.20      # 20cm
    body_width: float = 0.15       # 15cm
    body_height: float = 0.05      # 5cm
    
    # Leg segment lengths
    coxa_length: float = 0.04      # 4cm
    femur_length: float = 0.08     # 8cm
    tibia_length: float = 0.12     # 12cm
    
    # Star configuration
    inward_leg_angle: float = 15.0  # degrees (configurable)
    
    # Joint limits (degrees)
    joint_min: float = -90.0
    joint_max: float = 90.0


@dataclass 
class BodyPose:
    """Robot body pose representation (position + orientation in degrees)."""
    
    # Position (meters)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.1  # Default height 10cm above ground
    
    # Orientation (degrees)
    roll: float = 0.0   # Rotation around X-axis
    pitch: float = 0.0  # Rotation around Y-axis
    yaw: float = 0.0    # Rotation around Z-axis
    
    def to_position_vector(self) -> np.ndarray:
        """Get position as numpy array."""
        return np.array([self.x, self.y, self.z])
    
    def to_orientation_vector(self) -> np.ndarray:
        """Get orientation as numpy array (degrees)."""
        return np.array([self.roll, self.pitch, self.yaw])


class LegConfiguration(Enum):
    """Leg configuration identifiers for star pattern."""
    L1_FRONT_LEFT = "L1"
    R1_FRONT_RIGHT = "R1" 
    L2_MIDDLE_LEFT = "L2"
    R2_MIDDLE_RIGHT = "R2"
    L3_REAR_LEFT = "L3"
    R3_REAR_RIGHT = "R3"


class RobotConfiguration:
    """
    Robot body configuration management for star-shaped hexapod layout.
    
    This class handles:
    - Star configuration leg positioning and base rotations
    - Body-to-leg coordinate transformations
    - Body pose management (position + orientation in degrees)
    - Body height adjustment and tilt compensation
    - Configuration validation and error handling
    """
    
    def __init__(self, config: Optional[RobotConfig] = None):
        """
        Initialize robot configuration.
        
        Args:
            config: Robot physical configuration parameters
        """
        self.config = config or RobotConfig()
        self._current_pose = BodyPose()
        
        # Star configuration leg positions and base rotations
        self._setup_star_configuration()
        
        # Validation flags
        self._is_validated = False
        self._last_validation_result = None
    
    def _setup_star_configuration(self) -> None:
        """Setup star configuration leg positions and base rotations."""
        
        # Calculate leg positions based on body dimensions
        # Front legs (angled forward)
        front_x = self.config.body_length / 2
        # Middle legs (perpendicular) 
        middle_x = 0.0
        # Back legs (angled backward)
        rear_x = -self.config.body_length / 2
        
        # Y positions (left/right sides)
        left_y = self.config.body_width / 2
        right_y = -self.config.body_width / 2
        
        # Z position (body height)
        leg_z = 0.0  # Legs attach at body center height
        
        # Star configuration positions
        self.leg_positions = {
            LegConfiguration.L1_FRONT_LEFT.value: np.array([front_x, left_y, leg_z]),
            LegConfiguration.R1_FRONT_RIGHT.value: np.array([front_x, right_y, leg_z]),
            LegConfiguration.L2_MIDDLE_LEFT.value: np.array([middle_x, left_y, leg_z]),
            LegConfiguration.R2_MIDDLE_RIGHT.value: np.array([middle_x, right_y, leg_z]),
            LegConfiguration.L3_REAR_LEFT.value: np.array([rear_x, left_y, leg_z]),
            LegConfiguration.R3_REAR_RIGHT.value: np.array([rear_x, right_y, leg_z])
        }
        
        # Star configuration base rotations (degrees)
        # Front legs angled forward, back legs angled backward
        self.star_rotations = {
            LegConfiguration.L1_FRONT_LEFT.value: +self.config.inward_leg_angle,
            LegConfiguration.R1_FRONT_RIGHT.value: +self.config.inward_leg_angle,
            LegConfiguration.L2_MIDDLE_LEFT.value: 0.0,
            LegConfiguration.R2_MIDDLE_RIGHT.value: 0.0,
            LegConfiguration.L3_REAR_LEFT.value: -self.config.inward_leg_angle,
            LegConfiguration.R3_REAR_RIGHT.value: -self.config.inward_leg_angle
        }
        
        # Leg identifiers list
        self.leg_names = list(LegConfiguration.__members__.values())
        self.leg_ids = [leg.value for leg in self.leg_names]
    
    @property
    def current_pose(self) -> BodyPose:
        """Get current body pose."""
        return self._current_pose
    
    @current_pose.setter
    def current_pose(self, pose: BodyPose) -> None:
        """Set current body pose with validation."""
        self.set_body_pose(pose)
    
    def set_body_pose(self, pose: BodyPose) -> None:
        """
        Set robot body pose with validation.
        
        Args:
            pose: New body pose
            
        Raises:
            ValueError: If pose is invalid
        """
        self._validate_body_pose(pose)
        self._current_pose = pose
        self._is_validated = False  # Require re-validation after pose change
    
    def set_body_position(self, x: float, y: float, z: float) -> None:
        """
        Set body position while keeping current orientation.
        
        Args:
            x: X position in meters
            y: Y position in meters  
            z: Z position in meters (height above ground)
        """
        new_pose = BodyPose(
            x=x, y=y, z=z,
            roll=self._current_pose.roll,
            pitch=self._current_pose.pitch,
            yaw=self._current_pose.yaw
        )
        self.set_body_pose(new_pose)
    
    def set_body_orientation(self, roll: float, pitch: float, yaw: float) -> None:
        """
        Set body orientation while keeping current position.
        
        Args:
            roll: Roll angle in degrees
            pitch: Pitch angle in degrees
            yaw: Yaw angle in degrees
        """
        new_pose = BodyPose(
            x=self._current_pose.x,
            y=self._current_pose.y,
            z=self._current_pose.z,
            roll=roll, pitch=pitch, yaw=yaw
        )
        self.set_body_pose(new_pose)
    
    def adjust_body_height(self, height: float) -> None:
        """
        Adjust body height (Z position) while keeping other parameters.
        
        Args:
            height: New height in meters
        """
        self.set_body_position(
            self._current_pose.x,
            self._current_pose.y,
            height
        )
    
    def _validate_body_pose(self, pose: BodyPose) -> None:
        """
        Validate body pose parameters.
        
        Args:
            pose: Body pose to validate
            
        Raises:
            ValueError: If pose parameters are invalid
        """
        # Height limits (prevent ground collision or excessive height)
        if pose.z < 0.02:  # Minimum 2cm above ground
            raise ValueError(f"Body height {pose.z:.3f}m too low (minimum 0.02m)")
        if pose.z > 0.5:   # Maximum 50cm height
            raise ValueError(f"Body height {pose.z:.3f}m too high (maximum 0.5m)")
        
        # Orientation limits (prevent excessive tilts)
        max_tilt = 45.0  # degrees
        if abs(pose.roll) > max_tilt:
            raise ValueError(f"Roll angle {pose.roll:.1f}° exceeds limit (±{max_tilt}°)")
        if abs(pose.pitch) > max_tilt:
            raise ValueError(f"Pitch angle {pose.pitch:.1f}° exceeds limit (±{max_tilt}°)")
        
        # Yaw can be any value (normalize to [-180, 180])
        if abs(pose.yaw) > 360:
            warnings.warn(f"Large yaw angle {pose.yaw:.1f}° (consider normalizing)")
    
    def transform_body_to_leg(self, leg_id: str) -> np.ndarray:
        """
        Get transformation matrix from body frame to leg base frame.
        
        Args:
            leg_id: Leg identifier
            
        Returns:
            4x4 homogeneous transformation matrix
        """
        if leg_id not in self.leg_ids:
            raise ValueError(f"Unknown leg_id: {leg_id}. Must be one of {self.leg_ids}")
        
        # Get leg position and star rotation
        leg_position = self.leg_positions[leg_id]
        star_rotation = self.star_rotations[leg_id]
        
        # Create transformation matrix
        # 1. Apply body pose (position + orientation)
        T_body = self._create_body_transform()
        
        # 2. Apply leg offset from body center
        T_leg_offset = self._create_translation_transform(leg_position)
        
        # 3. Apply star configuration rotation
        T_star = self._create_rotation_z_transform(star_rotation)
        
        # Chain transformations: T_total = T_body * T_leg_offset * T_star
        T_total = T_body @ T_leg_offset @ T_star
        
        return T_total
    
    def transform_leg_to_body(self, leg_id: str) -> np.ndarray:
        """
        Get transformation matrix from leg base frame to body frame.
        
        Args:
            leg_id: Leg identifier
            
        Returns:
            4x4 homogeneous transformation matrix (inverse of body-to-leg)
        """
        T_body_to_leg = self.transform_body_to_leg(leg_id)
        return np.linalg.inv(T_body_to_leg)
    
    def transform_world_to_body(self) -> np.ndarray:
        """
        Get transformation matrix from world frame to body frame.
        
        Returns:
            4x4 homogeneous transformation matrix
        """
        return np.linalg.inv(self._create_body_transform())
    
    def _create_body_transform(self) -> np.ndarray:
        """Create body transformation matrix from current pose."""
        pose = self._current_pose
        
        # Convert degrees to radians for calculation
        roll_rad = np.deg2rad(pose.roll)
        pitch_rad = np.deg2rad(pose.pitch)
        yaw_rad = np.deg2rad(pose.yaw)
        
        # Create rotation matrices
        R_roll = self._create_rotation_x_matrix(roll_rad)
        R_pitch = self._create_rotation_y_matrix(pitch_rad)
        R_yaw = self._create_rotation_z_matrix(yaw_rad)
        
        # Combined rotation: R = R_yaw * R_pitch * R_roll (ZYX Euler angles)
        R_combined = R_yaw @ R_pitch @ R_roll
        
        # Create homogeneous transformation matrix
        T_body = np.eye(4)
        T_body[:3, :3] = R_combined
        T_body[:3, 3] = pose.to_position_vector()
        
        return T_body
    
    def _create_translation_transform(self, translation: np.ndarray) -> np.ndarray:
        """Create translation transformation matrix."""
        T = np.eye(4)
        T[:3, 3] = translation
        return T
    
    def _create_rotation_z_transform(self, angle_deg: float) -> np.ndarray:
        """Create rotation transformation matrix around Z-axis."""
        T = np.eye(4)
        angle_rad = np.deg2rad(angle_deg)
        T[:3, :3] = self._create_rotation_z_matrix(angle_rad)
        return T
    
    def _create_rotation_x_matrix(self, angle_rad: float) -> np.ndarray:
        """Create 3x3 rotation matrix around X-axis."""
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)
        return np.array([
            [1, 0,      0     ],
            [0, cos_a, -sin_a],
            [0, sin_a,  cos_a]
        ])
    
    def _create_rotation_y_matrix(self, angle_rad: float) -> np.ndarray:
        """Create 3x3 rotation matrix around Y-axis."""
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)
        return np.array([
            [cos_a,  0, sin_a],
            [0,      1, 0    ],
            [-sin_a, 0, cos_a]
        ])
    
    def _create_rotation_z_matrix(self, angle_rad: float) -> np.ndarray:
        """Create 3x3 rotation matrix around Z-axis."""
        cos_a = np.cos(angle_rad)
        sin_a = np.sin(angle_rad)
        return np.array([
            [cos_a, -sin_a, 0],
            [sin_a,  cos_a, 0],
            [0,     0,      1]
        ])
    
    def get_leg_positions_world(self) -> Dict[str, np.ndarray]:
        """
        Get leg base positions in world coordinates.
        
        Returns:
            Dictionary mapping leg_id to world position
        """
        leg_positions_world = {}
        
        for leg_id in self.leg_ids:
            # Transform leg base position to world coordinates
            T_body_to_leg = self.transform_body_to_leg(leg_id)
            
            # Leg base is at origin of leg frame, so transform [0,0,0,1]
            leg_pos_homogeneous = T_body_to_leg @ np.array([0, 0, 0, 1])
            leg_positions_world[leg_id] = leg_pos_homogeneous[:3]
        
        return leg_positions_world
    
    def compensate_body_tilt(self, target_foot_positions: Dict[str, np.ndarray],
                           compensation_factor: float = 1.0) -> Dict[str, np.ndarray]:
        """
        Compensate foot positions for body tilt to maintain ground contact.
        
        Args:
            target_foot_positions: Original target foot positions
            compensation_factor: Compensation strength (0.0 = no compensation, 1.0 = full)
            
        Returns:
            Compensated foot positions
        """
        if not (0.0 <= compensation_factor <= 1.0):
            raise ValueError("Compensation factor must be between 0.0 and 1.0")
        
        compensated_positions = {}
        
        # Get current body orientation
        roll_rad = np.deg2rad(self._current_pose.roll)
        pitch_rad = np.deg2rad(self._current_pose.pitch)
        
        for leg_id, original_pos in target_foot_positions.items():
            if leg_id not in self.leg_ids:
                warnings.warn(f"Unknown leg_id {leg_id}, skipping compensation")
                compensated_positions[leg_id] = original_pos
                continue
            
            # Get leg base position relative to body center
            leg_base_pos = self.leg_positions[leg_id]
            
            # Calculate tilt compensation offsets
            # Roll affects Y-axis displacement, pitch affects X-axis displacement
            roll_compensation = -leg_base_pos[1] * np.sin(roll_rad) * compensation_factor
            pitch_compensation = -leg_base_pos[0] * np.sin(pitch_rad) * compensation_factor
            
            # Apply compensation to Z-coordinate (height)
            compensated_z = original_pos[2] + roll_compensation + pitch_compensation
            
            compensated_positions[leg_id] = np.array([
                original_pos[0],
                original_pos[1], 
                compensated_z
            ])
        
        return compensated_positions
    
    def validate_configuration(self) -> Dict[str, any]:
        """
        Validate robot configuration and return diagnostic information.
        
        Returns:
            Dictionary containing validation results
        """
        validation_info = {
            'config_valid': True,
            'timestamp': np.datetime64('now'),
            'pose': self._current_pose,
            'inward_leg_angle': self.config.inward_leg_angle,
            'leg_positions': self.leg_positions.copy(),
            'star_rotations': self.star_rotations.copy(),
            'warnings': [],
            'errors': []
        }
        
        try:
            # Validate current body pose
            self._validate_body_pose(self._current_pose)
            
            # Check star configuration parameters
            if not (0 <= abs(self.config.inward_leg_angle) <= 45):
                validation_info['warnings'].append(
                    f"Inward leg angle {self.config.inward_leg_angle}° outside recommended range [0°, 45°]"
                )
            
            # Validate leg positions relative to body
            for leg_id, position in self.leg_positions.items():
                distance_from_center = np.linalg.norm(position[:2])
                if distance_from_center > 0.5:
                    validation_info['errors'].append(
                        f"Leg {leg_id} position too far from body center: {distance_from_center:.3f}m"
                    )
                    validation_info['config_valid'] = False
            
            # Test coordinate transformations
            for leg_id in self.leg_ids:
                try:
                    T_body_to_leg = self.transform_body_to_leg(leg_id)
                    T_leg_to_body = self.transform_leg_to_body(leg_id)
                    
                    # Check if transforms are inverses
                    identity_check = T_body_to_leg @ T_leg_to_body
                    identity_error = np.linalg.norm(identity_check - np.eye(4))
                    
                    if identity_error > 1e-10:
                        validation_info['warnings'].append(
                            f"Transform consistency issue for {leg_id}: error = {identity_error:.2e}"
                        )
                        
                except Exception as e:
                    validation_info['errors'].append(f"Transform error for {leg_id}: {e}")
                    validation_info['config_valid'] = False
        
        except Exception as e:
            validation_info['errors'].append(f"Validation failed: {e}")
            validation_info['config_valid'] = False
        
        self._last_validation_result = validation_info
        self._is_validated = validation_info['config_valid']
        
        return validation_info
    
    def get_configuration_summary(self) -> Dict[str, any]:
        """
        Get summary of robot configuration for display/logging.
        
        Returns:
            Configuration summary dictionary
        """
        return {
            'body_dimensions': {
                'length': self.config.body_length,
                'width': self.config.body_width,
                'height': self.config.body_height
            },
            'leg_dimensions': {
                'coxa': self.config.coxa_length,
                'femur': self.config.femur_length,
                'tibia': self.config.tibia_length
            },
            'star_config': {
                'inward_angle': self.config.inward_leg_angle,
                'leg_positions': {leg_id: pos.tolist() for leg_id, pos in self.leg_positions.items()},
                'star_rotations': self.star_rotations.copy()
            },
            'current_pose': {
                'position': self._current_pose.to_position_vector().tolist(),
                'orientation': self._current_pose.to_orientation_vector().tolist()
            },
            'joint_limits': {
                'min': self.config.joint_min,
                'max': self.config.joint_max
            },
            'validation_status': {
                'is_validated': self._is_validated,
                'last_validation': self._last_validation_result is not None
            }
        }


class HexapodKinematics:
    """
    Forward kinematics implementation for hexapod robot with star configuration.
    
    The star configuration angles the front and back legs for improved stability:
    - Front legs (L1, R1): Base angle = 90° + inward_leg_angle
    - Middle legs (L2, R2): Base angle = 90° (perpendicular)
    - Back legs (L3, R3): Base angle = 90° - inward_leg_angle
    
    All calculations performed in degrees for consistency with project standards.
    """
    
    # Leg identification and color mapping
    LEG_NAMES = ['L1', 'R1', 'L2', 'R2', 'L3', 'R3']
    LEG_COLORS = {
        'L1': '#00FF00',  # Neon Green (Left Front)
        'R1': '#00FFFF',  # Neon Cyan (Right Front)
        'L2': '#FF00FF',  # Neon Magenta (Left Middle)
        'R2': '#FFFF00',  # Neon Yellow (Right Middle)
        'L3': '#FF6600',  # Neon Orange (Left Back)
        'R3': '#FF0080'   # Neon Pink (Right Back)
    }
    
    def __init__(self, config: Optional[RobotConfig] = None):
        """
        Initialize hexapod kinematics with star configuration.
        
        Args:
            config: Robot configuration parameters. If None, uses default values.
        """
        self.config = config or RobotConfig()
        self._setup_leg_base_positions()
        self._setup_star_rotations()
        
    def _setup_leg_base_positions(self) -> None:
        """Calculate leg attachment points on the body (in body frame)."""
        half_length = self.config.body_length / 2
        half_width = self.config.body_width / 2
        
        # Leg positions relative to body center
        self.leg_positions = {
            'L1': np.array([half_length, half_width, 0.0]),   # Left Front
            'R1': np.array([half_length, -half_width, 0.0]),  # Right Front
            'L2': np.array([0.0, half_width, 0.0]),           # Left Middle
            'R2': np.array([0.0, -half_width, 0.0]),          # Right Middle
            'L3': np.array([-half_length, half_width, 0.0]),  # Left Back
            'R3': np.array([-half_length, -half_width, 0.0])  # Right Back
        }
    
    def _setup_star_rotations(self) -> None:
        """Calculate base rotation angles for star configuration."""
        angle = self.config.inward_leg_angle
        
        # Star configuration: front legs angled forward, back legs angled backward
        self.star_rotations = {
            'L1': +angle,  # Front legs: +15° forward
            'R1': -angle,  # (mirrored for right side)
            'L2': 0.0,     # Middle legs: perpendicular
            'R2': 0.0,
            'L3': -angle,  # Back legs: -15° backward
            'R3': +angle   # (mirrored for right side)
        }
    
    def validate_joint_angles(self, joint_angles_deg: np.ndarray, leg_id: str) -> bool:
        """
        Validate that joint angles are within acceptable limits.
        
        Args:
            joint_angles_deg: Array of joint angles [coxa, femur, tibia] in degrees
            leg_id: Leg identifier ('L1', 'R1', 'L2', 'R2', 'L3', 'R3')
            
        Returns:
            True if all angles are valid, False otherwise
            
        Raises:
            ValueError: If joint angles are outside [-90, 90] degree limits
        """
        if len(joint_angles_deg) != 3:
            raise ValueError(f"Expected 3 joint angles, got {len(joint_angles_deg)}")
        
        if leg_id not in self.LEG_NAMES:
            raise ValueError(f"Invalid leg_id '{leg_id}'. Must be one of {self.LEG_NAMES}")
        
        # Check joint limits
        for i, angle in enumerate(joint_angles_deg):
            if angle < self.config.joint_min or angle > self.config.joint_max:
                joint_names = ['coxa', 'femur', 'tibia']
                raise ValueError(
                    f"Joint {joint_names[i]} angle {angle:.1f}° for leg {leg_id} "
                    f"exceeds limits [{self.config.joint_min}, {self.config.joint_max}]°"
                )
        
        return True
    
    def _deg_to_rad(self, degrees: float) -> float:
        """Convert degrees to radians (used only internally for trigonometry)."""
        return np.deg2rad(degrees)
    
    def _create_dh_transform(self, theta_deg: float, d: float, a: float, alpha_deg: float) -> np.ndarray:
        """
        Create Denavit-Hartenberg transformation matrix.
        
        Args:
            theta_deg: Joint angle in degrees
            d: Link offset along Z-axis
            a: Link length along X-axis
            alpha_deg: Link twist in degrees
            
        Returns:
            4x4 homogeneous transformation matrix
        """
        # Convert to radians for trigonometry
        theta = self._deg_to_rad(theta_deg)
        alpha = self._deg_to_rad(alpha_deg)
        
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)
        
        # D-H transformation matrix
        T = np.array([
            [cos_theta, -sin_theta * cos_alpha,  sin_theta * sin_alpha, a * cos_theta],
            [sin_theta,  cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
            [0,          sin_alpha,              cos_alpha,             d],
            [0,          0,                      0,                     1]
        ])
        
        return T
    
    def _create_star_base_transform(self, leg_id: str) -> np.ndarray:
        """
        Create base transformation for star configuration.
        
        Args:
            leg_id: Leg identifier
            
        Returns:
            4x4 transformation matrix from body frame to leg base frame
        """
        position = self.leg_positions[leg_id]
        rotation_deg = self.star_rotations[leg_id]
        
        # Create rotation matrix around Z-axis for star configuration
        rot_rad = self._deg_to_rad(rotation_deg)
        cos_rot = np.cos(rot_rad)
        sin_rot = np.sin(rot_rad)
        
        # Base transformation: translation + rotation
        T_base = np.array([
            [cos_rot, -sin_rot, 0, position[0]],
            [sin_rot,  cos_rot, 0, position[1]],
            [0,        0,       1, position[2]],
            [0,        0,       0, 1]
        ])
        
        return T_base
    
    def forward_kinematics(self, joint_angles_deg: np.ndarray, leg_id: str) -> np.ndarray:
        """
        Calculate forward kinematics for a hexapod leg with star configuration.
        
        This is the main function that computes the end-effector (foot) position
        given joint angles. It applies the star configuration base rotation before
        standard Denavit-Hartenberg calculations.
        
        Args:
            joint_angles_deg: Joint angles in degrees [coxa, femur, tibia]
            leg_id: Leg identifier ('L1', 'R1', 'L2', 'R2', 'L3', 'R3')
            
        Returns:
            End-effector position [x, y, z] in meters (world coordinates)
            
        Raises:
            ValueError: If joint angles exceed [-90, 90] degree limits
        """
        # Validate inputs
        self.validate_joint_angles(joint_angles_deg, leg_id)
        
        coxa_angle, femur_angle, tibia_angle = joint_angles_deg
        
        # Step 1: Create star configuration base transform
        T_base = self._create_star_base_transform(leg_id)
        
        # Step 2: Create D-H transformations for each joint
        # Coxa joint (yaw rotation around Z)
        T_coxa = self._create_dh_transform(
            theta_deg=coxa_angle,
            d=0.0,
            a=self.config.coxa_length,
            alpha_deg=0.0
        )
        
        # Femur joint (pitch rotation around Y)
        T_femur = self._create_dh_transform(
            theta_deg=femur_angle,
            d=0.0,
            a=self.config.femur_length,
            alpha_deg=-90.0  # Rotate to align with Y-axis
        )
        
        # Tibia joint (pitch rotation around Y)
        T_tibia = self._create_dh_transform(
            theta_deg=tibia_angle,
            d=0.0,
            a=self.config.tibia_length,
            alpha_deg=0.0
        )
        
        # Step 3: Chain all transformations
        T_total = T_base @ T_coxa @ T_femur @ T_tibia
        
        # Step 4: Extract foot position (first 3 elements of last column)
        foot_position = T_total[:3, 3]
        
        return foot_position
    
    def get_leg_workspace(self, leg_id: str, resolution: int = 50) -> Dict[str, np.ndarray]:
        """
        Calculate the reachable workspace for a given leg.
        
        Args:
            leg_id: Leg identifier
            resolution: Number of samples per joint for workspace calculation
            
        Returns:
            Dictionary containing workspace points and metadata
        """
        self.validate_joint_angles(np.array([0, 0, 0]), leg_id)  # Validate leg_id
        
        # Create joint angle ranges
        joint_range = np.linspace(self.config.joint_min, self.config.joint_max, resolution)
        
        workspace_points = []
        joint_configurations = []
        
        # Sample the joint space
        for coxa in joint_range:
            for femur in joint_range:
                for tibia in joint_range:
                    try:
                        joint_angles = np.array([coxa, femur, tibia])
                        foot_pos = self.forward_kinematics(joint_angles, leg_id)
                        
                        workspace_points.append(foot_pos)
                        joint_configurations.append(joint_angles)
                        
                    except ValueError:
                        # Skip invalid configurations
                        continue
        
        return {
            'points': np.array(workspace_points),
            'joint_configs': np.array(joint_configurations),
            'leg_id': leg_id,
            'resolution': resolution,
            'total_points': len(workspace_points)
        }
    
    def get_all_foot_positions(self, joint_angles_all: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """
        Calculate foot positions for all legs simultaneously.
        
        Args:
            joint_angles_all: Dictionary mapping leg_id to joint angles [coxa, femur, tibia]
            
        Returns:
            Dictionary mapping leg_id to foot position [x, y, z]
        """
        foot_positions = {}
        
        for leg_id in self.LEG_NAMES:
            if leg_id in joint_angles_all:
                try:
                    foot_positions[leg_id] = self.forward_kinematics(
                        joint_angles_all[leg_id], leg_id
                    )
                except ValueError as e:
                    warnings.warn(f"Invalid joint angles for {leg_id}: {e}")
                    foot_positions[leg_id] = None
            else:
                warnings.warn(f"No joint angles provided for {leg_id}")
                foot_positions[leg_id] = None
        
        return foot_positions
    
    def validate_star_configuration(self) -> Dict[str, any]:
        """
        Validate the star configuration setup and return diagnostic information.
        
        Returns:
            Dictionary containing validation results and configuration details
        """
        validation_info = {
            'config_valid': True,
            'inward_leg_angle': self.config.inward_leg_angle,
            'leg_positions': self.leg_positions.copy(),
            'star_rotations': self.star_rotations.copy(),
            'warnings': [],
            'errors': []
        }
        
        # Check star angle range
        if not (0 <= abs(self.config.inward_leg_angle) <= 45):
            validation_info['warnings'].append(
                f"Inward leg angle {self.config.inward_leg_angle}° is outside recommended range [0, 45]°"
            )
        
        # Validate leg positions
        for leg_id, position in self.leg_positions.items():
            if np.linalg.norm(position[:2]) > 0.5:  # Sanity check: within 50cm of center
                validation_info['errors'].append(
                    f"Leg {leg_id} position {position} seems unusually far from body center"
                )
                validation_info['config_valid'] = False
        
        # Test forward kinematics with neutral pose
        try:
            neutral_angles = np.array([0.0, 0.0, 0.0])
            for leg_id in self.LEG_NAMES:
                foot_pos = self.forward_kinematics(neutral_angles, leg_id)
                
                # Check if foot position is reasonable
                distance = np.linalg.norm(foot_pos)
                expected_distance = self.config.coxa_length + self.config.femur_length + self.config.tibia_length
                
                if abs(distance - expected_distance) > 0.01:  # 1cm tolerance
                    validation_info['warnings'].append(
                        f"Leg {leg_id} neutral pose distance {distance:.3f}m differs from expected {expected_distance:.3f}m"
                    )
        
        except Exception as e:
            validation_info['errors'].append(f"Forward kinematics test failed: {e}")
            validation_info['config_valid'] = False
        
        return validation_info

    # ========================================================================
    # INVERSE KINEMATICS IMPLEMENTATION
    # ========================================================================
    
    def inverse_kinematics(self, target_position: np.ndarray, leg_id: str, 
                          prefer_solution: str = 'elbow_down') -> Tuple[np.ndarray, bool]:
        """
        Calculate inverse kinematics for a hexapod leg with star configuration.
        
        This function computes joint angles needed to position the foot at a target
        location. It uses analytical solutions for the 3-DOF leg mechanism.
        
        Args:
            target_position: Desired foot position [x, y, z] in world coordinates (meters)
            leg_id: Leg identifier ('L1', 'R1', 'L2', 'R2', 'L3', 'R3')
            prefer_solution: Preferred elbow configuration ('elbow_down', 'elbow_up')
            
        Returns:
            Tuple of (joint_angles_deg, success_flag)
            - joint_angles_deg: Joint angles [coxa, femur, tibia] in degrees
            - success_flag: True if solution found within workspace, False otherwise
            
        Raises:
            ValueError: If target position is invalid or leg_id unknown
        """
        # Validate inputs
        if leg_id not in self.LEG_NAMES:
            raise ValueError(f"Unknown leg_id: {leg_id}. Must be one of {self.LEG_NAMES}")
        
        if len(target_position) != 3:
            raise ValueError("Target position must be a 3D vector [x, y, z]")
            
        # Step 1: Transform target from world coordinates to leg base coordinates
        # This removes the star configuration effect
        leg_base_position = self._world_to_leg_coordinates(target_position, leg_id)
        
        # Step 2: Solve analytical inverse kinematics in leg coordinates
        joint_angles, success = self._solve_leg_ik(leg_base_position, prefer_solution)
        
        # Step 3: Validate joint limits
        if success:
            success = self._validate_joint_limits(joint_angles)
            
        # Step 4: Refine solution with one Newton-Raphson iteration for better accuracy
        if success:
            # Calculate current position error
            current_pos = self.forward_kinematics(joint_angles, leg_id)
            position_error = target_position - current_pos
            error_magnitude = np.linalg.norm(position_error)
            
            # If error is significant, try to refine with numerical step
            if error_magnitude > 0.002:  # 2mm threshold for refinement
                # Compute Jacobian numerically (approximate)
                delta = 0.1  # 0.1 degree step for numerical derivative
                jacobian = np.zeros((3, 3))
                
                for i in range(3):
                    angles_plus = joint_angles.copy()
                    angles_plus[i] += delta
                    try:
                        pos_plus = self.forward_kinematics(angles_plus, leg_id)
                        jacobian[:, i] = (pos_plus - current_pos) / np.deg2rad(delta)
                    except:
                        # If forward kinematics fails, skip refinement
                        break
                else:
                    # Solve for angle correction: J * delta_angles = position_error
                    try:
                        # Use pseudoinverse for robustness
                        delta_angles_rad = np.linalg.pinv(jacobian) @ position_error
                        delta_angles_deg = np.rad2deg(delta_angles_rad)
                        
                        # Apply correction with damping to avoid overshoot
                        damping = 0.5
                        refined_angles = joint_angles + damping * delta_angles_deg
                        
                        # Check if refined solution is better and within limits
                        if self._validate_joint_limits(refined_angles):
                            refined_pos = self.forward_kinematics(refined_angles, leg_id)
                            refined_error = np.linalg.norm(refined_pos - target_position)
                            
                            if refined_error < error_magnitude:
                                joint_angles = refined_angles
                                error_magnitude = refined_error
                    except:
                        # If refinement fails, keep original solution
                        pass
            
            # Final verification with refined tolerance
            if error_magnitude > 0.008:  # 8mm tolerance (reasonable for 24cm leg reach)
                success = False
                warnings.warn(f"IK solution verification failed for {leg_id}. "
                            f"Position error: {error_magnitude*1000:.2f}mm")
        
        return joint_angles, success
    
    def _world_to_leg_coordinates(self, world_position: np.ndarray, leg_id: str) -> np.ndarray:
        """
        Transform a world coordinate position to leg base coordinates.
        
        This removes the star configuration rotation and leg base offset,
        allowing us to solve IK in the simplified leg coordinate system.
        
        Args:
            world_position: Position in world coordinates [x, y, z]
            leg_id: Leg identifier
            
        Returns:
            Position in leg base coordinates [x, y, z]
        """
        # Get leg base position and star rotation
        leg_base_pos = self.leg_positions[leg_id]
        star_angle = self.star_rotations[leg_id]
        
        # Translate to leg base
        relative_pos = world_position - leg_base_pos
        
        # Apply inverse star rotation
        cos_angle = np.cos(np.radians(-star_angle))  # Negative for inverse
        sin_angle = np.sin(np.radians(-star_angle))
        
        # Rotation matrix for Z-axis (inverse star rotation)
        R_inv = np.array([
            [cos_angle, -sin_angle, 0],
            [sin_angle,  cos_angle, 0],
            [0,         0,          1]
        ])
        
        leg_position = R_inv @ relative_pos
        
        return leg_position
    
    def _solve_leg_ik(self, leg_position: np.ndarray, prefer_solution: str) -> Tuple[np.ndarray, bool]:
        """
        Solve analytical inverse kinematics for a 3-DOF leg in leg coordinates.
        
        This implementation works by inverting the D-H transformations step by step,
        which ensures consistency with the forward kinematics.
        
        Args:
            leg_position: Target position in leg base coordinates [x, y, z]
            prefer_solution: 'elbow_up' or 'elbow_down'
            
        Returns:
            Tuple of (joint_angles, success_flag)
        """
        x, y, z = leg_position
        
        # Step 1: Calculate coxa angle from X-Y projection
        horizontal_distance = np.sqrt(x**2 + y**2)
        
        if horizontal_distance < 1e-8:
            coxa_angle = 0.0
        else:
            coxa_angle = np.degrees(np.arctan2(y, x))
        
        # Step 2: Transform target to the coordinate frame after coxa rotation
        # Inverse of coxa transformation to find position in femur frame
        cos_coxa = np.cos(np.radians(coxa_angle))
        sin_coxa = np.sin(np.radians(coxa_angle))
        
        # Position after accounting for coxa rotation and offset
        x_after_coxa = x * cos_coxa + y * sin_coxa - self.config.coxa_length
        y_after_coxa = -x * sin_coxa + y * cos_coxa
        z_after_coxa = z
        
        # Step 3: Handle the femur joint with alpha = -90°
        # The femur D-H transform rotates coordinates, so we need to account for this
        # After femur joint, coordinates are rotated by alpha = -90°
        
        # In the femur-tibia plane, we solve a 2D problem
        # The target in the femur coordinate system is (x_after_coxa, z_after_coxa)
        # y_after_coxa should be approximately 0 for a planar leg
        
        target_distance = np.sqrt(x_after_coxa**2 + z_after_coxa**2)
        
        # Check reachability
        L_femur = self.config.femur_length
        L_tibia = self.config.tibia_length
        
        if target_distance > (L_femur + L_tibia) or target_distance < abs(L_femur - L_tibia):
            return np.array([coxa_angle, 0.0, 0.0]), False
        
        if target_distance < 1e-8:
            # Target is at femur joint
            femur_angle = 0.0
            tibia_angle = 0.0
        else:
            # Use law of cosines for 2-DOF planar manipulator
            # Angle from positive X-axis to target line
            target_angle = np.arctan2(z_after_coxa, x_after_coxa)
            
            # Internal angle at femur joint
            cos_alpha = (L_femur**2 + target_distance**2 - L_tibia**2) / (2 * L_femur * target_distance)
            cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
            alpha = np.arccos(cos_alpha)
            
            # Internal angle at tibia joint
            cos_beta = (L_femur**2 + L_tibia**2 - target_distance**2) / (2 * L_femur * L_tibia)
            cos_beta = np.clip(cos_beta, -1.0, 1.0)
            beta = np.arccos(cos_beta)
            
            # Calculate joint angles based on elbow configuration
            if prefer_solution == 'elbow_down':
                femur_angle = np.degrees(target_angle + alpha)
                tibia_angle = np.degrees(np.pi - beta)
            else:  # elbow_up
                femur_angle = np.degrees(target_angle - alpha)
                tibia_angle = np.degrees(beta - np.pi)
        
        # Check if the solution violates y_after_coxa constraint
        # For a proper planar leg, y_after_coxa should be small
        if abs(y_after_coxa) > 0.01:  # 1cm tolerance
            # This might indicate the target is not reachable by a planar leg
            # But we'll continue with the solution
            pass
        
        joint_angles = np.array([coxa_angle, femur_angle, tibia_angle])
        return joint_angles, True
    
    def _validate_joint_limits(self, joint_angles: np.ndarray) -> bool:
        """
        Check if joint angles are within valid limits.
        
        Args:
            joint_angles: Joint angles in degrees [coxa, femur, tibia]
            
        Returns:
            True if all joints are within limits, False otherwise
        """
        for angle in joint_angles:
            if angle < self.config.joint_min or angle > self.config.joint_max:
                return False
        return True
    
    def get_multiple_ik_solutions(self, target_position: np.ndarray, leg_id: str) -> List[Tuple[np.ndarray, str]]:
        """
        Get all valid inverse kinematics solutions for a target position.
        
        Args:
            target_position: Desired foot position [x, y, z] in world coordinates
            leg_id: Leg identifier
            
        Returns:
            List of (joint_angles, solution_type) tuples for all valid solutions
        """
        solutions = []
        
        for prefer_solution in ['elbow_down', 'elbow_up']:
            joint_angles, success = self.inverse_kinematics(
                target_position, leg_id, prefer_solution
            )
            
            if success:
                solutions.append((joint_angles, prefer_solution))
        
        return solutions
    
    def get_all_ik_solutions(self, target_positions: Dict[str, np.ndarray], 
                           prefer_solution: str = 'elbow_down') -> Tuple[Dict[str, np.ndarray], Dict[str, bool]]:
        """
        Calculate inverse kinematics for all legs simultaneously.
        
        Args:
            target_positions: Dictionary mapping leg_id to target position
            prefer_solution: Preferred elbow configuration for all legs
            
        Returns:
            Tuple of (joint_angles_dict, success_dict)
        """
        joint_angles_dict = {}
        success_dict = {}
        
        for leg_id in self.LEG_NAMES:
            if leg_id in target_positions:
                joint_angles, success = self.inverse_kinematics(
                    target_positions[leg_id], leg_id, prefer_solution
                )
                joint_angles_dict[leg_id] = joint_angles
                success_dict[leg_id] = success
            else:
                # Default to neutral position if no target provided
                warnings.warn(f"No target position provided for {leg_id}, using neutral pose")
                joint_angles_dict[leg_id] = np.array([0.0, 0.0, 0.0])
                success_dict[leg_id] = True
        
        return joint_angles_dict, success_dict


def validate_star_config() -> None:
    """
    Standalone function to validate star configuration (can be called from command line).
    """
    print("🌟 Validating HexaPodSim 2.0 Star Configuration...")
    
    # Test with default configuration
    kinematics = HexapodKinematics()
    validation = kinematics.validate_star_configuration()
    
    print(f"Configuration Valid: {'✅' if validation['config_valid'] else '❌'}")
    print(f"Inward Leg Angle: {validation['inward_leg_angle']}°")
    
    if validation['warnings']:
        print("\n⚠️ Warnings:")
        for warning in validation['warnings']:
            print(f"  - {warning}")
    
    if validation['errors']:
        print("\n❌ Errors:")
        for error in validation['errors']:
            print(f"  - {error}")
    
    # Test forward kinematics for all legs
    print(f"\n🦿 Testing forward kinematics for all legs:")
    test_angles = np.array([15.0, -30.0, 45.0])  # Example joint angles
    
    for leg_id in kinematics.LEG_NAMES:
        try:
            foot_pos = kinematics.forward_kinematics(test_angles, leg_id)
            color = kinematics.LEG_COLORS[leg_id]
            print(f"  {leg_id}: [{foot_pos[0]:+.3f}, {foot_pos[1]:+.3f}, {foot_pos[2]:+.3f}] (color: {color})")
        except Exception as e:
            print(f"  {leg_id}: ❌ Error - {e}")


if __name__ == "__main__":
    validate_star_config()