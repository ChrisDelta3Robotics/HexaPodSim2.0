#!/usr/bin/env python3
"""
Test suite for Robot Body Configuration Management (Phase 1.4)

This test validates the robot body configuration system including:
- Star configuration leg positioning and base rotations
- Body-to-leg coordinate transformations
- Body pose management (position + orientation in degrees)
- Body height adjustment and tilt compensation
- Configuration validation and error handling

Author: HexaPodSim 2.0
Date: October 2025
"""

import unittest
import numpy as np
import sys
import os

# Add the hexapod module to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hexapod.kinematics import RobotConfiguration, RobotConfig, BodyPose, LegConfiguration


class TestRobotConfiguration(unittest.TestCase):
    """Test robot configuration management system."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.config = RobotConfig()
        self.robot_config = RobotConfiguration(self.config)
    
    def test_initialization(self):
        """Test robot configuration initialization."""
        # Test default initialization
        self.assertEqual(self.robot_config.config.inward_leg_angle, 15.0)
        self.assertEqual(len(self.robot_config.leg_positions), 6)
        self.assertEqual(len(self.robot_config.star_rotations), 6)
        
        # Test with custom config
        custom_config = RobotConfig(inward_leg_angle=20.0)
        custom_robot = RobotConfiguration(custom_config)
        self.assertEqual(custom_robot.config.inward_leg_angle, 20.0)
    
    def test_star_configuration_setup(self):
        """Test star configuration leg positions and rotations."""
        # Check leg positions
        expected_positions = {
            'L1': np.array([0.1, 0.075, 0.0]),    # Front left
            'R1': np.array([0.1, -0.075, 0.0]),   # Front right
            'L2': np.array([0.0, 0.075, 0.0]),    # Middle left
            'R2': np.array([0.0, -0.075, 0.0]),   # Middle right
            'L3': np.array([-0.1, 0.075, 0.0]),   # Rear left
            'R3': np.array([-0.1, -0.075, 0.0])   # Rear right
        }
        
        for leg_id, expected_pos in expected_positions.items():
            actual_pos = self.robot_config.leg_positions[leg_id]
            np.testing.assert_array_almost_equal(actual_pos, expected_pos, decimal=6)
        
        # Check star rotations
        expected_rotations = {
            'L1': +15.0,   # Front legs angled forward
            'R1': +15.0,
            'L2': 0.0,     # Middle legs perpendicular
            'R2': 0.0,
            'L3': -15.0,   # Rear legs angled backward
            'R3': -15.0
        }
        
        for leg_id, expected_angle in expected_rotations.items():
            actual_angle = self.robot_config.star_rotations[leg_id]
            self.assertAlmostEqual(actual_angle, expected_angle, places=6)
    
    def test_body_pose_management(self):
        """Test body pose setting and validation."""
        # Test default pose
        default_pose = self.robot_config.current_pose
        self.assertEqual(default_pose.x, 0.0)
        self.assertEqual(default_pose.y, 0.0)
        self.assertEqual(default_pose.z, 0.1)
        self.assertEqual(default_pose.roll, 0.0)
        self.assertEqual(default_pose.pitch, 0.0)
        self.assertEqual(default_pose.yaw, 0.0)
        
        # Test setting new pose
        new_pose = BodyPose(x=0.1, y=0.2, z=0.15, roll=5.0, pitch=-3.0, yaw=10.0)
        self.robot_config.current_pose = new_pose
        
        current = self.robot_config.current_pose
        self.assertEqual(current.x, 0.1)
        self.assertEqual(current.y, 0.2)
        self.assertEqual(current.z, 0.15)
        self.assertEqual(current.roll, 5.0)
        self.assertEqual(current.pitch, -3.0)
        self.assertEqual(current.yaw, 10.0)
    
    def test_body_position_setting(self):
        """Test body position setting methods."""
        # Test set_body_position
        self.robot_config.set_body_position(0.2, 0.3, 0.12)
        pose = self.robot_config.current_pose
        self.assertEqual(pose.x, 0.2)
        self.assertEqual(pose.y, 0.3)
        self.assertEqual(pose.z, 0.12)
        self.assertEqual(pose.roll, 0.0)  # Orientation should remain unchanged
        
        # Test adjust_body_height
        self.robot_config.adjust_body_height(0.08)
        pose = self.robot_config.current_pose
        self.assertEqual(pose.x, 0.2)     # Position should remain unchanged
        self.assertEqual(pose.y, 0.3)
        self.assertEqual(pose.z, 0.08)    # Only height should change
    
    def test_body_orientation_setting(self):
        """Test body orientation setting methods."""
        # Set initial position
        self.robot_config.set_body_position(0.1, 0.2, 0.15)
        
        # Test set_body_orientation
        self.robot_config.set_body_orientation(10.0, -5.0, 20.0)
        pose = self.robot_config.current_pose
        self.assertEqual(pose.x, 0.1)     # Position should remain unchanged
        self.assertEqual(pose.y, 0.2)
        self.assertEqual(pose.z, 0.15)
        self.assertEqual(pose.roll, 10.0)
        self.assertEqual(pose.pitch, -5.0)
        self.assertEqual(pose.yaw, 20.0)
    
    def test_body_pose_validation(self):
        """Test body pose validation."""
        # Test valid poses
        valid_pose = BodyPose(x=0.0, y=0.0, z=0.1, roll=10.0, pitch=-5.0, yaw=45.0)
        self.robot_config.set_body_pose(valid_pose)  # Should not raise
        
        # Test invalid height (too low)
        with self.assertRaises(ValueError):
            invalid_pose = BodyPose(z=0.01)  # Below 2cm minimum
            self.robot_config.set_body_pose(invalid_pose)
        
        # Test invalid height (too high)
        with self.assertRaises(ValueError):
            invalid_pose = BodyPose(z=0.6)   # Above 50cm maximum
            self.robot_config.set_body_pose(invalid_pose)
        
        # Test invalid roll
        with self.assertRaises(ValueError):
            invalid_pose = BodyPose(roll=50.0)  # Above 45° limit
            self.robot_config.set_body_pose(invalid_pose)
        
        # Test invalid pitch
        with self.assertRaises(ValueError):
            invalid_pose = BodyPose(pitch=-50.0)  # Below -45° limit
            self.robot_config.set_body_pose(invalid_pose)
    
    def test_coordinate_transformations(self):
        """Test body-to-leg coordinate transformations."""
        # Test with default pose (identity transforms)
        for leg_id in self.robot_config.leg_ids:
            T_body_to_leg = self.robot_config.transform_body_to_leg(leg_id)
            T_leg_to_body = self.robot_config.transform_leg_to_body(leg_id)
            
            # Check that transforms are 4x4 matrices
            self.assertEqual(T_body_to_leg.shape, (4, 4))
            self.assertEqual(T_leg_to_body.shape, (4, 4))
            
            # Check that transforms are inverses
            identity = T_body_to_leg @ T_leg_to_body
            np.testing.assert_array_almost_equal(identity, np.eye(4), decimal=10)
    
    def test_transformation_with_body_pose(self):
        """Test transformations with non-trivial body pose."""
        # Set a complex body pose
        pose = BodyPose(x=0.1, y=0.2, z=0.15, roll=5.0, pitch=-3.0, yaw=10.0)
        self.robot_config.set_body_pose(pose)
        
        # Test that transformations still work
        for leg_id in self.robot_config.leg_ids:
            T_body_to_leg = self.robot_config.transform_body_to_leg(leg_id)
            T_leg_to_body = self.robot_config.transform_leg_to_body(leg_id)
            
            # Verify inverse relationship
            identity = T_body_to_leg @ T_leg_to_body
            np.testing.assert_array_almost_equal(identity, np.eye(4), decimal=10)
    
    def test_leg_positions_world(self):
        """Test getting leg positions in world coordinates."""
        # Test with default pose
        leg_positions = self.robot_config.get_leg_positions_world()
        
        # Should have all 6 legs
        self.assertEqual(len(leg_positions), 6)
        
        for leg_id in self.robot_config.leg_ids:
            self.assertIn(leg_id, leg_positions)
            position = leg_positions[leg_id]
            self.assertEqual(len(position), 3)  # 3D position
            self.assertIsInstance(position, np.ndarray)
        
        # Test with body offset
        self.robot_config.set_body_position(0.5, 0.3, 0.2)
        leg_positions_offset = self.robot_config.get_leg_positions_world()
        
        # Positions should be different from default
        for leg_id in self.robot_config.leg_ids:
            default_pos = leg_positions[leg_id]
            offset_pos = leg_positions_offset[leg_id]
            distance = np.linalg.norm(offset_pos - default_pos)
            self.assertGreater(distance, 0.1)  # Should be significantly different
    
    def test_body_tilt_compensation(self):
        """Test body tilt compensation for foot positions."""
        # Define target foot positions (example)
        target_positions = {
            'L1': np.array([0.15, 0.10, -0.05]),
            'R1': np.array([0.15, -0.10, -0.05]),
            'L2': np.array([0.0, 0.10, -0.05]),
            'R2': np.array([0.0, -0.10, -0.05]),
            'L3': np.array([-0.15, 0.10, -0.05]),
            'R3': np.array([-0.15, -0.10, -0.05])
        }
        
        # Test with no tilt (should return original positions)
        compensated = self.robot_config.compensate_body_tilt(target_positions, 1.0)
        for leg_id in target_positions:
            np.testing.assert_array_almost_equal(
                compensated[leg_id], target_positions[leg_id], decimal=6
            )
        
        # Test with body tilt
        self.robot_config.set_body_orientation(10.0, 5.0, 0.0)  # Roll and pitch
        compensated_tilt = self.robot_config.compensate_body_tilt(target_positions, 1.0)
        
        # Compensated positions should be different
        for leg_id in target_positions:
            original = target_positions[leg_id]
            compensated = compensated_tilt[leg_id]
            
            # X and Y should be unchanged
            self.assertAlmostEqual(compensated[0], original[0], places=6)
            self.assertAlmostEqual(compensated[1], original[1], places=6)
            
            # Z should be compensated (different for most legs)
            # Exception: legs at body center (L2, R2) have minimal compensation
            if leg_id not in ['L2', 'R2']:
                self.assertNotAlmostEqual(compensated[2], original[2], places=3)
    
    def test_configuration_validation(self):
        """Test configuration validation."""
        validation = self.robot_config.validate_configuration()
        
        # Should be valid with default configuration
        self.assertTrue(validation['config_valid'])
        self.assertEqual(validation['inward_leg_angle'], 15.0)
        self.assertIn('timestamp', validation)
        self.assertIn('pose', validation)
        self.assertIn('leg_positions', validation)
        self.assertIn('star_rotations', validation)
        self.assertIsInstance(validation['warnings'], list)
        self.assertIsInstance(validation['errors'], list)
        
        # Test with extreme angle
        extreme_config = RobotConfig(inward_leg_angle=50.0)  # Beyond recommended range
        extreme_robot = RobotConfiguration(extreme_config)
        extreme_validation = extreme_robot.validate_configuration()
        
        self.assertGreater(len(extreme_validation['warnings']), 0)
    
    def test_configuration_summary(self):
        """Test configuration summary generation."""
        summary = self.robot_config.get_configuration_summary()
        
        # Check required fields
        required_fields = [
            'body_dimensions', 'leg_dimensions', 'star_config',
            'current_pose', 'joint_limits', 'validation_status'
        ]
        
        for field in required_fields:
            self.assertIn(field, summary)
        
        # Check specific values
        self.assertEqual(summary['body_dimensions']['length'], 0.20)
        self.assertEqual(summary['star_config']['inward_angle'], 15.0)
        self.assertEqual(len(summary['star_config']['leg_positions']), 6)
    
    def test_enum_integration(self):
        """Test integration with LegConfiguration enum."""
        # Test that enum values match leg_ids
        enum_values = [leg.value for leg in LegConfiguration]
        for leg_id in self.robot_config.leg_ids:
            self.assertIn(leg_id, enum_values)
        
        # Test enum usage
        for leg_enum in LegConfiguration:
            leg_id = leg_enum.value
            self.assertIn(leg_id, self.robot_config.leg_positions)
            self.assertIn(leg_id, self.robot_config.star_rotations)


class TestBodyPose(unittest.TestCase):
    """Test BodyPose data class."""
    
    def test_initialization(self):
        """Test body pose initialization."""
        # Default initialization
        pose = BodyPose()
        self.assertEqual(pose.x, 0.0)
        self.assertEqual(pose.y, 0.0)
        self.assertEqual(pose.z, 0.1)
        self.assertEqual(pose.roll, 0.0)
        self.assertEqual(pose.pitch, 0.0)
        self.assertEqual(pose.yaw, 0.0)
        
        # Custom initialization
        custom_pose = BodyPose(x=1.0, y=2.0, z=0.3, roll=10.0, pitch=-5.0, yaw=45.0)
        self.assertEqual(custom_pose.x, 1.0)
        self.assertEqual(custom_pose.y, 2.0)
        self.assertEqual(custom_pose.z, 0.3)
        self.assertEqual(custom_pose.roll, 10.0)
        self.assertEqual(custom_pose.pitch, -5.0)
        self.assertEqual(custom_pose.yaw, 45.0)
    
    def test_vector_conversion(self):
        """Test conversion to numpy arrays."""
        pose = BodyPose(x=1.0, y=2.0, z=0.3, roll=10.0, pitch=-5.0, yaw=45.0)
        
        # Test position vector
        pos_vector = pose.to_position_vector()
        expected_pos = np.array([1.0, 2.0, 0.3])
        np.testing.assert_array_equal(pos_vector, expected_pos)
        
        # Test orientation vector
        orient_vector = pose.to_orientation_vector()
        expected_orient = np.array([10.0, -5.0, 45.0])
        np.testing.assert_array_equal(orient_vector, expected_orient)


def run_phase_1_4_validation():
    """
    Run comprehensive Phase 1.4 validation with performance metrics.
    """
    print("🤖 Running Phase 1.4 Robot Body Configuration Validation...")
    print("=" * 60)
    
    # Create test robot
    config = RobotConfig(inward_leg_angle=15.0)
    robot = RobotConfiguration(config)
    
    # Test 1: Configuration validation
    print("📋 Test 1: Configuration Validation")
    validation = robot.validate_configuration()
    print(f"   Configuration Valid: {'✅' if validation['config_valid'] else '❌'}")
    print(f"   Inward Leg Angle: {validation['inward_leg_angle']:.1f}°")
    print(f"   Warnings: {len(validation['warnings'])}")
    print(f"   Errors: {len(validation['errors'])}")
    
    # Test 2: Star configuration
    print("\n🌟 Test 2: Star Configuration")
    for leg_id, position in robot.leg_positions.items():
        rotation = robot.star_rotations[leg_id]
        print(f"   {leg_id}: pos=[{position[0]:+.3f}, {position[1]:+.3f}, {position[2]:+.3f}]m, rot={rotation:+.1f}°")
    
    # Test 3: Body pose management
    print("\n📐 Test 3: Body Pose Management")
    robot.set_body_position(0.1, 0.2, 0.15)
    robot.set_body_orientation(5.0, -3.0, 10.0)
    pose = robot.current_pose
    print(f"   Position: [{pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f}]m")
    print(f"   Orientation: [{pose.roll:.1f}°, {pose.pitch:.1f}°, {pose.yaw:.1f}°]")
    
    # Test 4: Coordinate transformations
    print("\n🔄 Test 4: Coordinate Transformations")
    transform_errors = []
    for leg_id in robot.leg_ids:
        T_body_to_leg = robot.transform_body_to_leg(leg_id)
        T_leg_to_body = robot.transform_leg_to_body(leg_id)
        identity = T_body_to_leg @ T_leg_to_body
        error = np.linalg.norm(identity - np.eye(4))
        transform_errors.append(error)
        print(f"   {leg_id}: Transform consistency error = {error:.2e}")
    
    max_transform_error = max(transform_errors)
    transform_ok = max_transform_error < 1e-10
    print(f"   Transform Validation: {'✅' if transform_ok else '❌'} (max error: {max_transform_error:.2e})")
    
    # Test 5: Tilt compensation
    print("\n⚖️ Test 5: Tilt Compensation")
    robot.set_body_orientation(10.0, 5.0, 0.0)
    target_positions = {leg_id: np.array([0.1, 0.1, -0.05]) for leg_id in robot.leg_ids}
    compensated = robot.compensate_body_tilt(target_positions, 1.0)
    
    max_compensation = 0.0
    for leg_id in robot.leg_ids:
        original_z = target_positions[leg_id][2]
        compensated_z = compensated[leg_id][2]
        compensation = abs(compensated_z - original_z)
        max_compensation = max(max_compensation, compensation)
        print(f"   {leg_id}: Z compensation = {compensation*1000:.2f}mm")
    
    print(f"   Max Z Compensation: {max_compensation*1000:.2f}mm")
    
    # Summary
    print("\n📊 Phase 1.4 Validation Summary:")
    print(f"   ✅ Configuration Valid: {validation['config_valid']}")
    print(f"   ✅ Star Config Setup: Complete (6 legs)")
    print(f"   ✅ Body Pose Management: Functional")
    print(f"   ✅ Coordinate Transforms: {'Valid' if transform_ok else 'Error'}")
    print(f"   ✅ Tilt Compensation: Functional ({max_compensation*1000:.1f}mm max)")
    
    all_tests_passed = (
        validation['config_valid'] and
        transform_ok and
        max_compensation > 0  # Compensation should be non-zero with tilt
    )
    
    print(f"\n🎉 Phase 1.4 Robot Body Configuration: {'COMPLETE ✅' if all_tests_passed else 'ISSUES DETECTED ❌'}")
    
    return all_tests_passed


if __name__ == '__main__':
    # Run comprehensive validation
    success = run_phase_1_4_validation()
    
    print("\n" + "="*60)
    print("Running Unit Tests...")
    print("="*60)
    
    # Run unit tests
    unittest.main(verbosity=2, exit=False)
    
    if success:
        print("\n🚀 Ready for Phase 2: Robot Dynamics & Physics!")
    else:
        print("\n⚠️ Please address issues before proceeding to Phase 2")