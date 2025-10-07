#!/usr/bin/env python3
"""
Unit tests for HexaPodSim 2.0 Forward Kinematics

This module contains comprehensive tests for the star configuration kinematics
implementation, validating mathematical accuracy and edge cases.

Run with: python -m pytest tests/test_kinematics.py -v
Or standalone: python tests/test_kinematics.py
"""

import sys
import os
import numpy as np
import unittest
from pathlib import Path

# Add hexapod module to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from hexapod.kinematics import HexapodKinematics, RobotConfig


class TestHexapodKinematics(unittest.TestCase):
    """Test cases for hexapod forward kinematics with star configuration."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.kinematics = HexapodKinematics()
        self.test_tolerance = 1e-6  # Numerical precision tolerance
    
    def test_initialization(self):
        """Test proper initialization of kinematics system."""
        # Test default configuration
        self.assertEqual(self.kinematics.config.inward_leg_angle, 15.0)
        self.assertEqual(len(self.kinematics.LEG_NAMES), 6)
        self.assertEqual(len(self.kinematics.leg_positions), 6)
        self.assertEqual(len(self.kinematics.star_rotations), 6)
        
        # Test custom configuration
        custom_config = RobotConfig(inward_leg_angle=20.0)
        custom_kinematics = HexapodKinematics(custom_config)
        self.assertEqual(custom_kinematics.config.inward_leg_angle, 20.0)
    
    def test_star_configuration_angles(self):
        """Test that star configuration angles are set correctly."""
        expected_rotations = {
            'L1': +15.0,  # Front left: forward
            'R1': -15.0,  # Front right: forward (mirrored)
            'L2': 0.0,    # Middle left: perpendicular
            'R2': 0.0,    # Middle right: perpendicular
            'L3': -15.0,  # Back left: backward
            'R3': +15.0   # Back right: backward (mirrored)
        }
        
        for leg_id, expected_angle in expected_rotations.items():
            actual_angle = self.kinematics.star_rotations[leg_id]
            self.assertAlmostEqual(actual_angle, expected_angle, places=3,
                                 msg=f"Star rotation for {leg_id} incorrect")
    
    def test_leg_positions(self):
        """Test that leg attachment positions are correct."""
        config = self.kinematics.config
        half_length = config.body_length / 2  # 0.10m
        half_width = config.body_width / 2    # 0.075m
        
        expected_positions = {
            'L1': [+half_length, +half_width, 0.0],  # Left Front
            'R1': [+half_length, -half_width, 0.0],  # Right Front
            'L2': [0.0, +half_width, 0.0],           # Left Middle
            'R2': [0.0, -half_width, 0.0],           # Right Middle
            'L3': [-half_length, +half_width, 0.0],  # Left Back
            'R3': [-half_length, -half_width, 0.0]   # Right Back
        }
        
        for leg_id, expected_pos in expected_positions.items():
            actual_pos = self.kinematics.leg_positions[leg_id]
            np.testing.assert_allclose(actual_pos, expected_pos, atol=self.test_tolerance,
                                     err_msg=f"Position for {leg_id} incorrect")
    
    def test_joint_angle_validation(self):
        """Test joint angle validation functionality."""
        # Valid angles
        valid_angles = np.array([0.0, 45.0, -30.0])
        self.assertTrue(self.kinematics.validate_joint_angles(valid_angles, 'L1'))
        
        # Invalid angles (exceeding limits)
        with self.assertRaises(ValueError):
            invalid_angles = np.array([95.0, 0.0, 0.0])  # Exceeds +90°
            self.kinematics.validate_joint_angles(invalid_angles, 'L1')
        
        with self.assertRaises(ValueError):
            invalid_angles = np.array([0.0, -95.0, 0.0])  # Exceeds -90°
            self.kinematics.validate_joint_angles(invalid_angles, 'L1')
        
        # Wrong number of angles
        with self.assertRaises(ValueError):
            wrong_size = np.array([0.0, 0.0])  # Only 2 angles
            self.kinematics.validate_joint_angles(wrong_size, 'L1')
        
        # Invalid leg ID
        with self.assertRaises(ValueError):
            self.kinematics.validate_joint_angles(valid_angles, 'INVALID')
    
    def test_forward_kinematics_neutral_pose(self):
        """Test forward kinematics with neutral joint angles (all zeros)."""
        neutral_angles = np.array([0.0, 0.0, 0.0])
        
        for leg_id in self.kinematics.LEG_NAMES:
            foot_pos = self.kinematics.forward_kinematics(neutral_angles, leg_id)
            
            # Check that result is 3D position
            self.assertEqual(len(foot_pos), 3)
            
            # In neutral pose with all joints at 0°, the foot should be at
            # body_position + star_rotation_offset + sum_of_link_lengths_in_x
            expected_reach = (self.kinematics.config.coxa_length + 
                            self.kinematics.config.femur_length + 
                            self.kinematics.config.tibia_length)
            
            # The actual distance will be affected by the star configuration
            actual_distance = np.linalg.norm(foot_pos - self.kinematics.leg_positions[leg_id])
            
            # Should be close to expected reach (within reasonable tolerance)
            self.assertLess(abs(actual_distance - expected_reach), 0.05,  # 5cm tolerance
                          msg=f"Neutral pose distance for {leg_id} seems incorrect")
    
    def test_forward_kinematics_extreme_poses(self):
        """Test forward kinematics with extreme but valid joint angles."""
        # Test maximum positive angles
        max_angles = np.array([90.0, 90.0, 90.0])
        for leg_id in self.kinematics.LEG_NAMES:
            foot_pos = self.kinematics.forward_kinematics(max_angles, leg_id)
            self.assertEqual(len(foot_pos), 3)
            # Should not produce NaN or infinite values
            self.assertTrue(np.all(np.isfinite(foot_pos)))
        
        # Test maximum negative angles
        min_angles = np.array([-90.0, -90.0, -90.0])
        for leg_id in self.kinematics.LEG_NAMES:
            foot_pos = self.kinematics.forward_kinematics(min_angles, leg_id)
            self.assertEqual(len(foot_pos), 3)
            self.assertTrue(np.all(np.isfinite(foot_pos)))
    
    def test_star_configuration_behavior(self):
        """Test that star configuration creates the expected behavioral patterns."""
        test_angles = np.array([0.0, 0.0, 0.0])  # Neutral pose for clear comparison
        
        # Get all leg positions in neutral pose
        all_positions = {}
        for leg_id in self.kinematics.LEG_NAMES:
            all_positions[leg_id] = self.kinematics.forward_kinematics(test_angles, leg_id)
        
        # Test that legs are positioned correctly relative to body
        l1_pos = all_positions['L1']
        r1_pos = all_positions['R1']
        l2_pos = all_positions['L2']
        r2_pos = all_positions['R2']
        l3_pos = all_positions['L3']
        r3_pos = all_positions['R3']
        
        # All legs should be on opposite sides of Y-axis
        self.assertGreater(l1_pos[1], 0, msg="L1 should be on positive Y side")
        self.assertLess(r1_pos[1], 0, msg="R1 should be on negative Y side")
        self.assertGreater(l2_pos[1], 0, msg="L2 should be on positive Y side")
        self.assertLess(r2_pos[1], 0, msg="R2 should be on negative Y side")
        self.assertGreater(l3_pos[1], 0, msg="L3 should be on positive Y side")
        self.assertLess(r3_pos[1], 0, msg="R3 should be on negative Y side")
        
        # Test that middle legs are perfectly symmetric in neutral pose
        self.assertAlmostEqual(l2_pos[0], r2_pos[0], places=6, msg="Middle legs X should be identical")
        self.assertAlmostEqual(l2_pos[2], r2_pos[2], places=6, msg="Middle legs Z should be identical")
        self.assertAlmostEqual(l2_pos[1], -r2_pos[1], places=6, msg="Middle legs Y should be opposite")
        
        # Test star configuration effect on front vs back legs
        # In neutral pose, all legs extend the same distance, but at different angles
        front_reach = np.linalg.norm([l1_pos[0], l1_pos[1]])  # Distance in XY plane
        middle_reach = np.linalg.norm([l2_pos[0], l2_pos[1]])
        back_reach = np.linalg.norm([l3_pos[0], l3_pos[1]])
        
        # Due to star configuration:
        # - Front legs (angled +15°) should have different XY reach than middle legs
        # - Back legs (angled -15°) should have different XY reach than middle legs
        # - Middle legs (0° angle) should be perpendicular
        
        self.assertNotAlmostEqual(front_reach, middle_reach, places=2,
                                 msg="Front legs should have different reach due to star angle")
        self.assertNotAlmostEqual(back_reach, middle_reach, places=2,
                                 msg="Back legs should have different reach due to star angle")
        
        # Verify that star rotations are applied correctly
        # The difference in X position should reflect the star angle effect
        expected_total_reach = (self.kinematics.config.coxa_length + 
                              self.kinematics.config.femur_length + 
                              self.kinematics.config.tibia_length)
        
        for leg_id, pos in all_positions.items():
            total_distance = np.linalg.norm(pos - self.kinematics.leg_positions[leg_id])
            self.assertAlmostEqual(total_distance, expected_total_reach, places=2,
                                 msg=f"Leg {leg_id} should have expected total reach in neutral pose")
    
    def test_all_foot_positions(self):
        """Test calculation of all foot positions simultaneously."""
        # Create joint angles for all legs
        joint_angles_all = {}
        for i, leg_id in enumerate(self.kinematics.LEG_NAMES):
            # Use different angles for each leg to test variety
            joint_angles_all[leg_id] = np.array([i*10.0, -i*5.0, i*15.0])
        
        # Calculate all positions
        foot_positions = self.kinematics.get_all_foot_positions(joint_angles_all)
        
        # Check that we got results for all legs
        self.assertEqual(len(foot_positions), 6)
        for leg_id in self.kinematics.LEG_NAMES:
            self.assertIn(leg_id, foot_positions)
            self.assertIsNotNone(foot_positions[leg_id])
            self.assertEqual(len(foot_positions[leg_id]), 3)
    
    def test_workspace_calculation(self):
        """Test workspace calculation for a leg."""
        # Calculate workspace for L1 with low resolution for speed
        workspace = self.kinematics.get_leg_workspace('L1', resolution=5)
        
        # Check workspace structure
        self.assertIn('points', workspace)
        self.assertIn('joint_configs', workspace)
        self.assertIn('leg_id', workspace)
        self.assertEqual(workspace['leg_id'], 'L1')
        
        # Check that we have valid workspace points
        points = workspace['points']
        self.assertGreater(len(points), 0)
        self.assertEqual(points.shape[1], 3)  # 3D points
        
        # All points should be finite
        self.assertTrue(np.all(np.isfinite(points)))
    
    def test_configuration_validation(self):
        """Test configuration validation functionality."""
        validation = self.kinematics.validate_star_configuration()
        
        # Check validation structure
        self.assertIn('config_valid', validation)
        self.assertIn('inward_leg_angle', validation)
        self.assertIn('leg_positions', validation)
        self.assertIn('star_rotations', validation)
        self.assertIn('warnings', validation)
        self.assertIn('errors', validation)
        
        # Should be valid with default configuration
        self.assertTrue(validation['config_valid'])
        self.assertEqual(validation['inward_leg_angle'], 15.0)
        
        # Test with extreme angle
        extreme_config = RobotConfig(inward_leg_angle=60.0)
        extreme_kinematics = HexapodKinematics(extreme_config)
        extreme_validation = extreme_kinematics.validate_star_configuration()
        
        # Should have warnings for extreme angle
        self.assertGreater(len(extreme_validation['warnings']), 0)
    
    def test_degree_based_interface(self):
        """Test that all interfaces use degrees consistently."""
        # Test with specific degree values
        test_angles_deg = np.array([30.0, -45.0, 60.0])
        
        for leg_id in self.kinematics.LEG_NAMES:
            # Forward kinematics should accept degrees
            foot_pos = self.kinematics.forward_kinematics(test_angles_deg, leg_id)
            
            # Should produce reasonable position (not affected by radian conversion)
            distance = np.linalg.norm(foot_pos)
            self.assertLess(distance, 1.0)  # Should be within 1 meter
            self.assertGreater(distance, 0.05)  # Should be more than 5cm
        
        # Test validation with degree limits
        self.assertTrue(self.kinematics.validate_joint_angles(
            np.array([89.9, -89.9, 0.0]), 'L1'))
        
        with self.assertRaises(ValueError):
            self.kinematics.validate_joint_angles(
                np.array([90.1, 0.0, 0.0]), 'L1')  # Just over limit
    
    def test_color_mapping(self):
        """Test that leg color mapping is complete and valid."""
        for leg_id in self.kinematics.LEG_NAMES:
            self.assertIn(leg_id, self.kinematics.LEG_COLORS)
            color = self.kinematics.LEG_COLORS[leg_id]
            # Should be hex color format
            self.assertTrue(color.startswith('#'))
            self.assertEqual(len(color), 7)  # #RRGGBB format


def run_performance_test():
    """Run performance tests for kinematics calculations."""
    print("\n🚀 Running Performance Tests...")
    
    kinematics = HexapodKinematics()
    test_angles = np.array([15.0, -30.0, 45.0])
    
    import time
    
    # Test single leg performance
    start_time = time.time()
    iterations = 1000
    
    for _ in range(iterations):
        kinematics.forward_kinematics(test_angles, 'L1')
    
    single_leg_time = (time.time() - start_time) / iterations * 1000  # ms
    print(f"Single leg kinematics: {single_leg_time:.3f} ms (target: < 1.0 ms)")
    
    # Test all legs performance
    joint_angles_all = {leg_id: test_angles for leg_id in kinematics.LEG_NAMES}
    
    start_time = time.time()
    for _ in range(iterations):
        kinematics.get_all_foot_positions(joint_angles_all)
    
    all_legs_time = (time.time() - start_time) / iterations * 1000  # ms
    print(f"All legs kinematics: {all_legs_time:.3f} ms (target: < 6.0 ms)")
    
    # Performance checks
    if single_leg_time < 1.0:
        print("✅ Single leg performance meets target")
    else:
        print("⚠️ Single leg performance may be too slow for real-time")
    
    if all_legs_time < 6.0:
        print("✅ All legs performance meets target")
    else:
        print("⚠️ All legs performance may be too slow for real-time")


if __name__ == "__main__":
    # Run unit tests
    print("🧪 Running HexaPodSim 2.0 Kinematics Tests...")
    
    # Create test suite
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(TestHexapodKinematics)
    
    # Run tests with verbose output
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Performance tests
    run_performance_test()
    
    # Summary
    print(f"\n📊 Test Summary:")
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    
    if result.wasSuccessful():
        print("🎉 All tests passed! Kinematics implementation is ready.")
    else:
        print("❌ Some tests failed. Please review the implementation.")
        sys.exit(1)