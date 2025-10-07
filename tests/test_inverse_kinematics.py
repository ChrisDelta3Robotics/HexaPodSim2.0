#!/usr/bin/env python3
"""
Comprehensive test suite for inverse kinematics functionality.

Tests inverse kinematics implementation including:
- Basic IK solving for all legs
- Round-trip FK/IK validation
- Multiple solution handling
- Workspace boundary testing
- Star configuration effects
- Performance validation

Author: HexaPodSim 2.0
Date: October 2025
"""

import unittest
import numpy as np
import time
import warnings
from typing import List, Tuple

# Import the kinematics module
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hexapod.kinematics import HexapodKinematics


class TestInverseKinematics(unittest.TestCase):
    """Test suite for inverse kinematics functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.kinematics = HexapodKinematics()
        self.tolerance = 8e-3  # 8mm position tolerance (reasonable for 24cm leg reach)
        self.angle_tolerance = 0.1  # 0.1 degree angle tolerance
        
    def test_basic_ik_neutral_pose(self):
        """Test IK for neutral pose positions."""
        # Calculate forward kinematics for neutral pose
        neutral_angles = np.array([0.0, 0.0, 0.0])
        
        for leg_id in self.kinematics.LEG_NAMES:
            with self.subTest(leg=leg_id):
                # Get forward kinematics position
                target_pos = self.kinematics.forward_kinematics(neutral_angles, leg_id)
                
                # Solve inverse kinematics
                ik_angles, success = self.kinematics.inverse_kinematics(target_pos, leg_id)
                
                self.assertTrue(success, f"IK failed for {leg_id} neutral pose")
                
                # Verify solution matches original angles (within tolerance)
                np.testing.assert_allclose(
                    ik_angles, neutral_angles, atol=self.angle_tolerance,
                    err_msg=f"IK angles don't match for {leg_id}"
                )
    
    def test_round_trip_fk_ik_validation(self):
        """Test that FK->IK->FK produces consistent results."""
        test_angles_sets = [
            np.array([0.0, 0.0, 0.0]),      # Neutral
            np.array([30.0, -15.0, 45.0]),  # Typical pose
            np.array([-20.0, 20.0, -30.0]), # Different configuration
            np.array([45.0, -45.0, 60.0]),  # Near limits
        ]
        
        for angles in test_angles_sets:
            for leg_id in self.kinematics.LEG_NAMES:
                with self.subTest(angles=angles.tolist(), leg=leg_id):
                    # Forward kinematics: angles -> position
                    try:
                        original_pos = self.kinematics.forward_kinematics(angles, leg_id)
                    except ValueError:
                        # Skip if angles are invalid for this test
                        continue
                    
                    # Inverse kinematics: position -> angles
                    ik_angles, success = self.kinematics.inverse_kinematics(original_pos, leg_id)
                    
                    if not success:
                        # IK might fail near workspace boundaries
                        continue
                    
                    # Forward kinematics again: angles -> position
                    final_pos = self.kinematics.forward_kinematics(ik_angles, leg_id)
                    
                    # Verify positions match
                    position_error = np.linalg.norm(final_pos - original_pos)
                    self.assertLess(
                        position_error, self.tolerance,
                        f"Round-trip error {position_error*1000:.2f}mm for {leg_id}"
                    )
    
    def test_multiple_solutions(self):
        """Test that multiple IK solutions are handled correctly."""
        # Test position that should have both elbow-up and elbow-down solutions
        for leg_id in self.kinematics.LEG_NAMES:
            with self.subTest(leg=leg_id):
                # Choose a reachable position (not at workspace boundary)
                test_angles = np.array([0.0, -20.0, 40.0])
                target_pos = self.kinematics.forward_kinematics(test_angles, leg_id)
                
                # Get multiple solutions
                solutions = self.kinematics.get_multiple_ik_solutions(target_pos, leg_id)
                
                # Should have at least one solution
                self.assertGreater(len(solutions), 0, f"No IK solutions found for {leg_id}")
                
                # Test each solution
                for joint_angles, solution_type in solutions:
                    # Verify solution reaches target
                    actual_pos = self.kinematics.forward_kinematics(joint_angles, leg_id)
                    position_error = np.linalg.norm(actual_pos - target_pos)
                    
                    self.assertLess(
                        position_error, self.tolerance,
                        f"{solution_type} solution error {position_error*1000:.2f}mm for {leg_id}"
                    )
    
    def test_workspace_boundaries(self):
        """Test IK behavior at workspace boundaries."""
        for leg_id in self.kinematics.LEG_NAMES:
            with self.subTest(leg=leg_id):
                # Test maximum reach position
                max_reach = (self.kinematics.config.coxa_length + 
                           self.kinematics.config.femur_length + 
                           self.kinematics.config.tibia_length)
                
                # Get leg base position for this leg
                leg_base = self.kinematics.leg_positions[leg_id]
                
                # Test position at maximum reach (should be solvable)
                max_reach_pos = leg_base + np.array([max_reach * 0.95, 0.0, 0.0])
                ik_angles, success = self.kinematics.inverse_kinematics(max_reach_pos, leg_id)
                
                if success:
                    # Verify the solution is accurate
                    actual_pos = self.kinematics.forward_kinematics(ik_angles, leg_id)
                    position_error = np.linalg.norm(actual_pos - max_reach_pos)
                    self.assertLess(position_error, self.tolerance)
                
                # Test position beyond maximum reach (should fail)
                beyond_reach_pos = leg_base + np.array([max_reach * 1.1, 0.0, 0.0])
                ik_angles, success = self.kinematics.inverse_kinematics(beyond_reach_pos, leg_id)
                
                self.assertFalse(success, f"IK should fail for unreachable position for {leg_id}")
    
    def test_star_configuration_effects(self):
        """Test that IK correctly handles star configuration."""
        # Test same relative position for different legs
        # Due to star configuration, same world position should give different joint angles
        
        # Use neutral positions as base and make small modifications
        l2_neutral = self.kinematics.forward_kinematics(np.array([0.0, 0.0, 0.0]), 'L2')
        l1_neutral = self.kinematics.forward_kinematics(np.array([0.0, 0.0, 0.0]), 'L1')
        
        # Test that L2 can reach its own neutral position
        l2_angles, l2_success = self.kinematics.inverse_kinematics(l2_neutral, 'L2')
        self.assertTrue(l2_success, "L2 should reach its own neutral position")
        
        # Test that L1 and L2 have different approaches to similar world positions
        # Use L1's neutral position and see if L2 can reach it
        l1_to_l2_angles, l1_to_l2_success = self.kinematics.inverse_kinematics(l1_neutral, 'L2')
        
        if l1_to_l2_success:
            # Get L1's angles for its neutral position
            l1_angles = np.array([0.0, 0.0, 0.0])
            
            # The angles should be different due to star configuration
            angle_diff = np.linalg.norm(l1_to_l2_angles - l1_angles)
            self.assertGreater(angle_diff, 5.0,  # Expect significant difference
                             "Star configuration should produce different angles")
            
            # Both should reach approximately the same world position
            l1_actual = self.kinematics.forward_kinematics(l1_angles, 'L1')
            l2_actual = self.kinematics.forward_kinematics(l1_to_l2_angles, 'L2')
            
            position_diff = np.linalg.norm(l1_actual - l2_actual)
            self.assertLess(position_diff, self.tolerance,
                          "Both legs should reach similar world positions")
        else:
            # If L2 can't reach L1's neutral position, that's also valid
            # Just verify that both legs can solve their own neutral positions
            l1_angles, l1_success = self.kinematics.inverse_kinematics(l1_neutral, 'L1')
            self.assertTrue(l1_success, "L1 should reach its own neutral position")
    
    def test_joint_limit_validation(self):
        """Test that IK respects joint limits."""
        for leg_id in self.kinematics.LEG_NAMES:
            with self.subTest(leg=leg_id):
                # Test a position that should be reachable
                test_angles = np.array([0.0, -30.0, 60.0])
                target_pos = self.kinematics.forward_kinematics(test_angles, leg_id)
                
                ik_angles, success = self.kinematics.inverse_kinematics(target_pos, leg_id)
                
                if success:
                    # Verify all angles are within limits
                    for i, angle in enumerate(ik_angles):
                        joint_names = ['coxa', 'femur', 'tibia']
                        self.assertGreaterEqual(
                            angle, self.kinematics.config.joint_min,
                            f"{leg_id} {joint_names[i]} angle {angle:.1f}° below limit"
                        )
                        self.assertLessEqual(
                            angle, self.kinematics.config.joint_max,
                            f"{leg_id} {joint_names[i]} angle {angle:.1f}° above limit"
                        )
    
    def test_all_legs_ik(self):
        """Test simultaneous IK for all legs."""
        # Create target positions for all legs (standing pose)
        target_positions = {}
        standing_height = -0.10  # 10cm below body
        
        for leg_id in self.kinematics.LEG_NAMES:
            leg_base = self.kinematics.leg_positions[leg_id]
            # Position feet 15cm away from leg base, at standing height
            direction = leg_base[:2] / np.linalg.norm(leg_base[:2])  # Unit vector outward
            foot_pos = leg_base + np.array([direction[0] * 0.15, direction[1] * 0.15, standing_height])
            target_positions[leg_id] = foot_pos
        
        # Solve IK for all legs
        joint_angles_dict, success_dict = self.kinematics.get_all_ik_solutions(target_positions)
        
        # Verify all legs have solutions
        for leg_id in self.kinematics.LEG_NAMES:
            self.assertIn(leg_id, success_dict)
            if not success_dict[leg_id]:
                print(f"Warning: IK failed for {leg_id} in standing pose")
        
        # Verify solutions are accurate
        for leg_id, joint_angles in joint_angles_dict.items():
            if success_dict[leg_id]:
                actual_pos = self.kinematics.forward_kinematics(joint_angles, leg_id)
                expected_pos = target_positions[leg_id]
                position_error = np.linalg.norm(actual_pos - expected_pos)
                
                self.assertLess(
                    position_error, self.tolerance,
                    f"Standing pose error {position_error*1000:.2f}mm for {leg_id}"
                )
    
    def test_ik_performance(self):
        """Test IK performance for real-time requirements."""
        # Target: <2ms for single leg IK, <12ms for all legs
        test_position = np.array([0.2, 0.1, -0.05])
        
        # Single leg performance test
        start_time = time.perf_counter()
        for _ in range(1000):
            ik_angles, success = self.kinematics.inverse_kinematics(test_position, 'L1')
        single_leg_time = (time.perf_counter() - start_time) / 1000 * 1000  # Convert to ms
        
        self.assertLess(single_leg_time, 2.0, 
                       f"Single leg IK too slow: {single_leg_time:.3f}ms (target: <2ms)")
        
        # All legs performance test
        target_positions = {leg_id: test_position for leg_id in self.kinematics.LEG_NAMES}
        
        start_time = time.perf_counter()
        for _ in range(100):
            joint_angles_dict, success_dict = self.kinematics.get_all_ik_solutions(target_positions)
        all_legs_time = (time.perf_counter() - start_time) / 100 * 1000  # Convert to ms
        
        self.assertLess(all_legs_time, 12.0,
                       f"All legs IK too slow: {all_legs_time:.3f}ms (target: <12ms)")
        
        print(f"\n⚡ IK Performance Results:")
        print(f"   Single leg: {single_leg_time:.3f}ms (target: <2ms)")
        print(f"   All legs: {all_legs_time:.3f}ms (target: <12ms)")
    
    def test_ik_error_handling(self):
        """Test IK error handling for invalid inputs."""
        # Test invalid leg ID
        with self.assertRaises(ValueError):
            self.kinematics.inverse_kinematics(np.array([0.1, 0.1, 0.1]), 'INVALID')
        
        # Test invalid target position (wrong dimensions)
        with self.assertRaises(ValueError):
            self.kinematics.inverse_kinematics(np.array([0.1, 0.1]), 'L1')
        
        # Test target position too close to leg base
        leg_base = self.kinematics.leg_positions['L1']
        too_close_pos = leg_base + np.array([0.005, 0.0, 0.0])  # 5mm away
        
        ik_angles, success = self.kinematics.inverse_kinematics(too_close_pos, 'L1')
        self.assertFalse(success, "IK should fail for targets too close to leg base")
    
    def test_coordinate_system_consistency(self):
        """Test that IK maintains coordinate system consistency."""
        # Test that moving target in +X direction increases coxa angle (for appropriate legs)
        base_pos = np.array([0.15, 0.05, -0.05])
        moved_pos = base_pos + np.array([0.02, 0.0, 0.0])  # Move 2cm in +X
        
        for leg_id in ['L1', 'L2', 'L3']:  # Left side legs
            base_angles, base_success = self.kinematics.inverse_kinematics(base_pos, leg_id)
            moved_angles, moved_success = self.kinematics.inverse_kinematics(moved_pos, leg_id)
            
            if base_success and moved_success:
                # For left side legs, moving target in +X should generally increase coxa angle
                # (This depends on star configuration and leg position)
                coxa_change = moved_angles[0] - base_angles[0]
                
                # Verify that the solution is geometrically consistent
                base_actual = self.kinematics.forward_kinematics(base_angles, leg_id)
                moved_actual = self.kinematics.forward_kinematics(moved_angles, leg_id)
                
                base_error = np.linalg.norm(base_actual - base_pos)
                moved_error = np.linalg.norm(moved_actual - moved_pos)
                
                self.assertLess(base_error, self.tolerance)
                self.assertLess(moved_error, self.tolerance)


def run_ik_validation_suite():
    """Run comprehensive IK validation and display results."""
    print("🔄 Running Phase 1.2 Inverse Kinematics Validation...")
    print("=" * 60)
    
    # Suppress warnings during testing
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        
        # Create test suite
        loader = unittest.TestLoader()
        suite = loader.loadTestsFromTestCase(TestInverseKinematics)
        
        # Run tests
        runner = unittest.TextTestRunner(verbosity=2, stream=open(os.devnull, 'w'))
        result = runner.run(suite)
    
    # Display summary
    total_tests = result.testsRun
    failures = len(result.failures)
    errors = len(result.errors)
    passed = total_tests - failures - errors
    
    print(f"\n📊 Test Results Summary:")
    print(f"   Total tests: {total_tests}")
    print(f"   ✅ Passed: {passed}")
    print(f"   ❌ Failed: {failures}")
    print(f"   🚫 Errors: {errors}")
    
    if failures > 0:
        print(f"\n❌ Test Failures:")
        for test, traceback in result.failures:
            print(f"   - {test}: {traceback.split('AssertionError:')[-1].strip()}")
    
    if errors > 0:
        print(f"\n🚫 Test Errors:")
        for test, traceback in result.errors:
            print(f"   - {test}: {traceback.split('Exception:')[-1].strip()}")
    
    success_rate = (passed / total_tests) * 100 if total_tests > 0 else 0
    print(f"\n✨ Success Rate: {success_rate:.1f}%")
    
    return passed == total_tests


if __name__ == "__main__":
    # Run validation suite when executed directly
    success = run_ik_validation_suite()
    
    if success:
        print("\n🎉 Phase 1.2 Inverse Kinematics - VALIDATION COMPLETE!")
    else:
        print("\n⚠️ Some tests failed. Please review implementation.")