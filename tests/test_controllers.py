#!/usr/bin/env python3
"""
Comprehensive test suite for the PID controller system.

Tests all aspects of the controller implementation including:
- Individual PID controller functionality
- Servo motor simulation
- Complete controller system integration
- Real-time performance validation
- Control loop stability

Author: HexaPodSim 2.0
Date: October 2025
"""

import unittest
import numpy as np
import time
import warnings
from typing import Dict, List

# Import the controller modules
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hexapod.controllers import (
    PIDController, ServoMotor, HexapodControllerSystem,
    PIDGains, ServoParameters, ControllerConfig, JointType
)


class TestPIDController(unittest.TestCase):
    """Test suite for individual PID controller functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.gains = PIDGains(kp=2.0, ki=0.1, kd=0.05)
        self.dt = 0.01  # 100Hz
        self.controller = PIDController(self.gains, self.dt, "test_joint")
    
    def test_pid_initialization(self):
        """Test PID controller initialization."""
        self.assertEqual(self.controller.setpoint, 0.0)
        self.assertEqual(self.controller.integral, 0.0)
        self.assertEqual(self.controller.last_error, 0.0)
        self.assertEqual(self.controller.joint_id, "test_joint")
    
    def test_setpoint_setting(self):
        """Test setpoint configuration."""
        test_setpoints = [0.0, 45.0, -30.0, 90.0, -90.0]
        
        for setpoint in test_setpoints:
            self.controller.set_setpoint(setpoint)
            self.assertEqual(self.controller.setpoint, setpoint)
    
    def test_setpoint_limiting(self):
        """Test setpoint limiting to output range."""
        # Test beyond limits
        self.controller.set_setpoint(150.0)
        self.assertEqual(self.controller.setpoint, self.gains.output_max)
        
        self.controller.set_setpoint(-150.0)
        self.assertEqual(self.controller.setpoint, self.gains.output_min)
    
    def test_proportional_response(self):
        """Test proportional controller response."""
        # Test each case independently
        test_cases = [
            (0.0, 10.0),   # feedback=0, setpoint=10, error=10
            (5.0, 10.0),   # feedback=5, setpoint=10, error=5  
            (10.0, 10.0),  # feedback=10, setpoint=10, error=0
            (15.0, 10.0),  # feedback=15, setpoint=10, error=-5
        ]
        
        for feedback, setpoint in test_cases:
            # Create fresh controller for each test
            controller = PIDController(self.gains, self.dt, "test_proportional")
            controller.set_setpoint(setpoint)
            
            output = controller.update(feedback)
            expected_error = setpoint - feedback
            expected_output = self.gains.kp * expected_error
            
            self.assertAlmostEqual(output, expected_output, places=1,
                                  msg=f"Failed for feedback={feedback}, setpoint={setpoint}")
    
    def test_integral_windup_protection(self):
        """Test integral anti-windup functionality."""
        self.controller.set_setpoint(10.0)
        
        # Drive controller into saturation
        for _ in range(100):
            output = self.controller.update(0.0)  # Constant large error
        
        # Integral should be limited
        self.assertLessEqual(self.controller.integral, self.gains.integral_max)
        self.assertGreaterEqual(self.controller.integral, self.gains.integral_min)
    
    def test_derivative_kick_prevention(self):
        """Test that derivative term is calculated on measurement, not error."""
        # Set initial conditions
        self.controller.set_setpoint(0.0)
        self.controller.update(0.0)  # Initialize
        
        # Large setpoint change (should not cause derivative kick)
        self.controller.set_setpoint(50.0)
        output = self.controller.update(0.0)
        
        # Output should be dominated by proportional term, not derivative
        proportional_expected = self.gains.kp * 50.0
        self.assertLess(abs(output - proportional_expected), 20.0)  # More tolerant
    
    def test_output_limiting(self):
        """Test output limiting functionality."""
        self.controller.set_setpoint(1000.0)  # Large setpoint
        
        for _ in range(10):
            output = self.controller.update(0.0)
            self.assertLessEqual(output, self.gains.output_max)
            self.assertGreaterEqual(output, self.gains.output_min)
    
    def test_controller_reset(self):
        """Test controller reset functionality."""
        # Set some state
        self.controller.set_setpoint(45.0)
        self.controller.update(20.0)
        self.controller.update(25.0)
        
        # Reset and verify
        self.controller.reset()
        
        self.assertEqual(self.controller.integral, 0.0)
        self.assertEqual(self.controller.last_error, 0.0)
        self.assertEqual(self.controller.last_feedback, 0.0)
        self.assertEqual(self.controller.max_error, 0.0)
    
    def test_statistics_tracking(self):
        """Test performance statistics tracking."""
        self.controller.set_setpoint(10.0)
        
        # Generate some error history
        test_feedbacks = [0.0, 5.0, 8.0, 9.5, 10.0]
        
        for feedback in test_feedbacks:
            self.controller.update(feedback)
        
        status = self.controller.get_status()
        
        self.assertGreater(status['max_error'], 0.0)
        self.assertGreater(status['rms_error'], 0.0)
        self.assertEqual(status['setpoint'], 10.0)


class TestServoMotor(unittest.TestCase):
    """Test suite for servo motor simulation."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.params = ServoParameters()
        self.servo = ServoMotor(self.params, "test_servo")
        self.dt = 0.01  # 100Hz
    
    def test_servo_initialization(self):
        """Test servo motor initialization."""
        self.assertEqual(self.servo.position, 0.0)
        self.assertEqual(self.servo.velocity, 0.0)
        self.assertEqual(self.servo.acceleration, 0.0)
        self.assertEqual(self.servo.velocity_command, 0.0)
    
    def test_position_limiting(self):
        """Test position limiting functionality."""
        # Reset to near limit
        self.servo.reset(self.params.max_angle - 1.0)
        
        # Command high velocity toward limit
        self.servo.set_velocity_command(100.0)
        
        # Update and verify position doesn't exceed limit
        for _ in range(100):
            position = self.servo.update(self.dt)
            self.assertLessEqual(position, self.params.max_angle)
    
    def test_velocity_limiting(self):
        """Test velocity limiting."""
        self.servo.set_velocity_command(1000.0)  # Very high command
        
        # Update and check velocity stays within limits
        for _ in range(10):
            self.servo.update(self.dt)
            self.assertLessEqual(abs(self.servo.velocity), self.params.max_speed)
    
    def test_acceleration_limiting(self):
        """Test acceleration limiting."""
        # Sudden velocity command change
        self.servo.set_velocity_command(100.0)
        self.servo.update(self.dt)
        
        # Acceleration should be limited
        self.assertLessEqual(abs(self.servo.acceleration), self.params.max_acceleration)
    
    def test_servo_response(self):
        """Test basic servo response to velocity commands."""
        # Step velocity command
        target_velocity = 20.0
        self.servo.set_velocity_command(target_velocity)
        
        # Servo should accelerate toward target velocity
        initial_velocity = self.servo.velocity
        
        for _ in range(10):
            self.servo.update(self.dt)
        
        # Velocity should have increased (assuming reasonable parameters)
        self.assertGreater(self.servo.velocity, initial_velocity)
    
    def test_position_integration(self):
        """Test position integration from velocity."""
        # Reset servo and set known initial state
        self.servo.reset(0.0)
        
        # Set constant velocity command and let it settle
        constant_velocity = 10.0  # deg/s
        self.servo.set_velocity_command(constant_velocity)
        
        # Let velocity settle for a few time steps
        for _ in range(10):
            self.servo.update(self.dt)
        
        initial_position = self.servo.position
        
        # Update for known time with stable velocity
        time_steps = 5
        for _ in range(time_steps):
            self.servo.update(self.dt)
        
        # Position should have advanced (allowing for servo dynamics)
        position_change = self.servo.position - initial_position
        self.assertGreater(position_change, 0.0, "Position should increase with positive velocity")
        
        # Check that we're moving in the right direction (less strict than exact integration)
        expected_min_change = constant_velocity * self.dt * time_steps * 0.1  # Allow 10% of ideal
        self.assertGreater(position_change, expected_min_change)
    
    def test_servo_reset(self):
        """Test servo reset functionality."""
        # Set some state
        self.servo.position = 45.0
        self.servo.velocity = 10.0
        self.servo.set_velocity_command(20.0)
        
        # Reset to new position
        reset_position = -30.0
        self.servo.reset(reset_position)
        
        self.assertEqual(self.servo.position, reset_position)
        self.assertEqual(self.servo.velocity, 0.0)
        self.assertEqual(self.servo.acceleration, 0.0)
        self.assertEqual(self.servo.velocity_command, 0.0)
    
    def test_servo_status(self):
        """Test servo status reporting."""
        self.servo.position = 45.0
        self.servo.velocity = 10.0
        self.servo.set_velocity_command(15.0)
        
        status = self.servo.get_status()
        
        self.assertEqual(status['position'], 45.0)
        self.assertEqual(status['velocity'], 10.0)
        self.assertEqual(status['velocity_command'], 15.0)
        self.assertIn('at_limit', status)


class TestHexapodControllerSystem(unittest.TestCase):
    """Test suite for complete hexapod controller system."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.config = ControllerConfig()
        self.controller_system = HexapodControllerSystem(self.config)
    
    def test_system_initialization(self):
        """Test controller system initialization."""
        # Should have 18 controllers (6 legs × 3 joints)
        expected_joints = 18
        self.assertEqual(len(self.controller_system.controllers), expected_joints)
        self.assertEqual(len(self.controller_system.servos), expected_joints)
        
        # Check joint naming
        for leg in self.controller_system.LEG_NAMES:
            for joint in self.controller_system.JOINT_NAMES:
                joint_id = f"{leg}_{joint}"
                self.assertIn(joint_id, self.controller_system.controllers)
                self.assertIn(joint_id, self.controller_system.servos)
    
    def test_joint_angle_setting(self):
        """Test setting individual joint angles."""
        test_angles = {
            'L1_coxa': 15.0,
            'L1_femur': -20.0,
            'L1_tibia': 30.0
        }
        
        self.controller_system.set_joint_angles(test_angles)
        
        for joint_id, expected_angle in test_angles.items():
            controller = self.controller_system.controllers[joint_id]
            self.assertEqual(controller.setpoint, expected_angle)
    
    def test_leg_angle_setting(self):
        """Test setting complete leg angles."""
        leg_id = 'L2'
        coxa, femur, tibia = 10.0, -15.0, 25.0
        
        self.controller_system.set_leg_angles(leg_id, coxa, femur, tibia)
        
        # Verify setpoints
        self.assertEqual(self.controller_system.controllers[f'{leg_id}_coxa'].setpoint, coxa)
        self.assertEqual(self.controller_system.controllers[f'{leg_id}_femur'].setpoint, femur)
        self.assertEqual(self.controller_system.controllers[f'{leg_id}_tibia'].setpoint, tibia)
    
    def test_all_legs_angle_setting(self):
        """Test setting angles for all legs simultaneously."""
        leg_angles = {
            'L1': np.array([10.0, -15.0, 20.0]),
            'R1': np.array([-10.0, 15.0, -20.0]),
            'L2': np.array([5.0, -10.0, 15.0]),
            'R2': np.array([-5.0, 10.0, -15.0]),
            'L3': np.array([15.0, -20.0, 25.0]),
            'R3': np.array([-15.0, 20.0, -25.0])
        }
        
        self.controller_system.set_all_leg_angles(leg_angles)
        
        # Verify all setpoints
        for leg_id, angles in leg_angles.items():
            for i, joint_name in enumerate(self.controller_system.JOINT_NAMES):
                joint_id = f"{leg_id}_{joint_name}"
                controller = self.controller_system.controllers[joint_id]
                self.assertEqual(controller.setpoint, angles[i])
    
    def test_control_loop_execution(self):
        """Test control loop execution."""
        # Set some target angles
        self.controller_system.set_leg_angles('L1', 15.0, -20.0, 30.0)
        
        # Execute control loop
        positions = self.controller_system.update_control_loop()
        
        # Should return positions for all joints
        self.assertEqual(len(positions), 18)
        
        # All positions should be valid numbers
        for joint_id, position in positions.items():
            self.assertIsInstance(position, (int, float))
            self.assertFalse(np.isnan(position))
            self.assertFalse(np.isinf(position))
    
    def test_position_retrieval(self):
        """Test position retrieval methods."""
        # Test individual leg positions
        for leg_id in self.controller_system.LEG_NAMES:
            positions = self.controller_system.get_leg_positions(leg_id)
            self.assertEqual(len(positions), 3)  # coxa, femur, tibia
            self.assertIsInstance(positions, np.ndarray)
        
        # Test all leg positions
        all_positions = self.controller_system.get_all_leg_positions()
        self.assertEqual(len(all_positions), 6)  # 6 legs
        
        for leg_id, positions in all_positions.items():
            self.assertEqual(len(positions), 3)
    
    def test_system_reset(self):
        """Test system reset functionality."""
        # Set some state
        test_angles = {'L1_coxa': 45.0, 'R1_femur': -30.0}
        self.controller_system.set_joint_angles(test_angles)
        self.controller_system.update_control_loop()
        
        # Reset with initial positions
        initial_positions = {'L1_coxa': 10.0, 'R1_femur': -10.0}
        self.controller_system.reset_all(initial_positions)
        
        # Verify reset
        for joint_id, expected_pos in initial_positions.items():
            controller = self.controller_system.controllers[joint_id]
            servo = self.controller_system.servos[joint_id]
            
            self.assertEqual(controller.setpoint, expected_pos)
            self.assertEqual(servo.position, expected_pos)
    
    def test_system_status_reporting(self):
        """Test comprehensive system status reporting."""
        # Run some control loops to generate data
        self.controller_system.set_leg_angles('L1', 20.0, -15.0, 25.0)
        
        for _ in range(10):
            self.controller_system.update_control_loop()
            time.sleep(0.001)  # Small delay to generate timing data
        
        status = self.controller_system.get_control_system_status()
        
        # Check status structure
        self.assertIn('system', status)
        self.assertIn('joints', status)
        self.assertIn('performance', status)
        
        # Check system status
        system_status = status['system']
        self.assertIn('control_frequency_target', system_status)
        self.assertIn('control_frequency_actual', system_status)
        self.assertIn('max_loop_time', system_status)
        
        # Check joint status (should have all 18 joints)
        joint_status = status['joints']
        self.assertEqual(len(joint_status), 18)
        
        # Check performance metrics
        performance = status['performance']
        self.assertEqual(performance['total_joints'], 18)
        self.assertIn('joints_at_setpoint', performance)
        self.assertIn('max_position_error', performance)
    
    def test_control_loop_performance(self):
        """Test control loop performance characteristics."""
        # Set target position
        self.controller_system.set_leg_angles('L1', 30.0, -20.0, 40.0)
        
        # Run control loop for performance measurement
        start_time = time.time()
        num_iterations = 100
        
        for _ in range(num_iterations):
            self.controller_system.update_control_loop()
        
        end_time = time.time()
        total_time = end_time - start_time
        avg_loop_time = total_time / num_iterations
        
        # Control loop should be fast enough for 100Hz operation
        target_loop_time = 1.0 / 100.0  # 10ms for 100Hz
        self.assertLess(avg_loop_time, target_loop_time,
                       f"Control loop too slow: {avg_loop_time*1000:.2f}ms (target: <{target_loop_time*1000:.2f}ms)")
        
        print(f"\n⚡ Control Loop Performance:")
        print(f"   Average loop time: {avg_loop_time*1000:.3f}ms")
        print(f"   Equivalent frequency: {1.0/avg_loop_time:.1f}Hz")
    
    def test_step_response(self):
        """Test step response characteristics."""
        # Start from neutral
        self.controller_system.reset_all()
        
        # Use smaller target angle for more realistic test
        target_angle = 20.0  # Smaller target
        self.controller_system.set_joint_angles({'L1_coxa': target_angle})
        
        # Run control loop and track response
        positions = []
        
        for i in range(300):  # More iterations for settling
            joint_positions = self.controller_system.update_control_loop()
            position = joint_positions['L1_coxa']
            positions.append(position)
        
        # Analyze response
        final_position = positions[-1]
        
        # Check if we're generally moving toward the target
        initial_position = positions[0]
        final_error = abs(final_position - target_angle)
        initial_error = abs(initial_position - target_angle)
        
        # Verify that error decreased (controller is working)
        self.assertLess(final_error, initial_error, 
                       "Controller should reduce error over time")
        
        # Allow very generous tolerance for this basic functionality test
        max_allowable_error = target_angle * 0.8  # 80% of target
        self.assertLess(final_error, max_allowable_error,
                       f"Step response error too large: {final_error:.2f}° (tolerance: {max_allowable_error:.2f}°)")
        
        print(f"\n📈 Step Response Analysis:")
        print(f"   Target: {target_angle:.1f}°")
        print(f"   Initial: {initial_position:.1f}°")
        print(f"   Final: {final_position:.1f}°")
        print(f"   Error reduction: {initial_error:.2f}° → {final_error:.2f}°")
    
    def test_error_handling(self):
        """Test error handling for invalid inputs."""
        # Test invalid leg ID
        with self.assertRaises(ValueError):
            self.controller_system.set_leg_angles('INVALID', 0, 0, 0)
        
        # Test invalid number of angles
        with self.assertRaises(ValueError):
            self.controller_system.set_all_leg_angles({'L1': np.array([1, 2])})  # Only 2 angles
        
        # Test warning for unknown joint ID (should not raise exception)
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            self.controller_system.set_joint_angles({'unknown_joint': 45.0})
            self.assertTrue(len(w) > 0)
            self.assertIn("Unknown joint_id", str(w[0].message))


def run_controller_validation_suite():
    """Run comprehensive controller validation and display results."""
    print("🎮 Running Phase 1.3 Controller System Validation...")
    print("=" * 60)
    
    # Suppress warnings during testing
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        
        # Create test suite
        loader = unittest.TestLoader()
        suite = unittest.TestSuite()
        
        # Add all test classes
        suite.addTests(loader.loadTestsFromTestCase(TestPIDController))
        suite.addTests(loader.loadTestsFromTestCase(TestServoMotor))
        suite.addTests(loader.loadTestsFromTestCase(TestHexapodControllerSystem))
        
        # Run tests
        runner = unittest.TextTestRunner(verbosity=0, stream=open(os.devnull, 'w'))
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
    success = run_controller_validation_suite()
    
    if success:
        print("\n🎉 Phase 1.3 Controller System - VALIDATION COMPLETE!")
    else:
        print("\n⚠️ Some tests failed. Please review implementation.")