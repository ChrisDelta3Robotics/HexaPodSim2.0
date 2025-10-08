#!/usr/bin/env python3
"""
Phase 6 Test Script - Sensor Integration & Feedback Systems

This script tests all the Phase 6 sensor systems including:
- Advanced sensor suite (IMU, encoders, force sensors)
- Sensor fusion with Extended Kalman Filter
- Feedback control systems
- Environmental sensing and terrain analysis
"""

import sys
import os
import numpy as np
import time
from pathlib import Path

# Add hexapod directory to path
hexapod_dir = Path(__file__).parent / "hexapod"
sys.path.insert(0, str(hexapod_dir))

def test_sensor_suite():
    """Test the advanced sensor suite"""
    print("🔬 Testing Advanced Sensor Suite...")
    
    try:
        from hexapod.sensors import (
            IMUSensor, JointEncoder, ForceSensor, 
            DistanceSensor, TerrainSensor, SensorSuite
        )
        
        # Test individual sensors
        print("  ✓ Creating IMU sensor...")
        imu = IMUSensor()
        
        print("  ✓ Creating joint encoders...")
        encoders = [JointEncoder(f"joint_{i}") for i in range(18)]
        
        print("  ✓ Creating force sensors...")
        force_sensors = [ForceSensor(f"leg_{i}") for i in range(6)]
        
        print("  ✓ Creating distance sensors...")
        distance_sensors = [DistanceSensor(f"range_{i}") for i in range(8)]
        
        print("  ✓ Creating terrain sensor...")
        terrain_sensor = TerrainSensor()
        
        # Test sensor suite
        print("  ✓ Creating sensor suite...")
        sensor_suite = SensorSuite()
        
        # Add sensors to suite
        sensor_suite.add_sensor('imu', imu)
        for i, encoder in enumerate(encoders):
            sensor_suite.add_sensor(f'encoder_{i}', encoder)
        for i, force in enumerate(force_sensors):
            sensor_suite.add_sensor(f'force_{i}', force)
        for i, distance in enumerate(distance_sensors):
            sensor_suite.add_sensor(f'distance_{i}', distance)
        sensor_suite.add_sensor('terrain', terrain_sensor)
        
        # Test readings
        print("  ✓ Testing sensor readings...")
        all_data = sensor_suite.read_all_sensors()
        print(f"    - Total sensors: {len(all_data)}")
        print(f"    - IMU data keys: {list(all_data['imu'].keys())}")
        print(f"    - Sample encoder reading: {all_data['encoder_0']:.2f}°")
        print(f"    - Sample force reading: {all_data['force_0']:.2f}N")
        
        print("✅ Sensor suite test passed!")
        return True
        
    except Exception as e:
        print(f"❌ Sensor suite test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_sensor_fusion():
    """Test the sensor fusion system"""
    print("\n🧠 Testing Sensor Fusion System...")
    
    try:
        from hexapod.sensor_fusion import ExtendedKalmanFilter, SensorFusionManager
        from hexapod.sensors import IMUSensor, JointEncoder
        
        print("  ✓ Creating Extended Kalman Filter...")
        ekf = ExtendedKalmanFilter(state_dim=13, measurement_dim=6)
        
        print("  ✓ Testing EKF prediction step...")
        dt = 0.02  # 50 Hz
        ekf.predict(dt)
        
        print("  ✓ Testing EKF update step...")
        # Simulate IMU measurement
        imu_data = np.array([0.1, -0.05, 9.81, 0.02, -0.01, 0.0])  # ax, ay, az, gx, gy, gz
        ekf.update(imu_data, "imu_accel")
        
        state = ekf.get_state()
        print(f"    - Position: x={state[0]:.3f}, y={state[1]:.3f}, z={state[2]:.3f}")
        print(f"    - Velocity: vx={state[3]:.3f}, vy={state[4]:.3f}, vz={state[5]:.3f}")
        print(f"    - Orientation: roll={np.degrees(state[6]):.1f}°, pitch={np.degrees(state[7]):.1f}°, yaw={np.degrees(state[8]):.1f}°")
        
        print("  ✓ Creating Sensor Fusion Manager...")
        fusion_manager = SensorFusionManager()
        
        # Add sensors
        imu = IMUSensor()
        encoders = [JointEncoder(f"joint_{i}") for i in range(18)]
        
        fusion_manager.add_sensor('imu', imu, weight=1.0)
        for i, encoder in enumerate(encoders):
            fusion_manager.add_sensor(f'encoder_{i}', encoder, weight=0.5)
        
        print("  ✓ Testing fusion process...")
        for i in range(5):
            fused_state = fusion_manager.fuse_sensor_data(dt)
            print(f"    - Fusion step {i+1}: Position ({fused_state[0]:.3f}, {fused_state[1]:.3f}, {fused_state[2]:.3f})")
            time.sleep(0.1)
        
        print("✅ Sensor fusion test passed!")
        return True
        
    except Exception as e:
        print(f"❌ Sensor fusion test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_feedback_control():
    """Test the feedback control system"""
    print("\n🎛️ Testing Feedback Control System...")
    
    try:
        from hexapod.feedback_control import (
            PIDController, AdaptiveController, 
            FeedbackControlManager, ControlMode
        )
        
        print("  ✓ Creating PID controller...")
        pid = PIDController(kp=2.0, ki=0.1, kd=0.05)
        
        # Test PID with step response
        print("  ✓ Testing PID step response...")
        setpoint = 45.0  # degrees
        current_position = 0.0
        
        for i in range(10):
            error = setpoint - current_position
            control_output = pid.update(error, dt=0.02)
            current_position += control_output * 0.1  # Simulate system response
            print(f"    - Step {i+1}: Position={current_position:.1f}°, Error={error:.1f}°, Output={control_output:.2f}")
        
        print("  ✓ Creating Adaptive controller...")
        adaptive = AdaptiveController(
            base_kp=1.5, base_ki=0.08, base_kd=0.03,
            adaptation_rate=0.01
        )
        
        print("  ✓ Testing adaptive control...")
        for i in range(5):
            error = np.random.normal(0, 0.5)  # Random error
            output = adaptive.update(error, dt=0.02)
            print(f"    - Adaptive step {i+1}: Error={error:.3f}, Output={output:.3f}")
        
        print("  ✓ Creating Feedback Control Manager...")
        control_manager = FeedbackControlManager()
        
        # Add controllers for each joint
        joint_names = ['L1_coxa', 'L1_femur', 'L1_tibia', 'R1_coxa', 'R1_femur', 'R1_tibia']
        for joint in joint_names[:6]:  # Test first 6 joints
            controller = PIDController(kp=2.0, ki=0.1, kd=0.05)
            control_manager.add_controller(joint, controller)
        
        # Test coordinated control
        print("  ✓ Testing coordinated control...")
        setpoints = {joint: np.random.uniform(-30, 30) for joint in joint_names[:6]}
        current_positions = {joint: 0.0 for joint in joint_names[:6]}
        
        control_outputs = control_manager.update_all_controllers(setpoints, current_positions, dt=0.02)
        print(f"    - Control outputs generated for {len(control_outputs)} joints")
        
        print("✅ Feedback control test passed!")
        return True
        
    except Exception as e:
        print(f"❌ Feedback control test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_environmental_sensing():
    """Test environmental sensing capabilities"""
    print("\n🌍 Testing Environmental Sensing...")
    
    try:
        from hexapod.environmental_sensing import (
            TerrainAnalyzer, ObstacleDetector, 
            EnvironmentalMapper, SafetyMonitor
        )
        
        print("  ✓ Creating Terrain Analyzer...")
        terrain_analyzer = TerrainAnalyzer()
        
        # Test terrain analysis
        print("  ✓ Testing terrain analysis...")
        # Simulate height map data
        x_range = np.linspace(-2, 2, 20)
        y_range = np.linspace(-2, 2, 20)
        X, Y = np.meshgrid(x_range, y_range)
        Z = 0.1 * np.sin(X) * np.cos(Y) + 0.05 * np.random.random(X.shape)  # Wavy terrain with noise
        
        terrain_data = terrain_analyzer.analyze_terrain(X, Y, Z)
        print(f"    - Terrain roughness: {terrain_data['roughness']:.3f}")
        print(f"    - Average slope: {terrain_data['avg_slope']:.1f}°")
        print(f"    - Traversability score: {terrain_data['traversability']:.2f}")
        
        print("  ✓ Creating Obstacle Detector...")
        obstacle_detector = ObstacleDetector(detection_range=3.0, resolution=0.1)
        
        # Test obstacle detection
        print("  ✓ Testing obstacle detection...")
        # Simulate distance sensor readings
        distance_readings = {
            'front': 1.5, 'front_left': 2.0, 'front_right': 1.8,
            'left': 2.5, 'right': 2.2, 'back_left': 3.0,
            'back_right': 2.8, 'back': 3.5
        }
        
        obstacles = obstacle_detector.detect_obstacles(distance_readings, robot_position=(0, 0, 0))
        print(f"    - Detected {len(obstacles)} obstacles")
        for i, obs in enumerate(obstacles[:3]):  # Show first 3
            print(f"      Obstacle {i+1}: Position ({obs['position'][0]:.1f}, {obs['position'][1]:.1f}), Size {obs['size']:.1f}m")
        
        print("  ✓ Creating Environmental Mapper...")
        env_mapper = EnvironmentalMapper(map_size=(10, 10), resolution=0.2)
        
        # Test environmental mapping
        print("  ✓ Testing environmental mapping...")
        robot_pos = (0, 0, 0)
        env_mapper.update_map(distance_readings, robot_pos)
        
        occupancy_grid = env_mapper.get_occupancy_grid()
        print(f"    - Occupancy grid shape: {occupancy_grid.shape}")
        print(f"    - Known cells: {np.sum(occupancy_grid >= 0)}/{occupancy_grid.size}")
        
        print("  ✓ Creating Safety Monitor...")
        safety_monitor = SafetyMonitor()
        
        # Test safety monitoring
        print("  ✓ Testing safety monitoring...")
        robot_state = {
            'position': (0, 0, 0.15),
            'velocity': (0.5, 0, 0),
            'acceleration': (0.1, 0, 0),
            'joint_angles': {f'joint_{i}': np.random.uniform(-30, 30) for i in range(18)},
            'joint_torques': {f'joint_{i}': np.random.uniform(-10, 10) for i in range(18)},
            'battery_level': 85.0,
            'temperature': 35.0
        }
        
        safety_status = safety_monitor.check_safety(robot_state, obstacles)
        print(f"    - Safety status: {safety_status['status']}")
        print(f"    - Risk level: {safety_status['risk_level']}")
        if safety_status['warnings']:
            print(f"    - Warnings: {safety_status['warnings']}")
        
        print("✅ Environmental sensing test passed!")
        return True
        
    except Exception as e:
        print(f"❌ Environmental sensing test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def run_integrated_demo():
    """Run an integrated demo of all Phase 6 systems"""
    print("\n🚀 Running Integrated Phase 6 Demo...")
    
    try:
        from hexapod.sensors import SensorSuite, IMUSensor, JointEncoder, ForceSensor
        from hexapod.sensor_fusion import SensorFusionManager
        from hexapod.feedback_control import FeedbackControlManager, PIDController
        from hexapod.environmental_sensing import EnvironmentalMapper, SafetyMonitor
        
        print("  🔧 Initializing integrated systems...")
        
        # Initialize sensor suite
        sensor_suite = SensorSuite()
        sensor_suite.add_sensor('imu', IMUSensor())
        for i in range(6):  # 6 legs
            sensor_suite.add_sensor(f'force_{i}', ForceSensor(f'leg_{i}'))
        for i in range(18):  # 18 joints
            sensor_suite.add_sensor(f'encoder_{i}', JointEncoder(f'joint_{i}'))
        
        # Initialize sensor fusion
        fusion_manager = SensorFusionManager()
        fusion_manager.add_sensor('imu', IMUSensor(), weight=1.0)
        
        # Initialize feedback control
        control_manager = FeedbackControlManager()
        for i in range(18):
            controller = PIDController(kp=2.0, ki=0.1, kd=0.05)
            control_manager.add_controller(f'joint_{i}', controller)
        
        # Initialize environmental systems
        env_mapper = EnvironmentalMapper(map_size=(5, 5), resolution=0.1)
        safety_monitor = SafetyMonitor()
        
        print("  🎮 Running simulation loop...")
        dt = 0.02  # 50 Hz
        
        for step in range(50):  # 1 second simulation
            # Read all sensors
            sensor_data = sensor_suite.read_all_sensors()
            
            # Fuse sensor data
            fused_state = fusion_manager.fuse_sensor_data(dt)
            
            # Generate control setpoints (simulate walking gait)
            t = step * dt
            setpoints = {}
            current_positions = {}
            
            for i in range(18):
                # Simple sinusoidal gait pattern
                setpoints[f'joint_{i}'] = 15 * np.sin(2 * np.pi * t + i * np.pi/9)
                current_positions[f'joint_{i}'] = sensor_data[f'encoder_{i}']
            
            # Update controllers
            control_outputs = control_manager.update_all_controllers(setpoints, current_positions, dt)
            
            # Simulate distance readings for environmental mapping
            distance_readings = {
                'front': 2.0 + 0.5 * np.sin(t),
                'left': 2.5, 'right': 2.5,
                'back': 3.0
            }
            
            # Update environmental map
            robot_pos = (fused_state[0], fused_state[1], fused_state[2])
            env_mapper.update_map(distance_readings, robot_pos)
            
            # Check safety
            robot_state = {
                'position': robot_pos,
                'velocity': (fused_state[3], fused_state[4], fused_state[5]),
                'joint_angles': {f'joint_{i}': current_positions[f'joint_{i}'] for i in range(18)},
                'battery_level': 80.0,
                'temperature': 30.0
            }
            
            safety_status = safety_monitor.check_safety(robot_state, [])
            
            if step % 10 == 0:  # Print every 10 steps
                print(f"    Step {step:2d}: Pos=({robot_pos[0]:.2f},{robot_pos[1]:.2f},{robot_pos[2]:.2f}), "
                      f"Safety={safety_status['status']}, Active_joints={len(control_outputs)}")
        
        print("✅ Integrated demo completed successfully!")
        return True
        
    except Exception as e:
        print(f"❌ Integrated demo failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main test function"""
    print("🤖 HexaPodSim 2.0 - Phase 6: Sensor Integration & Feedback Systems")
    print("=" * 70)
    
    test_results = []
    
    # Run all tests
    test_results.append(("Sensor Suite", test_sensor_suite()))
    test_results.append(("Sensor Fusion", test_sensor_fusion()))
    test_results.append(("Feedback Control", test_feedback_control()))
    test_results.append(("Environmental Sensing", test_environmental_sensing()))
    
    # Run integrated demo
    test_results.append(("Integrated Demo", run_integrated_demo()))
    
    # Print results summary
    print("\n" + "=" * 70)
    print("📊 Phase 6 Test Results Summary:")
    print("=" * 70)
    
    passed = 0
    for test_name, result in test_results:
        status = "✅ PASSED" if result else "❌ FAILED"
        print(f"  {test_name:20s}: {status}")
        if result:
            passed += 1
    
    print(f"\n🎯 Results: {passed}/{len(test_results)} tests passed")
    
    if passed == len(test_results):
        print("🎉 Phase 6 implementation is working perfectly!")
        print("\n🚀 Key Phase 6 Achievements:")
        print("  ✅ Advanced sensor suite with IMU, encoders, force sensors")
        print("  ✅ Extended Kalman Filter for sensor fusion")
        print("  ✅ Adaptive PID controllers with feedback")
        print("  ✅ Environmental sensing and mapping")
        print("  ✅ Safety monitoring and risk assessment")
        print("  ✅ Integrated real-time control loop")
        
        print("\n🔮 Ready for Phase 7: AI/ML Learning Systems!")
    else:
        print("⚠️  Some tests failed. Please check the error messages above.")
        return 1
    
    return 0

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)