#!/usr/bin/env python3
"""
Phase 6 Simple Test - Test basic sensor functionality first

This script tests the basic sensor components that are working.
"""

import sys
import os
import numpy as np
import time
from pathlib import Path

# Add hexapod directory to path
hexapod_dir = Path(__file__).parent / "hexapod"
sys.path.insert(0, str(hexapod_dir))

def test_basic_sensors():
    """Test the basic sensor functionality"""
    print("🔬 Testing Basic Sensor Components...")
    
    # Test comprehensive sensor suite
    try:
        from hexapod.sensors import (
            IMUSensor, JointEncoder, ForceSensor, 
            DistanceSensor, TerrainSensor, SensorSuite
        )
        
        print("  ✓ Import successful")
        
        # Test individual sensors
        print("  ✓ Creating and testing IMU sensor...")
        imu = IMUSensor("test_imu")
        imu_reading = imu.read()
        print(f"    IMU reading type: {type(imu_reading)}")
        
        print("  ✓ Creating and testing joint encoder...")
        encoder = JointEncoder("test_encoder")
        encoder.set_position(45.0)  # Set to 45 degrees
        encoder_reading = encoder.read()
        print(f"    Encoder reading: {encoder_reading.data:.2f}°")
        
        print("  ✓ Creating and testing force sensor...")
        force = ForceSensor("test_force")
        force.set_force(np.array([0.0, 0.0, -10.0]))  # 10N downward
        force_reading = force.read()
        print(f"    Force reading shape: {force_reading.data.shape}")
        
        print("  ✓ Creating and testing distance sensor...")
        distance = DistanceSensor("test_distance")
        distance.set_distance(2.5)  # 2.5 meters
        distance_reading = distance.read()
        print(f"    Distance reading: {distance_reading.data:.2f}m")
        
        print("  ✓ Creating and testing terrain sensor...")
        terrain = TerrainSensor("test_terrain")
        terrain.set_terrain_data([True, False, True, True, False, True], 0.0, 
                                np.array([0.0, 0.0, 1.0]), 0.1)
        terrain_reading = terrain.read()
        print(f"    Terrain contact count: {sum(terrain_reading.data['ground_contact'])}")
        
        print("  ✓ Creating sensor suite...")
        suite = SensorSuite()
        suite.add_sensor('imu', imu)
        suite.add_sensor('encoder', encoder)
        suite.add_sensor('force', force)
        suite.add_sensor('distance', distance)
        suite.add_sensor('terrain', terrain)
        
        print("  ✓ Testing sensor suite reading...")
        all_data = suite.read_all_sensors()
        print(f"    Suite read {len(all_data)} sensors successfully")
        
        print("✅ Basic sensor test passed!")
        return True
        
    except Exception as e:
        print(f"❌ Basic sensor test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_sensor_fusion_basic():
    """Test basic sensor fusion without complex dependencies"""
    print("\n🧠 Testing Basic Sensor Fusion...")
    
    try:
        from hexapod.sensor_fusion import ExtendedKalmanFilter, SensorFusionManager
        
        print("  ✓ Creating EKF...")
        ekf = ExtendedKalmanFilter(state_dim=13, measurement_dim=6)
        
        print("  ✓ Testing EKF prediction...")
        ekf.predict(0.02)
        
        print("  ✓ Testing EKF with accelerometer data...")
        accel_data = np.array([0.0, 0.0, 9.81])  # Gravity
        ekf.update(accel_data, "imu_accel")
        
        state = ekf.get_state()
        print(f"    Position: ({state[0]:.3f}, {state[1]:.3f}, {state[2]:.3f})")
        
        print("  ✓ Creating sensor fusion manager...")
        fusion_manager = SensorFusionManager()
        
        print("  ✓ Testing fusion with mock sensor...")
        # Create a simple mock sensor
        class MockSensor:
            def read(self):
                from hexapod.sensors import SensorReading
                return SensorReading(
                    sensor_id="mock",
                    timestamp=time.time(),
                    data=np.array([0.0, 0.0, 9.81, 0.0, 0.0, 0.0]),
                    quality=1.0,
                    sensor_type="mock"
                )
        
        mock_sensor = MockSensor()
        fusion_manager.add_sensor('mock_imu', mock_sensor, weight=1.0)
        
        # Test fusion
        fused_state = fusion_manager.fuse_sensor_data(0.02)
        print(f"    Fused state length: {len(fused_state)}")
        
        print("✅ Basic sensor fusion test passed!")
        return True
        
    except Exception as e:
        print(f"❌ Basic sensor fusion test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def run_mini_demo():
    """Run a small integrated demo"""
    print("\n🚀 Running Mini Integration Demo...")
    
    try:
        from hexapod.sensors import SensorSuite, IMUSensor, JointEncoder
        from hexapod.sensor_fusion import SensorFusionManager
        
        print("  🔧 Setting up mini system...")
        
        # Create sensor suite
        suite = SensorSuite()
        imu = IMUSensor("demo_imu")
        encoder = JointEncoder("demo_encoder")
        
        suite.add_sensor('imu', imu)
        suite.add_sensor('encoder', encoder)
        
        # Create fusion manager
        fusion = SensorFusionManager()
        fusion.add_sensor('imu', imu, weight=1.0)
        
        print("  🎮 Running 10-step simulation...")
        
        for step in range(10):
            # Simulate some motion
            encoder.set_position(30 * np.sin(step * 0.1))
            
            # Read sensors
            sensor_data = suite.read_all_sensors()
            
            # Fuse data
            fused_state = fusion.fuse_sensor_data(0.1)
            
            if step % 3 == 0:  # Print every 3rd step
                print(f"    Step {step}: Encoder={sensor_data['encoder']:.1f}°, "
                      f"Fused pos=({fused_state[0]:.3f}, {fused_state[1]:.3f}, {fused_state[2]:.3f})")
        
        print("✅ Mini demo completed successfully!")
        return True
        
    except Exception as e:
        print(f"❌ Mini demo failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main test function"""
    print("🤖 HexaPodSim 2.0 - Phase 6 Basic Test Suite")
    print("=" * 50)
    
    test_results = []
    
    # Run basic tests
    test_results.append(("Basic Sensors", test_basic_sensors()))
    test_results.append(("Basic Sensor Fusion", test_sensor_fusion_basic()))
    test_results.append(("Mini Integration Demo", run_mini_demo()))
    
    # Print results
    print("\n" + "=" * 50)
    print("📊 Test Results Summary:")
    print("=" * 50)
    
    passed = 0
    for test_name, result in test_results:
        status = "✅ PASSED" if result else "❌ FAILED"
        print(f"  {test_name:20s}: {status}")
        if result:
            passed += 1
    
    print(f"\n🎯 Results: {passed}/{len(test_results)} tests passed")
    
    if passed == len(test_results):
        print("🎉 Phase 6 basic implementation is working!")
        print("\n🚀 Basic Phase 6 Components Working:")
        print("  ✅ Advanced sensor suite (IMU, encoders, force, distance, terrain)")
        print("  ✅ Extended Kalman Filter for state estimation")
        print("  ✅ Basic sensor fusion manager")
        print("  ✅ Sensor data integration")
        print("  ✅ Real-time sensor reading")
        
        print("\n🔧 Next Steps for Full Phase 6:")
        print("  🔲 Add PID and adaptive controllers")
        print("  🔲 Add environmental sensing and mapping")
        print("  🔲 Add safety monitoring systems")
        print("  🔲 Integrate with GUI (Phase 5)")
    else:
        print("⚠️  Some basic tests failed. Fixing core issues first...")
        return 1
    
    return 0

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)