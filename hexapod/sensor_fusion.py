"""
Phase 6: Sensor Fusion & State Estimation
HexaPodSim 2.0 - Advanced Sensor Integration

This module implements sensor fusion algorithms for robust state estimation using
multiple sensor inputs. Includes Kalman filtering, complementary filters, and
advanced estimation techniques for hexapod robot navigation.

Features:
- Extended Kalman Filter (EKF) for state estimation
- Complementary filters for IMU/position fusion
- Multi-sensor data fusion algorithms
- Outlier detection and rejection
- Adaptive filtering based on sensor quality
- Real-time state estimation and prediction

Key Algorithms:
- EKF for position, velocity, and orientation estimation
- Complementary filter for IMU attitude estimation
- Weighted sensor fusion based on quality metrics
- Dead reckoning with sensor correction
- Bias estimation and compensation

Author: HexaPodSim Team
Date: October 2024
Phase: 6 - Sensor Integration & Feedback
"""

import numpy as np
import time
import logging
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any
from collections import deque
import threading
from abc import ABC, abstractmethod

try:
    from .sensors import SensorReading, SensorType, IMUSensor, PositionSensor
    from .sensors import euler_to_quaternion, quaternion_to_euler
except ImportError:
    # For standalone testing
    from sensors import SensorReading, SensorType, IMUSensor, PositionSensor
    from sensors import euler_to_quaternion, quaternion_to_euler

# Configure logging
logger = logging.getLogger(__name__)


@dataclass
class RobotState:
    """Complete robot state estimate"""
    timestamp: float
    
    # Position and orientation
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))  # m
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # m/s
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))  # m/s²
    
    # Orientation (quaternion: w,x,y,z)
    orientation: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # rad/s
    angular_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))  # rad/s²
    
    # Uncertainty estimates (diagonal of covariance matrix)
    position_uncertainty: np.ndarray = field(default_factory=lambda: np.ones(3) * 0.1)
    orientation_uncertainty: np.ndarray = field(default_factory=lambda: np.ones(3) * 0.01)
    
    # Confidence and quality
    confidence: float = 1.0
    quality: float = 1.0
    
    def get_euler_angles(self) -> Tuple[float, float, float]:
        """Get Euler angles from quaternion (degrees)"""
        roll, pitch, yaw = quaternion_to_euler(self.orientation)
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)
    
    def set_euler_angles(self, roll_deg: float, pitch_deg: float, yaw_deg: float):
        """Set orientation from Euler angles (degrees)"""
        roll, pitch, yaw = np.radians([roll_deg, pitch_deg, yaw_deg])
        self.orientation = euler_to_quaternion(roll, pitch, yaw)


class ExtendedKalmanFilter:
    """Extended Kalman Filter for robot state estimation"""
    
    def __init__(self, state_dim: int = 13, measurement_dim: int = 6):
        """
        Initialize EKF
        State vector: [px, py, pz, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
        - Position (3), Velocity (3), Quaternion (4), Angular velocity (3)
        """
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim
        
        # State vector
        self.x = np.zeros(state_dim)
        self.x[6] = 1.0  # Initialize quaternion w component
        
        # Covariance matrix
        self.P = np.eye(state_dim) * 0.1
        
        # Process noise covariance
        self.Q = np.eye(state_dim) * 0.01
        self.Q[3:6, 3:6] *= 0.1  # Higher velocity noise
        self.Q[10:13, 10:13] *= 0.05  # Angular velocity noise
        
        # Measurement noise covariance (will be updated per sensor)
        self.R = np.eye(measurement_dim) * 0.1
        
        # Process model Jacobian
        self.F = np.eye(state_dim)
        
        # Measurement model Jacobian
        self.H = np.zeros((measurement_dim, state_dim))
        
        self.last_update = time.time()
        self.initialized = False
        
        logger.info("Extended Kalman Filter initialized")
    
    def predict(self, dt: float, control_input: Optional[np.ndarray] = None):
        """Prediction step of EKF"""
        if dt <= 0:
            return
        
        # State transition model
        F = self.get_state_transition_matrix(dt)
        
        # Predict state
        self.x = self.predict_state(self.x, dt, control_input)
        
        # Normalize quaternion
        quat = self.x[6:10]
        quat_norm = np.linalg.norm(quat)
        if quat_norm > 0:
            self.x[6:10] = quat / quat_norm
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
    
    def update(self, measurement: np.ndarray, measurement_type: str, 
               measurement_noise: Optional[np.ndarray] = None):
        """Update step of EKF"""
        # Get measurement model
        H, expected_measurement = self.get_measurement_model(measurement_type)
        
        if H is None:
            return
        
        # Update measurement noise if provided
        if measurement_noise is not None:
            R = np.diag(measurement_noise)
        else:
            R = self.R[:H.shape[0], :H.shape[0]]
        
        # Innovation
        innovation = measurement - expected_measurement
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            logger.warning("Singular innovation covariance, skipping update")
            return
        
        # Update state
        self.x = self.x + K @ innovation
        
        # Normalize quaternion
        quat = self.x[6:10]
        quat_norm = np.linalg.norm(quat)
        if quat_norm > 0:
            self.x[6:10] = quat / quat_norm
        
        # Update covariance
        I_KH = np.eye(self.state_dim) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
    
    def get_state(self) -> np.ndarray:
        """Get current state estimate"""
        return self.x.copy()
    
    def get_covariance(self) -> np.ndarray:
        """Get current covariance matrix"""
        return self.P.copy()
    
    def get_position(self) -> np.ndarray:
        """Get current position estimate"""
        return self.x[0:3].copy()
    
    def get_velocity(self) -> np.ndarray:
        """Get current velocity estimate"""
        return self.x[3:6].copy()
    
    def get_orientation_quaternion(self) -> np.ndarray:
        """Get current orientation as quaternion"""
        return self.x[6:10].copy()
    
    def get_angular_velocity(self) -> np.ndarray:
        """Get current angular velocity"""
        if len(self.x) >= 13:
            return self.x[10:13].copy()
        return np.zeros(3)
    
    def get_state_transition_matrix(self, dt: float) -> np.ndarray:
        """Get state transition matrix F"""
        F = np.eye(self.state_dim)
        
        # Position integration
        F[0:3, 3:6] = np.eye(3) * dt
        
        # Quaternion propagation (simplified)
        # In practice, this would be more complex for quaternion kinematics
        
        return F
    
    def predict_state(self, x: np.ndarray, dt: float, 
                     control_input: Optional[np.ndarray] = None) -> np.ndarray:
        """Predict next state"""
        x_new = x.copy()
        
        # Position integration: p = p + v*dt
        x_new[0:3] += x[3:6] * dt
        
        # Quaternion integration (simplified)
        # q_new = q + 0.5 * q * omega * dt
        quat = x[6:10]
        
        # Ensure we have angular velocity components
        if len(x) >= 13:
            omega = x[10:13]
        else:
            omega = np.zeros(3)  # Default to no rotation
        
        # Quaternion derivative
        omega_quat = np.array([0.0, omega[0], omega[1], omega[2]])
        quat_dot = 0.5 * self.quaternion_multiply(quat, omega_quat)
        x_new[6:10] += quat_dot * dt
        
        return x_new
    
    def quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return np.array([w, x, y, z])
    
    def get_measurement_model(self, measurement_type: str) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get measurement model H and expected measurement"""
        if measurement_type == "position":
            # Direct position measurement
            H = np.zeros((3, self.state_dim))
            H[0:3, 0:3] = np.eye(3)
            expected = self.x[0:3]
            return H, expected
            
        elif measurement_type == "imu_accel":
            # IMU accelerometer measurement
            H = np.zeros((3, self.state_dim))
            # This would be more complex in practice, involving gravity compensation
            expected = np.array([0.0, 0.0, 9.81])  # Simplified
            return H, expected
            
        elif measurement_type == "imu_gyro":
            # IMU gyroscope measurement
            H = np.zeros((3, self.state_dim))
            if self.state_dim >= 13:
                H[0:3, 10:13] = np.eye(3)
                expected = self.x[10:13]
            else:
                expected = np.zeros(3)
            return H, expected
        
        else:
            logger.warning(f"Unknown measurement type: {measurement_type}")
            return None, None
    
    def get_state_estimate(self) -> RobotState:
        """Get current state estimate"""
        state = RobotState(timestamp=time.time())
        
        state.position = self.x[0:3].copy()
        state.velocity = self.x[3:6].copy()
        state.orientation = self.x[6:10].copy()
        
        # Handle angular velocity safely
        if len(self.x) >= 13:
            state.angular_velocity = self.x[10:13].copy()
        else:
            state.angular_velocity = np.zeros(3)
        
        # Extract uncertainties from diagonal of covariance matrix
        state.position_uncertainty = np.sqrt(np.diag(self.P[0:3, 0:3]))
        state.orientation_uncertainty = np.sqrt(np.diag(self.P[6:9, 6:9]))
        
        # Calculate confidence based on trace of covariance
        trace_P = np.trace(self.P)
        state.confidence = 1.0 / (1.0 + trace_P)
        
        return state


class ComplementaryFilter:
    """Complementary filter for IMU attitude estimation"""
    
    def __init__(self, alpha: float = 0.98):
        """
        Initialize complementary filter
        alpha: weight for gyroscope (0-1, closer to 1 trusts gyro more)
        """
        self.alpha = alpha
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # w,x,y,z quaternion
        self.last_update = time.time()
        self.initialized = False
        
        logger.info(f"Complementary filter initialized with alpha={alpha}")
    
    def update(self, gyro_rad_s: np.ndarray, accel_m_s2: np.ndarray, 
               mag_uT: Optional[np.ndarray] = None) -> np.ndarray:
        """Update orientation estimate"""
        current_time = time.time()
        dt = current_time - self.last_update
        
        if dt <= 0 or dt > 1.0:  # Skip if time step is invalid
            self.last_update = current_time
            return self.orientation
        
        if not self.initialized:
            # Initialize with accelerometer
            self.orientation = self.accel_to_quaternion(accel_m_s2)
            self.initialized = True
            self.last_update = current_time
            return self.orientation
        
        # Gyroscope integration
        gyro_quat = self.integrate_gyro(gyro_rad_s, dt)
        
        # Accelerometer attitude
        accel_quat = self.accel_to_quaternion(accel_m_s2)
        
        # Complementary filter
        self.orientation = self.slerp_quaternion(accel_quat, gyro_quat, self.alpha)
        
        # Normalize
        self.orientation = self.normalize_quaternion(self.orientation)
        
        self.last_update = current_time
        return self.orientation
    
    def integrate_gyro(self, gyro_rad_s: np.ndarray, dt: float) -> np.ndarray:
        """Integrate gyroscope to update orientation"""
        # Angular velocity quaternion
        omega_quat = np.array([0.0, gyro_rad_s[0], gyro_rad_s[1], gyro_rad_s[2]])
        
        # Quaternion integration: q_new = q + 0.5 * q * omega * dt
        quat_dot = 0.5 * self.quaternion_multiply(self.orientation, omega_quat)
        new_orientation = self.orientation + quat_dot * dt
        
        return self.normalize_quaternion(new_orientation)
    
    def accel_to_quaternion(self, accel_m_s2: np.ndarray) -> np.ndarray:
        """Convert accelerometer reading to orientation quaternion"""
        # Normalize accelerometer
        accel_norm = np.linalg.norm(accel_m_s2)
        if accel_norm < 0.1:  # Avoid division by zero
            return self.orientation
        
        accel_unit = accel_m_s2 / accel_norm
        
        # Calculate roll and pitch
        roll = np.arctan2(accel_unit[1], accel_unit[2])
        pitch = np.arctan2(-accel_unit[0], np.sqrt(accel_unit[1]**2 + accel_unit[2]**2))
        yaw = 0.0  # Cannot determine yaw from accelerometer alone
        
        return euler_to_quaternion(roll, pitch, yaw)
    
    def quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return np.array([w, x, y, z])
    
    def normalize_quaternion(self, q: np.ndarray) -> np.ndarray:
        """Normalize quaternion"""
        norm = np.linalg.norm(q)
        if norm > 0:
            return q / norm
        return np.array([1.0, 0.0, 0.0, 0.0])
    
    def slerp_quaternion(self, q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """Spherical linear interpolation between quaternions"""
        # Ensure shortest path
        dot = np.dot(q1, q2)
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        # If quaternions are very close, use linear interpolation
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return self.normalize_quaternion(result)
        
        # Calculate angle between quaternions
        theta_0 = np.arccos(np.abs(dot))
        sin_theta_0 = np.sin(theta_0)
        
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        
        return s0 * q1 + s1 * q2


class SensorFusionSystem:
    """Main sensor fusion system coordinating multiple sensors and filters"""
    
    def __init__(self):
        self.ekf = ExtendedKalmanFilter()
        self.comp_filter = ComplementaryFilter()
        
        # Sensor management
        self.sensors = {}
        self.sensor_readings = deque(maxlen=1000)
        
        # State estimation
        self.current_state = RobotState(timestamp=time.time())
        self.state_history = deque(maxlen=100)
        
        # Threading
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        
        # Quality thresholds
        self.min_sensor_quality = 0.3
        self.outlier_threshold = 3.0  # Standard deviations
        
        logger.info("Sensor fusion system initialized")
    
    def add_sensor(self, sensor):
        """Add sensor to fusion system"""
        self.sensors[sensor.sensor_id] = sensor
        logger.info(f"Added sensor {sensor.sensor_id} to fusion system")
    
    def remove_sensor(self, sensor_id: str):
        """Remove sensor from fusion system"""
        if sensor_id in self.sensors:
            del self.sensors[sensor_id]
            logger.info(f"Removed sensor {sensor_id} from fusion system")
    
    def start_fusion(self):
        """Start continuous sensor fusion"""
        if self._running:
            return
        
        self._running = True
        self._thread = threading.Thread(target=self._fusion_loop, daemon=True)
        self._thread.start()
        logger.info("Sensor fusion system started")
    
    def stop_fusion(self):
        """Stop sensor fusion"""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        logger.info("Sensor fusion system stopped")
    
    def _fusion_loop(self):
        """Main fusion loop"""
        while self._running:
            self.update_fusion()
            time.sleep(0.01)  # 100Hz fusion rate
    
    def update_fusion(self):
        """Update sensor fusion with latest readings"""
        current_time = time.time()
        
        # Collect recent sensor readings
        recent_readings = []
        for sensor in self.sensors.values():
            reading = sensor.read()
            if reading and reading.valid and reading.quality >= self.min_sensor_quality:
                recent_readings.append(reading)
                self.sensor_readings.append(reading)
        
        if not recent_readings:
            return
        
        with self._lock:
            # Predict step
            dt = current_time - self.current_state.timestamp
            if dt > 0:
                self.ekf.predict(dt)
            
            # Process each sensor reading
            for reading in recent_readings:
                self.process_sensor_reading(reading)
            
            # Update current state estimate
            self.current_state = self.ekf.get_state_estimate()
            self.current_state.timestamp = current_time
            
            # Store in history
            self.state_history.append(self.current_state)
    
    def process_sensor_reading(self, reading: SensorReading):
        """Process individual sensor reading"""
        try:
            if reading.sensor_type == SensorType.IMU:
                self.process_imu_reading(reading)
            elif reading.sensor_type == SensorType.POSITION:
                self.process_position_reading(reading)
            elif reading.sensor_type == SensorType.GYROSCOPE:
                self.process_gyro_reading(reading)
            elif reading.sensor_type == SensorType.ACCELEROMETER:
                self.process_accel_reading(reading)
            
        except Exception as e:
            logger.error(f"Error processing {reading.sensor_type} reading: {e}")
    
    def process_imu_reading(self, reading: SensorReading):
        """Process IMU sensor reading"""
        data = reading.data
        if len(data) >= 9:  # 9-DOF IMU
            gyro_deg_s = data[0:3]
            accel_g = data[3:6]
            mag_uT = data[6:9]
            
            # Convert to standard units
            gyro_rad_s = np.radians(gyro_deg_s)
            accel_m_s2 = accel_g * 9.81
            
            # Update complementary filter
            orientation = self.comp_filter.update(gyro_rad_s, accel_m_s2, mag_uT)
            
            # Update EKF with gyro measurement
            measurement_noise = np.ones(3) * (0.1 / reading.quality)
            self.ekf.update(gyro_rad_s, "imu_gyro", measurement_noise)
    
    def process_position_reading(self, reading: SensorReading):
        """Process position sensor reading"""
        position = reading.data
        measurement_noise = np.ones(3) * (0.05 / reading.quality)
        self.ekf.update(position, "position", measurement_noise)
    
    def process_gyro_reading(self, reading: SensorReading):
        """Process standalone gyroscope reading"""
        gyro_deg_s = reading.data
        gyro_rad_s = np.radians(gyro_deg_s)
        measurement_noise = np.ones(3) * (0.1 / reading.quality)
        self.ekf.update(gyro_rad_s, "imu_gyro", measurement_noise)
    
    def process_accel_reading(self, reading: SensorReading):
        """Process standalone accelerometer reading"""
        accel_g = reading.data
        accel_m_s2 = accel_g * 9.81
        measurement_noise = np.ones(3) * (0.2 / reading.quality)
        self.ekf.update(accel_m_s2, "imu_accel", measurement_noise)
    
    def get_current_state(self) -> RobotState:
        """Get current state estimate"""
        with self._lock:
            return self.current_state
    
    def get_state_history(self, count: int = 10) -> List[RobotState]:
        """Get recent state history"""
        with self._lock:
            return list(self.state_history)[-count:]
    
    def is_outlier(self, measurement: np.ndarray, expected: np.ndarray, 
                   uncertainty: np.ndarray) -> bool:
        """Check if measurement is an outlier"""
        residual = measurement - expected
        normalized_residual = residual / (uncertainty + 1e-6)
        
        return np.any(np.abs(normalized_residual) > self.outlier_threshold)
    
    def get_fusion_statistics(self) -> Dict[str, Any]:
        """Get fusion system statistics"""
        stats = {
            'num_sensors': len(self.sensors),
            'total_readings': len(self.sensor_readings),
            'current_confidence': self.current_state.confidence,
            'current_quality': self.current_state.quality,
            'position_uncertainty': np.mean(self.current_state.position_uncertainty),
            'orientation_uncertainty': np.mean(self.current_state.orientation_uncertainty)
        }
        
        # Per-sensor statistics
        sensor_stats = {}
        for sensor_id, sensor in self.sensors.items():
            recent_readings = sensor.get_recent_readings(10)
            if recent_readings:
                avg_quality = np.mean([r.quality for r in recent_readings])
                valid_count = sum(1 for r in recent_readings if r.valid)
                sensor_stats[sensor_id] = {
                    'avg_quality': avg_quality,
                    'valid_rate': valid_count / len(recent_readings),
                    'last_reading': recent_readings[-1].timestamp
                }
        
        stats['sensors'] = sensor_stats
        return stats


if __name__ == "__main__":
    # Test sensor fusion system
    print("🤖 Phase 6: Testing Sensor Fusion System")
    print("=" * 45)
    
    # Create fusion system
    fusion = SensorFusionSystem()
    
    # Create and add sensors
    imu = IMUSensor("main_imu", update_rate=100.0)
    position_sensor = PositionSensor("main_position", update_rate=50.0)
    
    fusion.add_sensor(imu)
    fusion.add_sensor(position_sensor)
    
    print("✅ Sensors added to fusion system")
    
    # Simulate some motion
    print("\n📊 Simulating robot motion...")
    
    for i in range(10):
        # Simulate IMU data
        t = i * 0.1
        angular_vel = np.array([0.1 * np.sin(t), 0.0, 0.05 * np.cos(t)])
        linear_accel = np.array([0.5 * np.cos(t), 0.0, 9.81])
        
        imu.set_motion_state(angular_vel, linear_accel)
        
        # Simulate position data
        position = np.array([t * 0.1, t * 0.05, 0.15])
        position_sensor.set_position(position)
        
        # Update fusion
        fusion.update_fusion()
        
        # Get state estimate
        state = fusion.get_current_state()
        
        print(f"Time {t:.1f}s: Pos={state.position}, "
              f"Orientation={state.get_euler_angles()}, "
              f"Confidence={state.confidence:.3f}")
        
        time.sleep(0.1)
    
    # Display statistics
    stats = fusion.get_fusion_statistics()
    print(f"\n📈 Fusion Statistics:")
    print(f"  Sensors: {stats['num_sensors']}")
    print(f"  Total readings: {stats['total_readings']}")
    print(f"  Confidence: {stats['current_confidence']:.3f}")
    print(f"  Position uncertainty: {stats['position_uncertainty']:.3f}m")
    
    print("\n✅ Sensor fusion test completed!")


class SensorFusionManager:
    """High-level manager for sensor fusion operations"""
    
    def __init__(self):
        self.sensors: Dict[str, Any] = {}
        self.sensor_weights: Dict[str, float] = {}
        self.ekf = ExtendedKalmanFilter(state_dim=13, measurement_dim=6)
        self.last_update = time.time()
        self.update_rate = 50.0  # Hz
        
        logger.info("Sensor Fusion Manager initialized")
    
    def add_sensor(self, name: str, sensor: Any, weight: float = 1.0):
        """Add a sensor to the fusion system"""
        self.sensors[name] = sensor
        self.sensor_weights[name] = weight
        logger.info(f"Added sensor '{name}' with weight {weight}")
    
    def remove_sensor(self, name: str):
        """Remove a sensor from the fusion system"""
        if name in self.sensors:
            del self.sensors[name]
            del self.sensor_weights[name]
            logger.info(f"Removed sensor '{name}'")
    
    def fuse_sensor_data(self, dt: float) -> np.ndarray:
        """Fuse data from all sensors and return state estimate"""
        current_time = time.time()
        
        # Prediction step
        self.ekf.predict(dt)
        
        # Update with sensor measurements
        for name, sensor in self.sensors.items():
            try:
                # Read sensor data
                if hasattr(sensor, 'read'):
                    reading = sensor.read()
                    if reading and hasattr(reading, 'data'):
                        data = reading.data
                        
                        # Determine measurement type based on sensor
                        if 'imu' in name.lower():
                            if isinstance(data, (list, np.ndarray)) and len(data) >= 6:
                                # IMU data: gyro + accel
                                gyro_data = np.array(data[3:6])  # Angular velocity
                                accel_data = np.array(data[0:3])  # Linear acceleration
                                
                                # Update with gyroscope data
                                self.ekf.update(gyro_data, "imu_gyro")
                                
                                # Update with accelerometer data  
                                self.ekf.update(accel_data, "imu_accel")
                        
                        elif 'position' in name.lower() or 'encoder' in name.lower():
                            # Position or encoder data
                            if isinstance(data, (list, np.ndarray)) and len(data) >= 3:
                                pos_data = np.array(data[:3])
                                self.ekf.update(pos_data, "position")
                            elif isinstance(data, (int, float)):
                                # Single encoder value - skip for now
                                pass
                                
            except Exception as e:
                logger.warning(f"Error processing sensor '{name}': {e}")
                continue
        
        self.last_update = current_time
        return self.ekf.get_state()
    
    def get_state_estimate(self) -> RobotState:
        """Get the current robot state estimate"""
        state_vector = self.ekf.get_state()
        
        robot_state = RobotState(timestamp=time.time())
        robot_state.position = state_vector[0:3]
        robot_state.velocity = state_vector[3:6]
        robot_state.orientation = state_vector[6:10]
        
        if len(state_vector) >= 13:
            robot_state.angular_velocity = state_vector[10:13]
        
        # Calculate confidence based on covariance
        position_cov = np.diag(self.ekf.P[0:3, 0:3])
        robot_state.confidence = 1.0 / (1.0 + np.mean(position_cov))
        robot_state.quality = robot_state.confidence
        
        return robot_state
    
    def get_fusion_statistics(self) -> Dict[str, Any]:
        """Get fusion system statistics"""
        return {
            'num_sensors': len(self.sensors),
            'sensor_names': list(self.sensors.keys()),
            'sensor_weights': self.sensor_weights.copy(),
            'last_update': self.last_update,
            'update_rate': self.update_rate,
            'current_confidence': self.get_state_estimate().confidence
        }
    
    def reset_fusion(self):
        """Reset the fusion system"""
        self.ekf = ExtendedKalmanFilter(state_dim=13, measurement_dim=6)
        self.last_update = time.time()
        logger.info("Sensor fusion system reset")