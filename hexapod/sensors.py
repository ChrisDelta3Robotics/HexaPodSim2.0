"""
Phase 6: Advanced Sensor Integration & Feedback Systems
HexaPodSim 2.0 - Comprehensive Sensor Suite

This module implements a complete sensor suite for the hexapod robot including:
- Inertial Measurement Unit (IMU) with gyroscope and accelerometer
- Position and orientation sensors
- Environmental sensors (LiDAR, ultrasonic, camera)
- Sensor fusion algorithms (Kalman filtering)
- Real-time feedback systems
- Noise modeling and filtering

Features:
- 9-DOF IMU simulation with realistic noise characteristics
- Multi-sensor fusion for robust state estimation
- Environmental mapping and obstacle detection
- Closed-loop feedback control integration
- Sensor calibration and bias compensation
- Real-time data processing and filtering

Author: HexaPodSim Team
Date: October 2024
Phase: 6 - Sensor Integration & Feedback
"""

import numpy as np
import time
import logging
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any, Callable
from enum import Enum
from abc import ABC, abstractmethod
import threading
from collections import deque

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Physical constants
GRAVITY = 9.81  # m/s²
EARTH_ROTATION_RATE = 7.2921e-5  # rad/s


class SensorType(Enum):
    """Sensor type enumeration"""
    IMU = "imu"
    GYROSCOPE = "gyroscope"
    ACCELEROMETER = "accelerometer"
    MAGNETOMETER = "magnetometer"
    POSITION = "position"
    ORIENTATION = "orientation"
    LIDAR = "lidar"
    ULTRASONIC = "ultrasonic"
    CAMERA = "camera"
    FORCE = "force"
    PRESSURE = "pressure"


@dataclass
class SensorNoise:
    """Sensor noise characteristics"""
    # White noise parameters
    white_noise_std: float = 0.01
    # Random walk parameters
    random_walk_std: float = 0.001
    # Bias stability
    bias_stability: float = 0.005
    # Scale factor error
    scale_factor_error: float = 0.001
    # Temperature sensitivity
    temp_sensitivity: float = 0.0001


@dataclass
class SensorReading:
    """Generic sensor reading with metadata"""
    timestamp: float
    sensor_type: SensorType
    sensor_id: str
    data: np.ndarray
    quality: float = 1.0  # Data quality indicator (0-1)
    valid: bool = True
    metadata: Dict[str, Any] = field(default_factory=dict)


class BaseSensor(ABC):
    """Abstract base class for all sensors"""
    
    def __init__(self, sensor_id: str, sensor_type: SensorType, 
                 update_rate: float = 100.0, noise: Optional[SensorNoise] = None):
        self.sensor_id = sensor_id
        self.sensor_type = sensor_type
        self.update_rate = update_rate  # Hz
        self.noise = noise or SensorNoise()
        
        # Internal state - initialize with proper size
        self.last_update = 0.0
        self.bias = None  # Will be initialized on first read
        self.scale_factors = None  # Will be initialized on first read
        self.temperature = 20.0  # °C
        self.calibrated = False
        
        # Data buffers
        self.readings_buffer = deque(maxlen=1000)
        self.raw_buffer = deque(maxlen=100)
        
        # Threading
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        
        logger.info(f"Initialized {sensor_type.value} sensor: {sensor_id}")
    
    @abstractmethod
    def read_raw(self) -> np.ndarray:
        """Read raw sensor data without noise or processing"""
        pass
    
    def add_noise(self, data: np.ndarray) -> np.ndarray:
        """Add realistic noise to sensor data"""
        data_shape = data.shape
        
        # Initialize bias and scale factors on first use
        if self.bias is None:
            self.bias = np.zeros(data_shape)
        if self.scale_factors is None:
            self.scale_factors = np.ones(data_shape)
        
        # White noise
        noise = np.random.normal(0, self.noise.white_noise_std, data_shape)
        
        # Random walk (integrated white noise)
        random_walk = np.random.normal(0, self.noise.random_walk_std, data_shape)
        
        # Bias drift (ensure bias has same shape as data)
        if self.bias.shape != data_shape:
            self.bias = np.resize(self.bias, data_shape)
        
        bias_drift = np.random.normal(0, self.noise.bias_stability, data_shape)
        self.bias += bias_drift * 0.01  # Slow bias evolution
        
        # Scale factor error
        if self.scale_factors.shape != data_shape:
            self.scale_factors = np.resize(self.scale_factors, data_shape)
        
        scale_error = self.noise.scale_factor_error * np.random.normal(0, 1, data_shape)
        
        # Temperature effect
        temp_effect = self.noise.temp_sensitivity * (self.temperature - 20.0)
        
        # Combine all noise sources
        noisy_data = data * (1 + scale_error) + noise + random_walk + self.bias + temp_effect
        
        return noisy_data
    
    def read(self) -> Optional[SensorReading]:
        """Read processed sensor data"""
        current_time = time.time()
        dt = current_time - self.last_update
        
        # Check update rate
        if dt < (1.0 / self.update_rate):
            return None
        
        with self._lock:
            try:
                # Get raw data
                raw_data = self.read_raw()
                self.raw_buffer.append((current_time, raw_data.copy()))
                
                # Add noise
                noisy_data = self.add_noise(raw_data)
                
                # Apply calibration
                if self.calibrated:
                    processed_data = self.apply_calibration(noisy_data)
                else:
                    processed_data = noisy_data
                
                # Create reading
                reading = SensorReading(
                    timestamp=current_time,
                    sensor_type=self.sensor_type,
                    sensor_id=self.sensor_id,
                    data=processed_data,
                    quality=self.calculate_quality(),
                    valid=self.validate_reading(processed_data)
                )
                
                self.readings_buffer.append(reading)
                self.last_update = current_time
                
                return reading
                
            except Exception as e:
                logger.error(f"Sensor {self.sensor_id} read error: {e}")
                return None
    
    def apply_calibration(self, data: np.ndarray) -> np.ndarray:
        """Apply sensor calibration"""
        # Remove known bias and apply scale factor correction
        return (data - self.bias) / self.scale_factors
    
    def calculate_quality(self) -> float:
        """Calculate data quality metric"""
        # Simple quality based on recent reading consistency
        if len(self.readings_buffer) < 2:
            return 1.0
        
        recent_readings = list(self.readings_buffer)[-10:]
        if len(recent_readings) < 2:
            return 1.0
        
        # Calculate variance of recent readings
        data_array = np.array([r.data for r in recent_readings])
        variance = np.mean(np.var(data_array, axis=0))
        
        # Convert variance to quality (inverse relationship)
        quality = 1.0 / (1.0 + variance * 100)
        return np.clip(quality, 0.1, 1.0)
    
    def validate_reading(self, data: np.ndarray) -> bool:
        """Validate sensor reading"""
        # Check for NaN or infinite values
        if not np.isfinite(data).all():
            return False
        
        # Check for reasonable magnitude
        if np.any(np.abs(data) > 1000):  # Adjust threshold per sensor type
            return False
        
        return True
    
    def calibrate(self, reference_data: Optional[np.ndarray] = None):
        """Calibrate the sensor"""
        logger.info(f"Calibrating sensor {self.sensor_id}")
        
        if reference_data is not None:
            # Use provided reference for calibration
            self.bias = np.mean(reference_data, axis=0)
        else:
            # Auto-calibration using recent readings
            if len(self.raw_buffer) >= 50:
                raw_data = np.array([data for _, data in list(self.raw_buffer)])
                self.bias = np.mean(raw_data, axis=0)
        
        self.calibrated = True
        logger.info(f"Sensor {self.sensor_id} calibrated with bias: {self.bias}")
    
    def start_continuous_reading(self):
        """Start continuous sensor reading in background thread"""
        if self._running:
            return
        
        self._running = True
        self._thread = threading.Thread(target=self._continuous_read_loop, daemon=True)
        self._thread.start()
        logger.info(f"Started continuous reading for sensor {self.sensor_id}")
    
    def stop_continuous_reading(self):
        """Stop continuous sensor reading"""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        logger.info(f"Stopped continuous reading for sensor {self.sensor_id}")
    
    def _continuous_read_loop(self):
        """Continuous reading loop"""
        while self._running:
            self.read()
            time.sleep(1.0 / self.update_rate)
    
    def get_recent_readings(self, count: int = 10) -> List[SensorReading]:
        """Get recent sensor readings"""
        with self._lock:
            return list(self.readings_buffer)[-count:]


class IMUSensor(BaseSensor):
    """9-DOF Inertial Measurement Unit sensor"""
    
    def __init__(self, sensor_id: str = "imu_main", update_rate: float = 200.0):
        # IMU-specific noise characteristics
        imu_noise = SensorNoise(
            white_noise_std=0.02,  # Typical IMU noise
            random_walk_std=0.001,
            bias_stability=0.01,
            scale_factor_error=0.005,
            temp_sensitivity=0.0002
        )
        
        super().__init__(sensor_id, SensorType.IMU, update_rate, imu_noise)
        
        # IMU-specific properties
        self.gyro_range = 2000.0  # deg/s
        self.accel_range = 16.0   # g
        self.mag_range = 4900.0   # µT
        
        # Current state (to be set by robot system)
        self.angular_velocity = np.zeros(3)  # rad/s
        self.linear_acceleration = np.zeros(3)  # m/s²
        self.magnetic_field = np.array([22.0, 5.0, -40.0])  # µT (typical Earth field)
        
        # Internal state
        self.orientation_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # w,x,y,z
        
        logger.info(f"IMU sensor initialized with ranges: ±{self.gyro_range}°/s, ±{self.accel_range}g")
    
    def read_raw(self) -> np.ndarray:
        """Read raw IMU data (gyro, accel, mag)"""
        # 9-DOF data: [gx, gy, gz, ax, ay, az, mx, my, mz]
        raw_data = np.concatenate([
            np.degrees(self.angular_velocity),  # Convert to degrees/s
            self.linear_acceleration / GRAVITY,  # Convert to g units
            self.magnetic_field  # µT
        ])
        
        return raw_data
    
    def set_motion_state(self, angular_velocity: np.ndarray, linear_acceleration: np.ndarray,
                        orientation_quaternion: Optional[np.ndarray] = None):
        """Set the current motion state for IMU simulation"""
        self.angular_velocity = angular_velocity.copy()
        self.linear_acceleration = linear_acceleration.copy()
        
        if orientation_quaternion is not None:
            self.orientation_quaternion = orientation_quaternion.copy()
    
    def get_orientation_estimate(self) -> Tuple[np.ndarray, float]:
        """Get orientation estimate from IMU using complementary filter"""
        if len(self.readings_buffer) < 2:
            return self.orientation_quaternion, 0.0
        
        # Simple complementary filter implementation
        # (In practice, would use more sophisticated algorithms like Madgwick or Mahony)
        
        recent_readings = list(self.readings_buffer)[-10:]
        gyro_data = np.array([r.data[:3] for r in recent_readings])  # deg/s
        accel_data = np.array([r.data[3:6] for r in recent_readings])  # g
        
        # Convert gyro to rad/s
        gyro_rad = np.radians(np.mean(gyro_data, axis=0))
        accel_g = np.mean(accel_data, axis=0)
        
        # Estimate roll and pitch from accelerometer
        roll = np.arctan2(accel_g[1], accel_g[2])
        pitch = np.arctan2(-accel_g[0], np.sqrt(accel_g[1]**2 + accel_g[2]**2))
        
        # For yaw, would need magnetometer compensation (simplified here)
        yaw = 0.0
        
        # Convert to quaternion
        orientation_quat = euler_to_quaternion(roll, pitch, yaw)
        
        # Calculate confidence based on acceleration magnitude
        accel_magnitude = np.linalg.norm(accel_g)
        confidence = 1.0 / (1.0 + abs(accel_magnitude - 1.0))  # Confidence decreases if not near 1g
        
        return orientation_quat, confidence


class PositionSensor(BaseSensor):
    """Position sensor (e.g., GPS, motion capture, visual odometry)"""
    
    def __init__(self, sensor_id: str = "position_main", update_rate: float = 50.0):
        position_noise = SensorNoise(
            white_noise_std=0.05,  # 5cm position noise
            random_walk_std=0.001,
            bias_stability=0.02,
            scale_factor_error=0.001
        )
        
        super().__init__(sensor_id, SensorType.POSITION, update_rate, position_noise)
        
        # Current position (to be set by robot system)
        self.position = np.zeros(3)  # m
        self.velocity = np.zeros(3)  # m/s
        
        logger.info("Position sensor initialized")
    
    def read_raw(self) -> np.ndarray:
        """Read raw position data"""
        return self.position.copy()
    
    def set_position(self, position: np.ndarray, velocity: Optional[np.ndarray] = None):
        """Set current position for sensor simulation"""
        self.position = position.copy()
        if velocity is not None:
            self.velocity = velocity.copy()


class LiDARSensor(BaseSensor):
    """LiDAR sensor for environment scanning"""
    
    def __init__(self, sensor_id: str = "lidar_main", update_rate: float = 10.0,
                 max_range: float = 5.0, angular_resolution: float = 1.0):
        lidar_noise = SensorNoise(
            white_noise_std=0.02,  # 2cm range noise
            random_walk_std=0.0005,
            bias_stability=0.01
        )
        
        super().__init__(sensor_id, SensorType.LIDAR, update_rate, lidar_noise)
        
        self.max_range = max_range
        self.angular_resolution = angular_resolution
        self.num_beams = int(360.0 / angular_resolution)
        
        # Environment map (simple obstacle list for now)
        self.obstacles = []  # List of (x, y, radius) tuples
        self.robot_position = np.zeros(2)
        self.robot_orientation = 0.0  # radians
        
        logger.info(f"LiDAR initialized: {self.num_beams} beams, {max_range}m range")
    
    def read_raw(self) -> np.ndarray:
        """Read raw LiDAR scan data"""
        ranges = np.full(self.num_beams, self.max_range)
        
        for i in range(self.num_beams):
            angle = np.radians(i * self.angular_resolution) + self.robot_orientation
            
            # Cast ray and find nearest obstacle
            ray_range = self.cast_ray(angle)
            ranges[i] = min(ray_range, self.max_range)
        
        return ranges
    
    def cast_ray(self, angle: float) -> float:
        """Cast a ray and return distance to nearest obstacle"""
        ray_dir = np.array([np.cos(angle), np.sin(angle)])
        min_distance = self.max_range
        
        for obs_x, obs_y, obs_radius in self.obstacles:
            # Calculate distance to obstacle
            to_obstacle = np.array([obs_x, obs_y]) - self.robot_position
            
            # Project onto ray direction
            projection = np.dot(to_obstacle, ray_dir)
            
            if projection > 0:  # Obstacle is in front
                # Calculate perpendicular distance
                perp_dist = np.linalg.norm(to_obstacle - projection * ray_dir)
                
                if perp_dist <= obs_radius:
                    # Ray intersects obstacle
                    intersection_dist = projection - np.sqrt(obs_radius**2 - perp_dist**2)
                    if intersection_dist > 0:
                        min_distance = min(min_distance, intersection_dist)
        
        return min_distance
    
    def set_robot_pose(self, position: np.ndarray, orientation: float):
        """Set robot pose for LiDAR simulation"""
        self.robot_position = position[:2].copy()
        self.robot_orientation = orientation
    
    def add_obstacle(self, x: float, y: float, radius: float):
        """Add circular obstacle to environment"""
        self.obstacles.append((x, y, radius))
    
    def clear_obstacles(self):
        """Clear all obstacles"""
        self.obstacles.clear()


class UltrasonicSensor(BaseSensor):
    """Ultrasonic distance sensor"""
    
    def __init__(self, sensor_id: str, position: np.ndarray, direction: np.ndarray,
                 max_range: float = 2.0, update_rate: float = 20.0):
        ultrasonic_noise = SensorNoise(
            white_noise_std=0.01,  # 1cm noise
            random_walk_std=0.0002,
            bias_stability=0.005
        )
        
        super().__init__(sensor_id, SensorType.ULTRASONIC, update_rate, ultrasonic_noise)
        
        self.position = position.copy()  # Sensor position relative to robot
        self.direction = direction / np.linalg.norm(direction)  # Unit vector
        self.max_range = max_range
        self.beam_width = np.radians(15.0)  # 15 degree beam width
        
        # Robot state
        self.robot_position = np.zeros(3)
        self.robot_orientation = 0.0
        
        logger.info(f"Ultrasonic sensor {sensor_id} initialized")
    
    def read_raw(self) -> np.ndarray:
        """Read raw ultrasonic distance"""
        # Transform sensor to world coordinates
        world_position = self.robot_position + self.position
        world_direction = self.rotate_vector(self.direction, self.robot_orientation)
        
        # Simple obstacle detection (placeholder)
        distance = self.max_range
        
        # Return single distance value
        return np.array([distance])
    
    def rotate_vector(self, vector: np.ndarray, angle: float) -> np.ndarray:
        """Rotate 2D vector by angle"""
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        rotation_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
        return rotation_matrix @ vector[:2]
    
    def set_robot_pose(self, position: np.ndarray, orientation: float):
        """Set robot pose for sensor simulation"""
        self.robot_position = position.copy()
        self.robot_orientation = orientation


# Utility functions
def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert Euler angles to quaternion (w,x,y,z)"""
    cr, cp, cy = np.cos(roll/2), np.cos(pitch/2), np.cos(yaw/2)
    sr, sp, sy = np.sin(roll/2), np.sin(pitch/2), np.sin(yaw/2)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return np.array([w, x, y, z])


def quaternion_to_euler(q: np.ndarray) -> Tuple[float, float, float]:
    """Convert quaternion (w,x,y,z) to Euler angles"""
    w, x, y, z = q
    
    # Roll
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw


class JointEncoder(BaseSensor):
    """Joint position encoder sensor"""
    
    def __init__(self, sensor_id: str, resolution: float = 0.1):
        super().__init__(sensor_id, SensorType.ENCODER)
        self.resolution = resolution  # degrees
        self.position = 0.0  # Current joint position in degrees
        self.noise_std = 0.05  # Standard deviation of noise in degrees
        
    def set_position(self, position: float):
        """Set the actual joint position"""
        self.position = position
    
    def read_raw(self) -> Optional[np.ndarray]:
        """Read raw sensor data"""
        # Add noise
        noise = np.random.normal(0, self.noise_std)
        measured_position = self.position + noise
        
        # Quantize based on resolution
        measured_position = np.round(measured_position / self.resolution) * self.resolution
        
        return np.array([measured_position])
        
    def read(self) -> Optional[SensorReading]:
        """Read joint position with noise"""
        raw_data = self.read_raw()
        if raw_data is None:
            return None
            
        measured_position = raw_data[0]
        
        # Calculate quality based on noise level
        noise = measured_position - self.position
        quality = max(0.0, 1.0 - abs(noise) / (3 * self.noise_std))
        
        return SensorReading(
            sensor_id=self.sensor_id,
            timestamp=time.time(),
            data=measured_position,
            quality=quality,
            sensor_type=self.sensor_type
        )


class ForceSensor(BaseSensor):
    """Force/torque sensor for legs"""
    
    def __init__(self, sensor_id: str, max_force: float = 100.0):
        super().__init__(sensor_id, SensorType.FORCE)
        self.max_force = max_force  # Maximum measurable force in Newtons
        self.force = np.array([0.0, 0.0, 0.0])  # Current force vector
        self.torque = np.array([0.0, 0.0, 0.0])  # Current torque vector
        self.noise_std = 0.1  # Standard deviation of noise
        
    def set_force(self, force: np.ndarray, torque: np.ndarray = None):
        """Set the actual force and torque"""
        self.force = force
        if torque is not None:
            self.torque = torque
    
    def read_raw(self) -> Optional[np.ndarray]:
        """Read raw sensor data"""        
        # Add noise to force and torque
        force_noise = np.random.normal(0, self.noise_std, 3)
        torque_noise = np.random.normal(0, self.noise_std, 3)
        
        measured_force = self.force + force_noise
        measured_torque = self.torque + torque_noise
        
        # Combine force and torque into 6D data
        return np.concatenate([measured_force, measured_torque])
            
    def read(self) -> Optional[SensorReading]:
        """Read force/torque data with noise"""
        data = self.read_raw()
        if data is None:
            return None
        
        # Calculate quality based on force magnitude
        force_magnitude = np.linalg.norm(data[:3])
        quality = min(1.0, force_magnitude / self.max_force)
        
        return SensorReading(
            sensor_id=self.sensor_id,
            timestamp=time.time(),
            data=data,
            quality=quality,
            sensor_type=self.sensor_type
        )


class DistanceSensor(BaseSensor):
    """Ultrasonic or infrared distance sensor"""
    
    def __init__(self, sensor_id: str, max_range: float = 4.0, direction: np.ndarray = None):
        super().__init__(sensor_id, SensorType.DISTANCE)
        self.max_range = max_range  # Maximum range in meters
        self.direction = direction if direction is not None else np.array([1.0, 0.0, 0.0])
        self.distance = max_range  # Current measured distance
        self.noise_std = 0.02  # Standard deviation of noise in meters
        
    def set_distance(self, distance: float):
        """Set the measured distance"""
        self.distance = min(distance, self.max_range)
    
    def read_raw(self) -> Optional[np.ndarray]:
        """Read raw sensor data"""        
        # Add noise
        noise = np.random.normal(0, self.noise_std)
        measured_distance = max(0.0, min(self.distance + noise, self.max_range))
        
        return np.array([measured_distance])
        
    def read(self) -> Optional[SensorReading]:
        """Read distance with noise"""
        raw_data = self.read_raw()
        if raw_data is None:
            return None
            
        measured_distance = raw_data[0]
        
        # Calculate quality based on range
        quality = 1.0 - (measured_distance / self.max_range)
        
        return SensorReading(
            sensor_id=self.sensor_id,
            timestamp=time.time(),
            data=measured_distance,
            quality=quality,
            sensor_type=self.sensor_type
        )


class TerrainSensor(BaseSensor):
    """Terrain sensing for ground contact and surface analysis"""
    
    def __init__(self, sensor_id: str = "terrain"):
        super().__init__(sensor_id, SensorType.TERRAIN)
        self.ground_contact = [False] * 6  # Ground contact for each leg
        self.ground_height = 0.0  # Ground height at robot position
        self.surface_normal = np.array([0.0, 0.0, 1.0])  # Surface normal vector
        self.roughness = 0.0  # Surface roughness metric
        
    def set_terrain_data(self, ground_contact: List[bool], ground_height: float, 
                        surface_normal: np.ndarray, roughness: float):
        """Set terrain sensing data"""
        self.ground_contact = ground_contact
        self.ground_height = ground_height
        self.surface_normal = surface_normal
        self.roughness = roughness
    
    def read_raw(self) -> Optional[np.ndarray]:
        """Read raw sensor data"""        
        # Package terrain data as numerical array
        # [contact_count, ground_height, normal_x, normal_y, normal_z, roughness]
        contact_count = sum(self.ground_contact)
        return np.array([
            contact_count, self.ground_height,
            self.surface_normal[0], self.surface_normal[1], self.surface_normal[2],
            self.roughness
        ])
        
    def read(self) -> Optional[SensorReading]:
        """Read terrain data"""
        # Package terrain data
        data = {
            'ground_contact': self.ground_contact,
            'ground_height': self.ground_height,
            'surface_normal': self.surface_normal,
            'roughness': self.roughness
        }
        
        # Quality based on number of legs in contact
        contact_count = sum(self.ground_contact)
        quality = contact_count / 6.0
        
        return SensorReading(
            sensor_id=self.sensor_id,
            timestamp=time.time(),
            data=data,
            quality=quality,
            sensor_type=self.sensor_type
        )


class SensorSuite:
    """Collection and management of multiple sensors"""
    
    def __init__(self):
        self.sensors: Dict[str, BaseSensor] = {}
        self.sensor_readings: Dict[str, SensorReading] = {}
        self.reading_history: Dict[str, deque] = {}
        self.max_history = 100  # Maximum number of readings to store
        
    def add_sensor(self, name: str, sensor: BaseSensor):
        """Add a sensor to the suite"""
        self.sensors[name] = sensor
        self.reading_history[name] = deque(maxlen=self.max_history)
        logger.info(f"Added sensor '{name}' of type {sensor.sensor_type}")
        
    def remove_sensor(self, name: str):
        """Remove a sensor from the suite"""
        if name in self.sensors:
            del self.sensors[name]
            del self.reading_history[name]
            if name in self.sensor_readings:
                del self.sensor_readings[name]
            logger.info(f"Removed sensor '{name}'")
            
    def read_sensor(self, name: str) -> Optional[SensorReading]:
        """Read a specific sensor"""
        if name not in self.sensors:
            return None
            
        reading = self.sensors[name].read()
        if reading:
            self.sensor_readings[name] = reading
            self.reading_history[name].append(reading)
            
        return reading
        
    def read_all_sensors(self) -> Dict[str, Any]:
        """Read all sensors and return data dictionary"""
        all_data = {}
        
        for name, sensor in self.sensors.items():
            reading = self.read_sensor(name)
            if reading:
                all_data[name] = reading.data
                
        return all_data
        
    def get_sensor_history(self, name: str, count: int = 10) -> List[SensorReading]:
        """Get recent history for a sensor"""
        if name not in self.reading_history:
            return []
            
        history = list(self.reading_history[name])
        return history[-count:] if len(history) > count else history
        
    def get_sensor_quality(self, name: str) -> float:
        """Get the current quality of a sensor"""
        if name in self.sensor_readings:
            return self.sensor_readings[name].quality
        return 0.0
        
    def enable_sensor(self, name: str):
        """Enable a specific sensor"""
        if name in self.sensors:
            self.sensors[name].enable()
            
    def disable_sensor(self, name: str):
        """Disable a specific sensor"""
        if name in self.sensors:
            self.sensors[name].disable()
            
    def calibrate_all(self):
        """Calibrate all sensors"""
        for sensor in self.sensors.values():
            sensor.calibrate()
        logger.info("All sensors calibrated")
        
    def get_status(self) -> Dict[str, Dict[str, Any]]:
        """Get status of all sensors"""
        status = {}
        
        for name, sensor in self.sensors.items():
            status[name] = {
                'type': sensor.sensor_type.value,
                'enabled': sensor.enabled,
                'last_reading': self.sensor_readings.get(name, None),
                'quality': self.get_sensor_quality(name),
                'history_count': len(self.reading_history[name])
            }
            
        return status


# Update SensorType enum to include missing types
class SensorType(Enum):
    """Sensor type enumeration"""
    IMU = "imu"
    GYROSCOPE = "gyroscope"
    ACCELEROMETER = "accelerometer"
    MAGNETOMETER = "magnetometer"
    POSITION = "position"
    ORIENTATION = "orientation"
    LIDAR = "lidar"
    ULTRASONIC = "ultrasonic"
    CAMERA = "camera"
    ENCODER = "encoder"
    FORCE = "force"
    DISTANCE = "distance"
    TERRAIN = "terrain"
    TEMPERATURE = "temperature"
    PRESSURE = "pressure"
    HUMIDITY = "humidity"


if __name__ == "__main__":
    # Test sensor implementations
    print("🤖 Phase 6: Testing Sensor Systems")
    print("=" * 40)
    
    # Test IMU sensor
    print("\n📱 Testing IMU Sensor...")
    imu = IMUSensor("test_imu")
    
    # Simulate motion
    imu.set_motion_state(
        angular_velocity=np.array([0.1, 0.0, 0.2]),  # rad/s
        linear_acceleration=np.array([0.0, 0.0, GRAVITY])  # m/s²
    )
    
    # Read sensor data
    reading = imu.read()
    if reading:
        print(f"IMU Reading: {reading.data[:6]}")  # Show gyro and accel
        print(f"Quality: {reading.quality:.3f}")
    
    # Test Position sensor
    print("\n📍 Testing Position Sensor...")
    pos_sensor = PositionSensor("test_position")
    pos_sensor.set_position(np.array([1.0, 2.0, 0.15]))
    
    reading = pos_sensor.read()
    if reading:
        print(f"Position: {reading.data}")
        print(f"Quality: {reading.quality:.3f}")
    
    # Test LiDAR sensor
    print("\n🔍 Testing LiDAR Sensor...")
    lidar = LiDARSensor("test_lidar", max_range=3.0, angular_resolution=5.0)
    
    # Add some obstacles
    lidar.add_obstacle(2.0, 0.0, 0.3)  # Obstacle at 2m forward
    lidar.add_obstacle(0.0, 1.5, 0.2)  # Obstacle to the right
    
    lidar.set_robot_pose(np.array([0.0, 0.0]), 0.0)
    
    reading = lidar.read()
    if reading:
        print(f"LiDAR ranges (first 10): {reading.data[:10]}")
        print(f"Min distance: {np.min(reading.data):.2f}m")
    
    print("\n✅ All sensor tests completed!")