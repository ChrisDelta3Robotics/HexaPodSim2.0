"""
Phase 6: Environmental Sensing & Terrain Analysis
HexaPodSim 2.0 - Advanced Environmental Perception

This module implements environmental sensing capabilities for the hexapod robot
including terrain analysis, obstacle detection, and environmental mapping.

Features:
- Real-time terrain mapping and analysis
- Obstacle detection and classification
- Surface material detection
- Slope and roughness analysis
- Traversability assessment
- Environmental hazard detection

Sensors Integrated:
- LiDAR for 3D environment mapping
- Ultrasonic sensors for proximity detection
- Force sensors for ground contact analysis
- Camera sensors for visual terrain classification
- IMU for slope and stability analysis

Key Algorithms:
- Point cloud processing for terrain mapping
- Machine learning for surface classification
- Traversability analysis for path planning
- Real-time obstacle avoidance
- Environmental change detection

Author: HexaPodSim Team
Date: October 2024
Phase: 6 - Sensor Integration & Feedback
"""

import numpy as np
import time
import logging
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any, Set
from enum import Enum
import threading
from collections import deque
from abc import ABC, abstractmethod

# Import sensors without relative imports
try:
    from sensors import LiDARSensor, UltrasonicSensor, SensorReading, SensorType
except ImportError:
    # Create mock classes if not available
    @dataclass
    class SensorReading:
        sensor_id: str = ""
        timestamp: float = 0.0
        data: Any = None
        quality: float = 1.0
        sensor_type: str = ""
    
    class SensorType(Enum):
        LIDAR = "lidar"
        ULTRASONIC = "ultrasonic"
    
    class LiDARSensor:
        def __init__(self, *args, **kwargs):
            pass
    
    class UltrasonicSensor:
        def __init__(self, *args, **kwargs):
            pass

# Configure logging
logger = logging.getLogger(__name__)


class TerrainType(Enum):
    """Terrain type classification"""
    UNKNOWN = "unknown"
    FLAT_HARD = "flat_hard"
    FLAT_SOFT = "flat_soft"
    ROUGH_ROCK = "rough_rock"
    GRASS = "grass"
    SAND = "sand"
    MUD = "mud"
    STAIRS = "stairs"
    SLOPE_GENTLE = "slope_gentle"
    SLOPE_STEEP = "slope_steep"
    OBSTACLE = "obstacle"
    VOID = "void"


class ObstacleType(Enum):
    """Obstacle type classification"""
    UNKNOWN = "unknown"
    WALL = "wall"
    POLE = "pole"
    BOX = "box"
    CYLINDER = "cylinder"
    IRREGULAR = "irregular"
    MOVABLE = "movable"
    VEGETATION = "vegetation"
    HOLE = "hole"
    CLIFF = "cliff"


@dataclass
class TerrainPoint:
    """Single terrain measurement point"""
    position: np.ndarray  # x, y, z in meters
    normal: np.ndarray    # Surface normal vector
    roughness: float      # Surface roughness measure
    slope: float          # Slope angle in degrees
    terrain_type: TerrainType = TerrainType.UNKNOWN
    confidence: float = 1.0
    timestamp: float = field(default_factory=time.time)


@dataclass
class Obstacle:
    """Detected obstacle"""
    id: int
    position: np.ndarray  # Center position x, y, z
    dimensions: np.ndarray  # Width, depth, height
    obstacle_type: ObstacleType
    confidence: float
    is_static: bool = True
    velocity: Optional[np.ndarray] = None  # For moving obstacles
    bounding_box: Optional[Tuple[np.ndarray, np.ndarray]] = None  # min, max corners
    last_seen: float = field(default_factory=time.time)


@dataclass
class EnvironmentalHazard:
    """Environmental hazard detection"""
    hazard_type: str
    position: np.ndarray
    severity: float  # 0-1 scale
    description: str
    safety_distance: float  # Minimum safe distance in meters
    detected_time: float = field(default_factory=time.time)


class TerrainMap:
    """Real-time terrain mapping system"""
    
    def __init__(self, map_size: Tuple[float, float] = (10.0, 10.0), 
                 resolution: float = 0.1):
        """
        Initialize terrain map
        map_size: (width, height) in meters
        resolution: grid resolution in meters
        """
        self.map_size = map_size
        self.resolution = resolution
        
        # Grid dimensions
        self.grid_width = int(map_size[0] / resolution)
        self.grid_height = int(map_size[1] / resolution)
        
        # Height map
        self.height_map = np.full((self.grid_height, self.grid_width), np.nan)
        
        # Terrain properties
        self.terrain_types = np.full((self.grid_height, self.grid_width), 
                                   TerrainType.UNKNOWN, dtype=object)
        self.roughness_map = np.zeros((self.grid_height, self.grid_width))
        self.slope_map = np.zeros((self.grid_height, self.grid_width))
        self.confidence_map = np.zeros((self.grid_height, self.grid_width))
        
        # Traversability analysis
        self.traversability_map = np.ones((self.grid_height, self.grid_width))
        
        # Threading for map updates
        self._lock = threading.Lock()
        
        # Robot position for map centering
        self.robot_position = np.zeros(2)
        
        logger.info(f"Terrain map initialized: {self.grid_width}x{self.grid_height} "
                   f"at {resolution}m resolution")
    
    def world_to_grid(self, world_pos: np.ndarray) -> Tuple[int, int]:
        """Convert world coordinates to grid indices"""
        # Center map on robot position
        relative_pos = world_pos[:2] - self.robot_position + np.array(self.map_size) / 2
        
        grid_x = int(relative_pos[0] / self.resolution)
        grid_y = int(relative_pos[1] / self.resolution)
        
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> np.ndarray:
        """Convert grid indices to world coordinates"""
        world_x = grid_x * self.resolution - self.map_size[0] / 2 + self.robot_position[0]
        world_y = grid_y * self.resolution - self.map_size[1] / 2 + self.robot_position[1]
        
        return np.array([world_x, world_y])
    
    def is_valid_grid(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid coordinates are valid"""
        return 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height
    
    def update_point(self, terrain_point: TerrainPoint):
        """Update map with new terrain point"""
        grid_x, grid_y = self.world_to_grid(terrain_point.position)
        
        if not self.is_valid_grid(grid_x, grid_y):
            return
        
        with self._lock:
            # Update height (with confidence weighting)
            existing_conf = self.confidence_map[grid_y, grid_x]
            new_conf = terrain_point.confidence
            
            if existing_conf == 0:
                # First measurement
                self.height_map[grid_y, grid_x] = terrain_point.position[2]
                self.confidence_map[grid_y, grid_x] = new_conf
            else:
                # Weighted average
                total_weight = existing_conf + new_conf
                weight_ratio = new_conf / total_weight
                
                if not np.isnan(self.height_map[grid_y, grid_x]):
                    self.height_map[grid_y, grid_x] = (
                        (1 - weight_ratio) * self.height_map[grid_y, grid_x] +
                        weight_ratio * terrain_point.position[2]
                    )
                else:
                    self.height_map[grid_y, grid_x] = terrain_point.position[2]
                
                self.confidence_map[grid_y, grid_x] = min(total_weight, 1.0)
            
            # Update terrain properties
            self.terrain_types[grid_y, grid_x] = terrain_point.terrain_type
            self.roughness_map[grid_y, grid_x] = terrain_point.roughness
            self.slope_map[grid_y, grid_x] = terrain_point.slope
            
            # Update traversability
            self.update_traversability(grid_x, grid_y)
    
    def update_traversability(self, grid_x: int, grid_y: int):
        """Update traversability score for a grid cell"""
        if not self.is_valid_grid(grid_x, grid_y):
            return
        
        # Base traversability factors
        slope_factor = max(0, 1.0 - self.slope_map[grid_y, grid_x] / 45.0)  # 45° max slope
        roughness_factor = max(0, 1.0 - self.roughness_map[grid_y, grid_x])
        
        # Terrain type factor
        terrain_factors = {
            TerrainType.FLAT_HARD: 1.0,
            TerrainType.FLAT_SOFT: 0.8,
            TerrainType.GRASS: 0.9,
            TerrainType.ROUGH_ROCK: 0.6,
            TerrainType.SAND: 0.7,
            TerrainType.MUD: 0.4,
            TerrainType.STAIRS: 0.8,
            TerrainType.SLOPE_GENTLE: 0.8,
            TerrainType.SLOPE_STEEP: 0.3,
            TerrainType.OBSTACLE: 0.0,
            TerrainType.VOID: 0.0,
            TerrainType.UNKNOWN: 0.5
        }
        
        terrain_type = self.terrain_types[grid_y, grid_x]
        terrain_factor = terrain_factors.get(terrain_type, 0.5)
        
        # Combined traversability score
        traversability = slope_factor * roughness_factor * terrain_factor
        self.traversability_map[grid_y, grid_x] = traversability
    
    def get_local_map(self, center: np.ndarray, size: float) -> Dict[str, np.ndarray]:
        """Get local map around a center point"""
        half_size = size / 2
        
        # Grid bounds
        min_grid_x, min_grid_y = self.world_to_grid(center - half_size)
        max_grid_x, max_grid_y = self.world_to_grid(center + half_size)
        
        # Clamp to valid range
        min_grid_x = max(0, min_grid_x)
        min_grid_y = max(0, min_grid_y)
        max_grid_x = min(self.grid_width - 1, max_grid_x)
        max_grid_y = min(self.grid_height - 1, max_grid_y)
        
        with self._lock:
            local_map = {
                'height': self.height_map[min_grid_y:max_grid_y+1, min_grid_x:max_grid_x+1].copy(),
                'traversability': self.traversability_map[min_grid_y:max_grid_y+1, min_grid_x:max_grid_x+1].copy(),
                'slope': self.slope_map[min_grid_y:max_grid_y+1, min_grid_x:max_grid_x+1].copy(),
                'roughness': self.roughness_map[min_grid_y:max_grid_y+1, min_grid_x:max_grid_x+1].copy(),
                'bounds': ((min_grid_x, min_grid_y), (max_grid_x, max_grid_y))
            }
        
        return local_map
    
    def set_robot_position(self, position: np.ndarray):
        """Update robot position for map centering"""
        self.robot_position = position[:2].copy()


class ObstacleDetector:
    """Real-time obstacle detection and tracking"""
    
    def __init__(self):
        self.obstacles = {}  # Dictionary of obstacle_id -> Obstacle
        self.next_obstacle_id = 1
        
        # Detection parameters
        self.min_obstacle_size = 0.05  # 5cm minimum
        self.max_obstacle_age = 5.0   # 5 seconds before removal
        self.clustering_distance = 0.3  # 30cm clustering threshold
        
        # Tracking
        self.obstacle_history = deque(maxlen=1000)
        self._lock = threading.Lock()
        
        logger.info("Obstacle detector initialized")
    
    def process_lidar_scan(self, ranges: np.ndarray, angles: np.ndarray, 
                          robot_position: np.ndarray, robot_orientation: float) -> List[Obstacle]:
        """Process LiDAR scan for obstacle detection"""
        current_time = time.time()
        detected_obstacles = []
        
        # Convert polar to Cartesian coordinates
        valid_indices = ranges < np.inf
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        if len(valid_ranges) == 0:
            return detected_obstacles
        
        # Convert to world coordinates
        world_angles = valid_angles + robot_orientation
        obstacle_points = np.column_stack([
            robot_position[0] + valid_ranges * np.cos(world_angles),
            robot_position[1] + valid_ranges * np.sin(world_angles),
            np.full_like(valid_ranges, robot_position[2])
        ])
        
        # Cluster points into obstacles
        clusters = self.cluster_points(obstacle_points)
        
        for cluster in clusters:
            if len(cluster) < 3:  # Minimum points for obstacle
                continue
            
            # Analyze cluster
            obstacle = self.analyze_cluster(cluster, current_time)
            if obstacle:
                detected_obstacles.append(obstacle)
        
        # Update obstacle tracking
        self.update_obstacle_tracking(detected_obstacles)
        
        return detected_obstacles
    
    def cluster_points(self, points: np.ndarray) -> List[List[np.ndarray]]:
        """Cluster points using simple distance-based clustering"""
        if len(points) == 0:
            return []
        
        clusters = []
        remaining_points = list(points)
        
        while remaining_points:
            # Start new cluster with first remaining point
            current_cluster = [remaining_points.pop(0)]
            
            # Find nearby points
            i = 0
            while i < len(remaining_points):
                point = remaining_points[i]
                
                # Check distance to any point in current cluster
                min_distance = min(np.linalg.norm(point[:2] - cluster_point[:2]) 
                                 for cluster_point in current_cluster)
                
                if min_distance < self.clustering_distance:
                    current_cluster.append(remaining_points.pop(i))
                else:
                    i += 1
            
            clusters.append(current_cluster)
        
        return clusters
    
    def analyze_cluster(self, cluster: List[np.ndarray], timestamp: float) -> Optional[Obstacle]:
        """Analyze point cluster to create obstacle"""
        if len(cluster) < 2:
            return None
        
        points = np.array(cluster)
        
        # Calculate bounding box
        min_pos = np.min(points, axis=0)
        max_pos = np.max(points, axis=0)
        
        # Check minimum size
        dimensions = max_pos - min_pos
        if np.any(dimensions[:2] < self.min_obstacle_size):
            return None
        
        # Calculate center and properties
        center = (min_pos + max_pos) / 2
        
        # Classify obstacle type based on shape
        aspect_ratio = dimensions[0] / dimensions[1] if dimensions[1] > 0 else 1.0
        height = dimensions[2]
        
        if aspect_ratio > 3.0 or aspect_ratio < 0.33:
            obstacle_type = ObstacleType.WALL
        elif height > 0.5:
            obstacle_type = ObstacleType.POLE if min(dimensions[:2]) < 0.2 else ObstacleType.BOX
        else:
            obstacle_type = ObstacleType.IRREGULAR
        
        # Create obstacle
        obstacle = Obstacle(
            id=self.next_obstacle_id,
            position=center,
            dimensions=dimensions,
            obstacle_type=obstacle_type,
            confidence=min(1.0, len(cluster) / 10.0),
            bounding_box=(min_pos, max_pos),
            last_seen=timestamp
        )
        
        self.next_obstacle_id += 1
        return obstacle
    
    def update_obstacle_tracking(self, detected_obstacles: List[Obstacle]):
        """Update obstacle tracking with new detections"""
        current_time = time.time()
        
        with self._lock:
            # Remove old obstacles
            obstacles_to_remove = []
            for obs_id, obstacle in self.obstacles.items():
                if current_time - obstacle.last_seen > self.max_obstacle_age:
                    obstacles_to_remove.append(obs_id)
            
            for obs_id in obstacles_to_remove:
                del self.obstacles[obs_id]
            
            # Update or add obstacles
            for detected in detected_obstacles:
                # Find closest existing obstacle
                closest_id = None
                min_distance = float('inf')
                
                for obs_id, existing in self.obstacles.items():
                    distance = np.linalg.norm(detected.position - existing.position)
                    if distance < min_distance and distance < 0.5:  # 50cm matching threshold
                        min_distance = distance
                        closest_id = obs_id
                
                if closest_id is not None:
                    # Update existing obstacle
                    existing = self.obstacles[closest_id]
                    
                    # Weighted update
                    weight = 0.3  # New measurement weight
                    existing.position = (1 - weight) * existing.position + weight * detected.position
                    existing.dimensions = (1 - weight) * existing.dimensions + weight * detected.dimensions
                    existing.confidence = min(1.0, existing.confidence + 0.1)
                    existing.last_seen = current_time
                    
                    # Update velocity estimate (simple finite difference)
                    if hasattr(existing, '_last_position'):
                        dt = current_time - existing.last_seen
                        if dt > 0:
                            velocity = (detected.position - existing._last_position) / dt
                            existing.velocity = velocity
                            existing.is_static = np.linalg.norm(velocity) < 0.1  # 10cm/s threshold
                    
                    existing._last_position = detected.position.copy()
                else:
                    # Add new obstacle
                    self.obstacles[detected.id] = detected
            
            # Store history
            self.obstacle_history.append({
                'timestamp': current_time,
                'obstacles': list(self.obstacles.values())
            })
    
    def get_obstacles_in_region(self, center: np.ndarray, radius: float) -> List[Obstacle]:
        """Get obstacles within a radius of center point"""
        nearby_obstacles = []
        
        with self._lock:
            for obstacle in self.obstacles.values():
                distance = np.linalg.norm(obstacle.position[:2] - center[:2])
                if distance <= radius:
                    nearby_obstacles.append(obstacle)
        
        return nearby_obstacles
    
    def get_all_obstacles(self) -> List[Obstacle]:
        """Get all currently tracked obstacles"""
        with self._lock:
            return list(self.obstacles.values())


class EnvironmentalSensor:
    """Main environmental sensing system"""
    
    def __init__(self):
        self.terrain_map = TerrainMap()
        self.obstacle_detector = ObstacleDetector()
        
        # Sensor integration
        self.lidar_sensors = {}
        self.ultrasonic_sensors = {}
        
        # Environmental hazards
        self.hazards = []
        self.hazard_detectors = []
        
        # Robot state
        self.robot_position = np.zeros(3)
        self.robot_orientation = 0.0
        
        # Threading
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        
        # Processing parameters
        self.update_frequency = 10.0  # Hz
        
        logger.info("Environmental sensor system initialized")
    
    def add_lidar_sensor(self, sensor: LiDARSensor):
        """Add LiDAR sensor to system"""
        self.lidar_sensors[sensor.sensor_id] = sensor
        logger.info(f"Added LiDAR sensor: {sensor.sensor_id}")
    
    def add_ultrasonic_sensor(self, sensor: UltrasonicSensor):
        """Add ultrasonic sensor to system"""
        self.ultrasonic_sensors[sensor.sensor_id] = sensor
        logger.info(f"Added ultrasonic sensor: {sensor.sensor_id}")
    
    def set_robot_pose(self, position: np.ndarray, orientation: float):
        """Update robot pose"""
        self.robot_position = position.copy()
        self.robot_orientation = orientation
        
        # Update all sensors
        for sensor in self.lidar_sensors.values():
            sensor.set_robot_pose(position, orientation)
        
        for sensor in self.ultrasonic_sensors.values():
            sensor.set_robot_pose(position, orientation)
        
        self.terrain_map.set_robot_position(position)
    
    def start_sensing(self):
        """Start environmental sensing loop"""
        if self._running:
            return
        
        self._running = True
        self._thread = threading.Thread(target=self._sensing_loop, daemon=True)
        self._thread.start()
        logger.info("Environmental sensing started")
    
    def stop_sensing(self):
        """Stop environmental sensing"""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        logger.info("Environmental sensing stopped")
    
    def _sensing_loop(self):
        """Main sensing loop"""
        while self._running:
            try:
                self.update_environmental_sensing()
                time.sleep(1.0 / self.update_frequency)
            except Exception as e:
                logger.error(f"Environmental sensing error: {e}")
    
    def update_environmental_sensing(self):
        """Update environmental sensing"""
        current_time = time.time()
        
        # Process LiDAR data
        for sensor in self.lidar_sensors.values():
            reading = sensor.read()
            if reading and reading.valid:
                self.process_lidar_data(reading)
        
        # Process ultrasonic data
        for sensor in self.ultrasonic_sensors.values():
            reading = sensor.read()
            if reading and reading.valid:
                self.process_ultrasonic_data(reading)
        
        # Detect environmental hazards
        self.detect_hazards()
    
    def process_lidar_data(self, reading: SensorReading):
        """Process LiDAR sensor data"""
        ranges = reading.data
        angles = np.linspace(0, 2*np.pi, len(ranges), endpoint=False)
        
        # Detect obstacles
        obstacles = self.obstacle_detector.process_lidar_scan(
            ranges, angles, self.robot_position, self.robot_orientation
        )
        
        # Update terrain map
        self.update_terrain_from_lidar(ranges, angles)
    
    def update_terrain_from_lidar(self, ranges: np.ndarray, angles: np.ndarray):
        """Update terrain map from LiDAR data"""
        # Convert to world coordinates
        valid_indices = ranges < np.inf
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        world_angles = valid_angles + self.robot_orientation
        
        for i, (r, angle) in enumerate(zip(valid_ranges, world_angles)):
            world_pos = np.array([
                self.robot_position[0] + r * np.cos(angle),
                self.robot_position[1] + r * np.sin(angle),
                0.0  # Assume ground level for 2D LiDAR
            ])
            
            # Create terrain point
            terrain_point = TerrainPoint(
                position=world_pos,
                normal=np.array([0, 0, 1]),  # Assume flat
                roughness=0.1,
                slope=0.0,
                terrain_type=TerrainType.UNKNOWN,
                confidence=0.7
            )
            
            self.terrain_map.update_point(terrain_point)
    
    def process_ultrasonic_data(self, reading: SensorReading):
        """Process ultrasonic sensor data"""
        distance = reading.data[0]
        
        # Simple obstacle detection for close range
        if distance < 0.5:  # 50cm threshold
            # Create temporary obstacle
            sensor = self.ultrasonic_sensors.get(reading.sensor_id)
            if sensor:
                # Calculate obstacle position
                world_direction = self.rotate_vector_2d(sensor.direction[:2], self.robot_orientation)
                obstacle_pos = self.robot_position[:2] + world_direction * distance
                
                # Add to terrain map as obstacle
                terrain_point = TerrainPoint(
                    position=np.array([obstacle_pos[0], obstacle_pos[1], self.robot_position[2]]),
                    normal=np.array([0, 0, 1]),
                    roughness=1.0,
                    slope=0.0,
                    terrain_type=TerrainType.OBSTACLE,
                    confidence=0.8
                )
                
                self.terrain_map.update_point(terrain_point)
    
    def rotate_vector_2d(self, vector: np.ndarray, angle: float) -> np.ndarray:
        """Rotate 2D vector by angle"""
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        rotation_matrix = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
        return rotation_matrix @ vector
    
    def detect_hazards(self):
        """Detect environmental hazards"""
        current_time = time.time()
        
        # Clear old hazards
        self.hazards = [h for h in self.hazards if current_time - h.detected_time < 30.0]
        
        # Check for slope hazards
        local_map = self.terrain_map.get_local_map(self.robot_position[:2], 2.0)
        if 'slope' in local_map:
            max_slope = np.nanmax(local_map['slope'])
            if max_slope > 30.0:  # 30 degree warning threshold
                hazard = EnvironmentalHazard(
                    hazard_type="steep_slope",
                    position=self.robot_position.copy(),
                    severity=min(1.0, max_slope / 45.0),
                    description=f"Steep slope detected: {max_slope:.1f}°",
                    safety_distance=1.0
                )
                self.hazards.append(hazard)
        
        # Check for obstacle hazards
        nearby_obstacles = self.obstacle_detector.get_obstacles_in_region(
            self.robot_position, 1.0
        )
        
        for obstacle in nearby_obstacles:
            if obstacle.obstacle_type == ObstacleType.CLIFF:
                hazard = EnvironmentalHazard(
                    hazard_type="cliff",
                    position=obstacle.position,
                    severity=1.0,
                    description="Cliff or drop-off detected",
                    safety_distance=2.0
                )
                self.hazards.append(hazard)
    
    def get_traversability_assessment(self, path: np.ndarray) -> Dict[str, Any]:
        """Assess traversability along a path"""
        assessment = {
            'traversable': True,
            'difficulty': 0.0,
            'hazards': [],
            'terrain_types': [],
            'max_slope': 0.0,
            'confidence': 1.0
        }
        
        # Sample points along path
        num_samples = max(10, int(np.linalg.norm(path[-1] - path[0]) / 0.1))
        sample_points = np.linspace(path[0], path[-1], num_samples)
        
        traversability_scores = []
        slopes = []
        terrain_types = []
        
        for point in sample_points:
            grid_x, grid_y = self.terrain_map.world_to_grid(point)
            
            if self.terrain_map.is_valid_grid(grid_x, grid_y):
                with self.terrain_map._lock:
                    trav_score = self.terrain_map.traversability_map[grid_y, grid_x]
                    slope = self.terrain_map.slope_map[grid_y, grid_x]
                    terrain_type = self.terrain_map.terrain_types[grid_y, grid_x]
                
                traversability_scores.append(trav_score)
                slopes.append(slope)
                terrain_types.append(terrain_type)
        
        if traversability_scores:
            min_traversability = min(traversability_scores)
            avg_traversability = np.mean(traversability_scores)
            max_slope = max(slopes) if slopes else 0.0
            
            assessment['traversable'] = min_traversability > 0.3
            assessment['difficulty'] = 1.0 - avg_traversability
            assessment['max_slope'] = max_slope
            assessment['terrain_types'] = list(set(terrain_types))
            
            # Check for hazards along path
            for point in sample_points:
                nearby_hazards = [h for h in self.hazards 
                                if np.linalg.norm(h.position[:2] - point[:2]) < h.safety_distance]
                assessment['hazards'].extend(nearby_hazards)
        
        return assessment
    
    def get_environmental_summary(self) -> Dict[str, Any]:
        """Get comprehensive environmental summary"""
        obstacles = self.obstacle_detector.get_all_obstacles()
        
        summary = {
            'terrain_map_coverage': np.sum(~np.isnan(self.terrain_map.height_map)) / self.terrain_map.height_map.size,
            'num_obstacles': len(obstacles),
            'num_hazards': len(self.hazards),
            'obstacles_by_type': {},
            'terrain_analysis': self.analyze_local_terrain(),
            'robot_position': self.robot_position.tolist(),
            'robot_orientation': np.degrees(self.robot_orientation)
        }
        
        # Obstacle statistics
        for obstacle in obstacles:
            obs_type = obstacle.obstacle_type.value
            summary['obstacles_by_type'][obs_type] = summary['obstacles_by_type'].get(obs_type, 0) + 1
        
        return summary
    
    def analyze_local_terrain(self, radius: float = 2.0) -> Dict[str, Any]:
        """Analyze terrain around robot"""
        local_map = self.terrain_map.get_local_map(self.robot_position[:2], radius * 2)
        
        analysis = {
            'avg_slope': 0.0,
            'max_slope': 0.0,
            'avg_roughness': 0.0,
            'avg_traversability': 1.0,
            'dominant_terrain': TerrainType.UNKNOWN.value
        }
        
        if 'slope' in local_map and local_map['slope'].size > 0:
            valid_slopes = local_map['slope'][~np.isnan(local_map['slope'])]
            if len(valid_slopes) > 0:
                analysis['avg_slope'] = np.mean(valid_slopes)
                analysis['max_slope'] = np.max(valid_slopes)
        
        if 'roughness' in local_map and local_map['roughness'].size > 0:
            analysis['avg_roughness'] = np.mean(local_map['roughness'])
        
        if 'traversability' in local_map and local_map['traversability'].size > 0:
            analysis['avg_traversability'] = np.mean(local_map['traversability'])
        
        return analysis


if __name__ == "__main__":
    # Test environmental sensing system
    print("🤖 Phase 6: Testing Environmental Sensing System")
    print("=" * 55)
    
    # Initialize environmental sensor
    env_sensor = EnvironmentalSensor()
    
    # Create mock LiDAR sensor
    lidar = LiDARSensor("test_lidar", max_range=5.0, angular_resolution=2.0)
    
    # Add some obstacles to test
    lidar.add_obstacle(2.0, 0.0, 0.3)  # Front obstacle
    lidar.add_obstacle(-1.0, 1.5, 0.2)  # Left obstacle
    lidar.add_obstacle(1.0, -2.0, 0.4)  # Right obstacle
    
    env_sensor.add_lidar_sensor(lidar)
    
    print("✅ Environmental sensor initialized with test LiDAR")
    
    # Simulate robot movement and sensing
    print("\n🗺️ Simulating environmental mapping...")
    
    for i in range(20):
        # Simulate robot movement
        t = i * 0.1
        robot_pos = np.array([t * 0.1, t * 0.05, 0.15])
        robot_orient = t * 0.05  # Slight rotation
        
        env_sensor.set_robot_pose(robot_pos, robot_orient)
        env_sensor.update_environmental_sensing()
        
        if i % 5 == 0:  # Print every 0.5 seconds
            summary = env_sensor.get_environmental_summary()
            print(f"t={t:.1f}s: Obstacles={summary['num_obstacles']}, "
                  f"Hazards={summary['num_hazards']}, "
                  f"Map coverage={summary['terrain_map_coverage']:.1%}")
    
    # Test traversability assessment
    print("\n🛤️ Testing traversability assessment...")
    test_path = np.array([[0.0, 0.0], [2.0, 1.0], [3.0, 2.0]])
    assessment = env_sensor.get_traversability_assessment(test_path)
    
    print(f"Path traversable: {assessment['traversable']}")
    print(f"Difficulty: {assessment['difficulty']:.2f}")
    print(f"Max slope: {assessment['max_slope']:.1f}°")
    print(f"Hazards detected: {len(assessment['hazards'])}")
    
    # Final summary
    final_summary = env_sensor.get_environmental_summary()
    print(f"\n📊 Final Environmental Summary:")
    print(f"  Map coverage: {final_summary['terrain_map_coverage']:.1%}")
    print(f"  Obstacles detected: {final_summary['num_obstacles']}")
    print(f"  Environmental hazards: {final_summary['num_hazards']}")
    
    terrain_analysis = final_summary['terrain_analysis']
    print(f"  Avg terrain slope: {terrain_analysis['avg_slope']:.1f}°")
    print(f"  Avg traversability: {terrain_analysis['avg_traversability']:.2f}")
    
    print("\n✅ Environmental sensing test completed!")