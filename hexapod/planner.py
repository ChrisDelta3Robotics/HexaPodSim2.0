"""
Path Planning Module for HexaPodSim 2.0

This module implements various path planning algorithms for autonomous navigation:
- A* pathfinding for global path planning
- Probabilistic Roadmap (PRM) for local navigation
- Footstep planning for terrain navigation
- Hierarchical control integration

Author: HexaPodSim Team
Date: 2024
"""

import numpy as np
import heapq
import logging
from typing import List, Tuple, Optional, Dict, Set, Callable
from dataclasses import dataclass, field
from enum import Enum
import time
import random
from abc import ABC, abstractmethod

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


@dataclass
class Point2D:
    """2D point representation"""
    x: float
    y: float
    
    def __post_init__(self):
        self.x = float(self.x)
        self.y = float(self.y)
    
    def distance_to(self, other: 'Point2D') -> float:
        """Calculate Euclidean distance to another point"""
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def manhattan_distance_to(self, other: 'Point2D') -> float:
        """Calculate Manhattan distance to another point"""
        return abs(self.x - other.x) + abs(self.y - other.y)
    
    def __add__(self, other: 'Point2D') -> 'Point2D':
        return Point2D(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other: 'Point2D') -> 'Point2D':
        return Point2D(self.x - other.x, self.y - other.y)
    
    def __mul__(self, scalar: float) -> 'Point2D':
        return Point2D(self.x * scalar, self.y * scalar)
    
    def __eq__(self, other: 'Point2D') -> bool:
        return np.isclose(self.x, other.x) and np.isclose(self.y, other.y)
    
    def __hash__(self) -> int:
        return hash((round(self.x, 6), round(self.y, 6)))


@dataclass
class GridCell:
    """Grid cell representation for occupancy mapping"""
    x: int
    y: int
    cost: float = 1.0
    occupied: bool = False
    
    def __eq__(self, other: 'GridCell') -> bool:
        return self.x == other.x and self.y == other.y
    
    def __hash__(self) -> int:
        return hash((self.x, self.y))
    
    def __lt__(self, other: 'GridCell') -> bool:
        return self.cost < other.cost


class HeuristicType(Enum):
    """Heuristic function types for A* algorithm"""
    MANHATTAN = "manhattan"
    EUCLIDEAN = "euclidean"
    DIAGONAL = "diagonal"
    OCTILE = "octile"


class OccupancyGrid:
    """
    2D occupancy grid for representing environment obstacles and costs
    """
    
    def __init__(self, width: int, height: int, resolution: float = 0.05):
        """
        Initialize occupancy grid
        
        Args:
            width: Grid width in cells
            height: Grid height in cells
            resolution: Size of each cell in meters
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        
        # Initialize grid with free space (False = free, True = occupied)
        self.grid = np.zeros((height, width), dtype=bool)
        
        # Cost grid for path planning (1.0 = normal cost, higher = avoid)
        self.cost_grid = np.ones((height, width), dtype=float)
        
        # Safety margin around obstacles
        self.safety_margin = 0.1  # meters
        self.safety_cells = int(self.safety_margin / resolution)
        
        logger.info(f"Initialized occupancy grid: {width}x{height} cells at {resolution}m resolution")
    
    def world_to_grid(self, point: Point2D) -> Tuple[int, int]:
        """Convert world coordinates to grid indices"""
        grid_x = int(point.x / self.resolution)
        grid_y = int(point.y / self.resolution)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Point2D:
        """Convert grid indices to world coordinates"""
        world_x = (grid_x + 0.5) * self.resolution
        world_y = (grid_y + 0.5) * self.resolution
        return Point2D(world_x, world_y)
    
    def is_valid_cell(self, grid_x: int, grid_y: int) -> bool:
        """Check if grid coordinates are within bounds"""
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height
    
    def is_occupied(self, grid_x: int, grid_y: int) -> bool:
        """Check if a grid cell is occupied"""
        if not self.is_valid_cell(grid_x, grid_y):
            return True  # Out of bounds considered occupied
        return self.grid[grid_y, grid_x]
    
    def set_occupied(self, grid_x: int, grid_y: int, occupied: bool = True):
        """Set occupancy status of a grid cell"""
        if self.is_valid_cell(grid_x, grid_y):
            self.grid[grid_y, grid_x] = occupied
    
    def add_obstacle(self, center: Point2D, radius: float):
        """Add circular obstacle to grid"""
        center_x, center_y = self.world_to_grid(center)
        radius_cells = int(radius / self.resolution)
        
        for y in range(max(0, center_y - radius_cells), 
                      min(self.height, center_y + radius_cells + 1)):
            for x in range(max(0, center_x - radius_cells), 
                          min(self.width, center_x + radius_cells + 1)):
                if np.sqrt((x - center_x)**2 + (y - center_y)**2) <= radius_cells:
                    self.set_occupied(x, y, True)
        
        self._update_safety_margins()
        logger.info(f"Added obstacle at {center} with radius {radius}m")
    
    def add_rectangle_obstacle(self, corner1: Point2D, corner2: Point2D):
        """Add rectangular obstacle to grid"""
        min_x = min(corner1.x, corner2.x)
        max_x = max(corner1.x, corner2.x)
        min_y = min(corner1.y, corner2.y)
        max_y = max(corner1.y, corner2.y)
        
        grid_min_x, grid_min_y = self.world_to_grid(Point2D(min_x, min_y))
        grid_max_x, grid_max_y = self.world_to_grid(Point2D(max_x, max_y))
        
        for y in range(grid_min_y, grid_max_y + 1):
            for x in range(grid_min_x, grid_max_x + 1):
                self.set_occupied(x, y, True)
        
        self._update_safety_margins()
        logger.info(f"Added rectangle obstacle from {corner1} to {corner2}")
    
    def _update_safety_margins(self):
        """Update cost grid with safety margins around obstacles"""
        if self.safety_cells <= 0:
            return
        
        # Reset cost grid
        self.cost_grid.fill(1.0)
        
        # Add higher costs around obstacles
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y, x]:  # If occupied
                    # Add safety margin around obstacle
                    for dy in range(-self.safety_cells, self.safety_cells + 1):
                        for dx in range(-self.safety_cells, self.safety_cells + 1):
                            safe_x, safe_y = x + dx, y + dy
                            if self.is_valid_cell(safe_x, safe_y):
                                distance = np.sqrt(dx**2 + dy**2)
                                if distance <= self.safety_cells:
                                    # Higher cost closer to obstacles
                                    cost_multiplier = 1.0 + (self.safety_cells - distance) * 5.0
                                    self.cost_grid[safe_y, safe_x] = max(
                                        self.cost_grid[safe_y, safe_x], 
                                        cost_multiplier
                                    )
    
    def get_cost(self, grid_x: int, grid_y: int) -> float:
        """Get movement cost for a grid cell"""
        if not self.is_valid_cell(grid_x, grid_y):
            return float('inf')
        if self.is_occupied(grid_x, grid_y):
            return float('inf')
        return self.cost_grid[grid_y, grid_x]
    
    def get_neighbors(self, grid_x: int, grid_y: int, include_diagonals: bool = True) -> List[Tuple[int, int]]:
        """Get valid neighboring cells"""
        neighbors = []
        
        # 4-connected neighbors
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
        # Add diagonal neighbors if requested
        if include_diagonals:
            directions.extend([(1, 1), (1, -1), (-1, 1), (-1, -1)])
        
        for dx, dy in directions:
            neighbor_x, neighbor_y = grid_x + dx, grid_y + dy
            if (self.is_valid_cell(neighbor_x, neighbor_y) and 
                not self.is_occupied(neighbor_x, neighbor_y)):
                neighbors.append((neighbor_x, neighbor_y))
        
        return neighbors


class AStarPlanner:
    """
    A* pathfinding algorithm implementation for hexapod navigation
    """
    
    def __init__(self, occupancy_grid: OccupancyGrid, 
                 heuristic_type: HeuristicType = HeuristicType.EUCLIDEAN):
        """
        Initialize A* planner
        
        Args:
            occupancy_grid: Environment representation
            heuristic_type: Heuristic function to use
        """
        self.grid = occupancy_grid
        self.heuristic_type = heuristic_type
        
        # Planning statistics
        self.last_search_stats = {
            'nodes_expanded': 0,
            'path_length': 0,
            'planning_time': 0.0,
            'path_cost': 0.0
        }
        
        logger.info(f"Initialized A* planner with {heuristic_type.value} heuristic")
    
    def heuristic(self, current: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Calculate heuristic cost from current to goal"""
        curr_x, curr_y = current
        goal_x, goal_y = goal
        
        dx = abs(curr_x - goal_x)
        dy = abs(curr_y - goal_y)
        
        if self.heuristic_type == HeuristicType.MANHATTAN:
            return dx + dy
        elif self.heuristic_type == HeuristicType.EUCLIDEAN:
            return np.sqrt(dx**2 + dy**2)
        elif self.heuristic_type == HeuristicType.DIAGONAL:
            return max(dx, dy)
        elif self.heuristic_type == HeuristicType.OCTILE:
            # Octile distance (allows 8-directional movement)
            return max(dx, dy) + (np.sqrt(2) - 1) * min(dx, dy)
        else:
            return np.sqrt(dx**2 + dy**2)  # Default to Euclidean
    
    def plan_path(self, start: Point2D, goal: Point2D) -> Optional[List[Point2D]]:
        """
        Plan path from start to goal using A* algorithm
        
        Args:
            start: Starting position in world coordinates
            goal: Goal position in world coordinates
            
        Returns:
            List of waypoints in world coordinates, or None if no path found
        """
        start_time = time.time()
        
        # Convert to grid coordinates
        start_grid = self.grid.world_to_grid(start)
        goal_grid = self.grid.world_to_grid(goal)
        
        # Validate start and goal
        if self.grid.is_occupied(*start_grid):
            logger.warning(f"Start position {start} is occupied")
            return None
        
        if self.grid.is_occupied(*goal_grid):
            logger.warning(f"Goal position {goal} is occupied")
            return None
        
        # A* algorithm implementation
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}
        
        open_set_hash = {start_grid}
        closed_set = set()
        
        nodes_expanded = 0
        
        while open_set:
            # Get node with lowest f_score
            current_f, current = heapq.heappop(open_set)
            open_set_hash.remove(current)
            
            if current == goal_grid:
                # Reconstruct path
                path = self._reconstruct_path(came_from, current)
                
                # Convert to world coordinates
                world_path = [self.grid.grid_to_world(x, y) for x, y in path]
                
                # Update statistics
                planning_time = time.time() - start_time
                self.last_search_stats = {
                    'nodes_expanded': nodes_expanded,
                    'path_length': len(world_path),
                    'planning_time': planning_time,
                    'path_cost': g_score[current]
                }
                
                logger.info(f"Path found: {len(world_path)} waypoints, "
                           f"{nodes_expanded} nodes expanded, "
                           f"{planning_time:.3f}s planning time")
                
                return world_path
            
            closed_set.add(current)
            nodes_expanded += 1
            
            # Explore neighbors
            for neighbor in self.grid.get_neighbors(*current):
                if neighbor in closed_set:
                    continue
                
                # Calculate movement cost
                current_x, current_y = current
                neighbor_x, neighbor_y = neighbor
                
                # Movement cost includes grid cost and distance
                dx = abs(neighbor_x - current_x)
                dy = abs(neighbor_y - current_y)
                move_cost = np.sqrt(dx**2 + dy**2) if (dx > 0 and dy > 0) else 1.0
                
                tentative_g = g_score[current] + move_cost * self.grid.get_cost(*neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_grid)
                    
                    if neighbor not in open_set_hash:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        open_set_hash.add(neighbor)
        
        # No path found
        planning_time = time.time() - start_time
        self.last_search_stats = {
            'nodes_expanded': nodes_expanded,
            'path_length': 0,
            'planning_time': planning_time,
            'path_cost': float('inf')
        }
        
        logger.warning(f"No path found from {start} to {goal}")
        return None
    
    def _reconstruct_path(self, came_from: Dict, current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstruct path from came_from dictionary"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def smooth_path(self, path: List[Point2D], max_iterations: int = 100) -> List[Point2D]:
        """
        Smooth path using simple line-of-sight optimization
        
        Args:
            path: Original path waypoints
            max_iterations: Maximum smoothing iterations
            
        Returns:
            Smoothed path
        """
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        current_idx = 0
        
        for _ in range(max_iterations):
            if current_idx >= len(path) - 1:
                break
            
            # Find furthest point with line-of-sight
            furthest_idx = current_idx + 1
            for i in range(current_idx + 2, len(path)):
                if self._has_line_of_sight(path[current_idx], path[i]):
                    furthest_idx = i
                else:
                    break
            
            smoothed.append(path[furthest_idx])
            current_idx = furthest_idx
        
        logger.info(f"Path smoothed from {len(path)} to {len(smoothed)} waypoints")
        return smoothed
    
    def _has_line_of_sight(self, start: Point2D, end: Point2D) -> bool:
        """Check if there's a clear line of sight between two points"""
        start_grid = self.grid.world_to_grid(start)
        end_grid = self.grid.world_to_grid(end)
        
        # Bresenham's line algorithm for grid traversal
        x0, y0 = start_grid
        x1, y1 = end_grid
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            if self.grid.is_occupied(x, y):
                return False
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return True
    
    def get_search_statistics(self) -> Dict:
        """Get statistics from last path search"""
        return self.last_search_stats.copy()


# Path smoothing utilities
class PathSmoother:
    """
    Advanced path smoothing using Bezier curves and splines
    """
    
    @staticmethod
    def bezier_smooth(path: List[Point2D], num_points: int = 50) -> List[Point2D]:
        """
        Smooth path using cubic Bezier curves
        
        Args:
            path: Original path waypoints
            num_points: Number of points in smoothed path
            
        Returns:
            Smoothed path using Bezier curves
        """
        if len(path) < 3:
            return path
        
        smoothed_path = []
        
        for i in range(len(path) - 1):
            if i == 0:
                # First segment
                p0 = path[i]
                p1 = path[i]
                p2 = path[i + 1]
                p3 = path[i + 1] if i + 2 >= len(path) else path[i + 2]
            elif i == len(path) - 2:
                # Last segment
                p0 = path[i - 1] if i - 1 >= 0 else path[i]
                p1 = path[i]
                p2 = path[i + 1]
                p3 = path[i + 1]
            else:
                # Middle segments
                p0 = path[i - 1]
                p1 = path[i]
                p2 = path[i + 1]
                p3 = path[i + 2]
            
            # Generate Bezier curve points
            segment_points = num_points // (len(path) - 1)
            for j in range(segment_points):
                t = j / segment_points
                point = PathSmoother._cubic_bezier(p0, p1, p2, p3, t)
                smoothed_path.append(point)
        
        # Add final point
        smoothed_path.append(path[-1])
        
        return smoothed_path
    
    @staticmethod
    def _cubic_bezier(p0: Point2D, p1: Point2D, p2: Point2D, p3: Point2D, t: float) -> Point2D:
        """Calculate point on cubic Bezier curve"""
        u = 1 - t
        tt = t * t
        uu = u * u
        uuu = uu * u
        ttt = tt * t
        
        # Control points for smooth curve
        cp1 = p1 + (p2 - p0) * 0.25
        cp2 = p2 + (p1 - p3) * 0.25
        
        x = uuu * p1.x + 3 * uu * t * cp1.x + 3 * u * tt * cp2.x + ttt * p2.x
        y = uuu * p1.y + 3 * uu * t * cp1.y + 3 * u * tt * cp2.y + ttt * p2.y
        
        return Point2D(x, y)


@dataclass
class Point3D:
    """3D point representation for foot planning"""
    x: float
    y: float
    z: float
    
    def __post_init__(self):
        self.x = float(self.x)
        self.y = float(self.y)
        self.z = float(self.z)
    
    def distance_to(self, other: 'Point3D') -> float:
        """Calculate Euclidean distance to another point"""
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)
    
    def to_2d(self) -> Point2D:
        """Convert to 2D point (ignore z)"""
        return Point2D(self.x, self.y)
    
    def __add__(self, other: 'Point3D') -> 'Point3D':
        return Point3D(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other: 'Point3D') -> 'Point3D':
        return Point3D(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, scalar: float) -> 'Point3D':
        return Point3D(self.x * scalar, self.y * scalar, self.z * scalar)


@dataclass
class PRMNode:
    """Node in Probabilistic Roadmap"""
    position: Point3D
    id: int
    neighbors: List[int] = field(default_factory=list)
    
    def add_neighbor(self, neighbor_id: int):
        """Add neighbor connection"""
        if neighbor_id not in self.neighbors:
            self.neighbors.append(neighbor_id)


class PRMPlanner:
    """
    Probabilistic Roadmap (PRM) planner for foot trajectory planning
    """
    
    def __init__(self, workspace_bounds: Tuple[Point3D, Point3D], 
                 max_connection_distance: float = 0.2):
        """
        Initialize PRM planner
        
        Args:
            workspace_bounds: (min_point, max_point) defining workspace
            max_connection_distance: Maximum distance for connecting nodes
        """
        self.min_bounds, self.max_bounds = workspace_bounds
        self.max_connection_distance = max_connection_distance
        
        self.nodes: Dict[int, PRMNode] = {}
        self.next_node_id = 0
        
        # Obstacle checking function (can be set externally)
        self.collision_checker: Optional[Callable[[Point3D], bool]] = None
        self.line_collision_checker: Optional[Callable[[Point3D, Point3D], bool]] = None
        
        logger.info(f"Initialized PRM planner with workspace {self.min_bounds} to {self.max_bounds}")
    
    def set_collision_checkers(self, 
                             point_checker: Callable[[Point3D], bool],
                             line_checker: Callable[[Point3D, Point3D], bool]):
        """
        Set collision checking functions
        
        Args:
            point_checker: Function to check if a point is in collision
            line_checker: Function to check if a line segment is in collision
        """
        self.collision_checker = point_checker
        self.line_collision_checker = line_checker
    
    def sample_random_point(self) -> Point3D:
        """Sample random point in workspace"""
        x = random.uniform(self.min_bounds.x, self.max_bounds.x)
        y = random.uniform(self.min_bounds.y, self.max_bounds.y)
        z = random.uniform(self.min_bounds.z, self.max_bounds.z)
        return Point3D(x, y, z)
    
    def is_collision_free(self, point: Point3D) -> bool:
        """Check if point is collision-free"""
        if self.collision_checker is None:
            return True  # No collision checker set
        return not self.collision_checker(point)
    
    def is_path_collision_free(self, start: Point3D, end: Point3D) -> bool:
        """Check if path between two points is collision-free"""
        if self.line_collision_checker is None:
            return True  # No collision checker set
        return not self.line_collision_checker(start, end)
    
    def build_roadmap(self, num_samples: int = 500):
        """
        Build probabilistic roadmap
        
        Args:
            num_samples: Number of random samples to generate
        """
        logger.info(f"Building PRM roadmap with {num_samples} samples...")
        
        # Clear existing roadmap
        self.nodes.clear()
        self.next_node_id = 0
        
        # Sample points and add collision-free ones
        for _ in range(num_samples):
            point = self.sample_random_point()
            if self.is_collision_free(point):
                self.add_node(point)
        
        # Connect nearby nodes
        self.connect_nodes()
        
        logger.info(f"Roadmap built with {len(self.nodes)} nodes")
    
    def add_node(self, position: Point3D) -> int:
        """Add node to roadmap"""
        node = PRMNode(position, self.next_node_id)
        self.nodes[self.next_node_id] = node
        self.next_node_id += 1
        return node.id
    
    def connect_nodes(self):
        """Connect nearby nodes in roadmap"""
        node_list = list(self.nodes.values())
        
        for i, node1 in enumerate(node_list):
            for j, node2 in enumerate(node_list[i+1:], i+1):
                distance = node1.position.distance_to(node2.position)
                
                if (distance <= self.max_connection_distance and
                    self.is_path_collision_free(node1.position, node2.position)):
                    
                    node1.add_neighbor(node2.id)
                    node2.add_neighbor(node1.id)
    
    def find_nearest_node(self, point: Point3D) -> Optional[int]:
        """Find nearest node to given point"""
        if not self.nodes:
            return None
        
        min_distance = float('inf')
        nearest_id = None
        
        for node_id, node in self.nodes.items():
            distance = node.position.distance_to(point)
            if distance < min_distance:
                min_distance = distance
                nearest_id = node_id
        
        return nearest_id
    
    def plan_path(self, start: Point3D, goal: Point3D) -> Optional[List[Point3D]]:
        """
        Plan path from start to goal using roadmap
        
        Args:
            start: Starting position
            goal: Goal position
            
        Returns:
            List of waypoints or None if no path found
        """
        if not self.nodes:
            logger.warning("No roadmap built yet")
            return None
        
        # Add start and goal to roadmap temporarily
        start_id = None
        goal_id = None
        
        if self.is_collision_free(start):
            start_id = self.add_node(start)
            self.connect_node_to_roadmap(start_id)
        
        if self.is_collision_free(goal):
            goal_id = self.add_node(goal)
            self.connect_node_to_roadmap(goal_id)
        
        if start_id is None or goal_id is None:
            logger.warning("Start or goal position is in collision")
            return None
        
        # Find path using Dijkstra's algorithm
        path_node_ids = self.dijkstra(start_id, goal_id)
        
        if path_node_ids:
            path = [self.nodes[node_id].position for node_id in path_node_ids]
            logger.info(f"PRM path found with {len(path)} waypoints")
            return path
        else:
            logger.warning("No PRM path found")
            return None
    
    def connect_node_to_roadmap(self, node_id: int):
        """Connect a node to existing roadmap"""
        node = self.nodes[node_id]
        
        for other_id, other_node in self.nodes.items():
            if other_id == node_id:
                continue
            
            distance = node.position.distance_to(other_node.position)
            
            if (distance <= self.max_connection_distance and
                self.is_path_collision_free(node.position, other_node.position)):
                
                node.add_neighbor(other_id)
                other_node.add_neighbor(node_id)
    
    def dijkstra(self, start_id: int, goal_id: int) -> Optional[List[int]]:
        """Find shortest path in roadmap using Dijkstra's algorithm"""
        distances = {node_id: float('inf') for node_id in self.nodes}
        distances[start_id] = 0
        previous = {}
        
        unvisited = set(self.nodes.keys())
        
        while unvisited:
            # Find unvisited node with minimum distance
            current_id = min(unvisited, key=lambda x: distances[x])
            
            if distances[current_id] == float('inf'):
                break  # No path to remaining nodes
            
            if current_id == goal_id:
                # Reconstruct path
                path = []
                while current_id is not None:
                    path.append(current_id)
                    current_id = previous.get(current_id)
                path.reverse()
                return path
            
            unvisited.remove(current_id)
            current_node = self.nodes[current_id]
            
            # Update distances to neighbors
            for neighbor_id in current_node.neighbors:
                if neighbor_id in unvisited:
                    neighbor_node = self.nodes[neighbor_id]
                    edge_weight = current_node.position.distance_to(neighbor_node.position)
                    alt_distance = distances[current_id] + edge_weight
                    
                    if alt_distance < distances[neighbor_id]:
                        distances[neighbor_id] = alt_distance
                        previous[neighbor_id] = current_id
        
        return None  # No path found


class FootstepPlanner:
    """
    Intelligent footstep planning for terrain navigation
    """
    
    def __init__(self, step_length: float = 0.15, step_height: float = 0.05):
        """
        Initialize footstep planner
        
        Args:
            step_length: Maximum step length in meters
            step_height: Maximum step height in meters
        """
        self.step_length = step_length
        self.step_height = step_height
        
        # Terrain analysis functions (can be set externally)
        self.terrain_height_func: Optional[Callable[[float, float], float]] = None
        self.terrain_slope_func: Optional[Callable[[float, float], float]] = None
        self.terrain_stability_func: Optional[Callable[[float, float], float]] = None
        
        logger.info(f"Initialized footstep planner: max_step={step_length}m, max_height={step_height}m")
    
    def set_terrain_functions(self,
                            height_func: Callable[[float, float], float],
                            slope_func: Optional[Callable[[float, float], float]] = None,
                            stability_func: Optional[Callable[[float, float], float]] = None):
        """
        Set terrain analysis functions
        
        Args:
            height_func: Function to get terrain height at (x, y)
            slope_func: Function to get terrain slope at (x, y) in degrees
            stability_func: Function to get terrain stability score at (x, y) [0-1]
        """
        self.terrain_height_func = height_func
        self.terrain_slope_func = slope_func
        self.terrain_stability_func = stability_func
    
    def evaluate_foothold(self, position: Point2D) -> float:
        """
        Evaluate quality of foothold at given position
        
        Args:
            position: 2D position to evaluate
            
        Returns:
            Foothold score [0-1], higher is better
        """
        if self.terrain_height_func is None:
            return 1.0  # No terrain function, assume good
        
        score = 1.0
        
        # Check terrain slope
        if self.terrain_slope_func:
            slope = abs(self.terrain_slope_func(position.x, position.y))
            max_slope = 30.0  # degrees
            if slope > max_slope:
                return 0.0  # Too steep
            score *= (1.0 - slope / max_slope)
        
        # Check terrain stability
        if self.terrain_stability_func:
            stability = self.terrain_stability_func(position.x, position.y)
            score *= stability
        
        return score
    
    def generate_footstep_candidates(self, current_pos: Point3D, 
                                   direction: Point2D,
                                   num_candidates: int = 8) -> List[Point3D]:
        """
        Generate candidate footstep positions
        
        Args:
            current_pos: Current foot position
            direction: Desired movement direction (2D)
            num_candidates: Number of candidates to generate
            
        Returns:
            List of candidate footstep positions
        """
        candidates = []
        
        # Normalize direction
        dir_length = np.sqrt(direction.x**2 + direction.y**2)
        if dir_length > 0:
            direction = Point2D(direction.x / dir_length, direction.y / dir_length)
        else:
            direction = Point2D(1.0, 0.0)  # Default forward
        
        # Generate candidates in arc around desired direction
        for i in range(num_candidates):
            # Angle offset from desired direction
            angle_offset = (i - num_candidates // 2) * (60.0 / num_candidates)  # ±30 degree spread
            angle_rad = np.radians(angle_offset)
            
            # Rotate direction vector
            rotated_x = direction.x * np.cos(angle_rad) - direction.y * np.sin(angle_rad)
            rotated_y = direction.x * np.sin(angle_rad) + direction.y * np.cos(angle_rad)
            
            # Scale by step length (with some variation)
            step_scale = 0.7 + 0.3 * (i / max(1, num_candidates - 1))  # 0.7 to 1.0
            step_distance = self.step_length * step_scale
            
            # Calculate candidate position
            candidate_x = current_pos.x + rotated_x * step_distance
            candidate_y = current_pos.y + rotated_y * step_distance
            
            # Get terrain height
            if self.terrain_height_func:
                candidate_z = self.terrain_height_func(candidate_x, candidate_y)
            else:
                candidate_z = current_pos.z
            
            candidates.append(Point3D(candidate_x, candidate_y, candidate_z))
        
        return candidates
    
    def plan_footsteps(self, start_pos: Point3D, goal_pos: Point3D,
                      max_steps: int = 20) -> List[Point3D]:
        """
        Plan sequence of footsteps from start to goal
        
        Args:
            start_pos: Starting foot position
            goal_pos: Goal foot position
            max_steps: Maximum number of steps to plan
            
        Returns:
            List of footstep positions
        """
        footsteps = [start_pos]
        current_pos = start_pos
        
        for step in range(max_steps):
            # Calculate direction to goal
            goal_dir = Point2D(goal_pos.x - current_pos.x, goal_pos.y - current_pos.y)
            remaining_distance = np.sqrt(goal_dir.x**2 + goal_dir.y**2)
            
            # If close to goal, step directly to it
            if remaining_distance < self.step_length * 1.2:
                if self.evaluate_foothold(goal_pos.to_2d()) > 0.5:
                    footsteps.append(goal_pos)
                break
            
            # Generate and evaluate candidates
            candidates = self.generate_footstep_candidates(current_pos, goal_dir)
            
            best_candidate = None
            best_score = -1.0
            
            for candidate in candidates:
                # Check if step is feasible
                step_distance = current_pos.distance_to(candidate)
                step_height_diff = abs(candidate.z - current_pos.z)
                
                if (step_distance > self.step_length or 
                    step_height_diff > self.step_height):
                    continue
                
                # Evaluate foothold quality
                foothold_score = self.evaluate_foothold(candidate.to_2d())
                
                # Bias toward goal direction
                goal_distance = candidate.distance_to(goal_pos)
                progress_score = 1.0 / (1.0 + goal_distance)
                
                total_score = foothold_score * 0.7 + progress_score * 0.3
                
                if total_score > best_score:
                    best_score = total_score
                    best_candidate = candidate
            
            if best_candidate is None:
                logger.warning(f"No valid footstep found at step {step}")
                break
            
            footsteps.append(best_candidate)
            current_pos = best_candidate
        
        logger.info(f"Planned {len(footsteps)} footsteps")
        return footsteps
class NavigationState(Enum):
    """Navigation system states"""
    IDLE = "idle"
    PLANNING = "planning"
    EXECUTING = "executing"
    REPLANNING = "replanning"
    GOAL_REACHED = "goal_reached"
    ERROR = "error"


@dataclass
class NavigationCommand:
    """High-level navigation command"""
    goal_position: Point2D
    max_velocity: float = 0.5
    tolerance: float = 0.1
    timeout: float = 30.0
    priority: int = 1
    
    def __post_init__(self):
        self.timestamp = time.time()


@dataclass
class NavigationStatus:
    """Navigation system status"""
    state: NavigationState
    current_position: Point2D
    goal_position: Optional[Point2D]
    progress: float  # 0.0 to 1.0
    error_message: Optional[str] = None
    estimated_time_remaining: Optional[float] = None
    
    def __post_init__(self):
        self.timestamp = time.time()


class HierarchicalController:
    """
    Hierarchical control architecture integrating all planning levels
    
    Three-tier architecture:
    1. High-level: Body path and navigation planning
    2. Mid-level: Footstep and gait planning  
    3. Low-level: Motion control and joint coordination
    """
    
    def __init__(self, occupancy_grid: OccupancyGrid):
        """
        Initialize hierarchical controller
        
        Args:
            occupancy_grid: Environment representation
        """
        self.grid = occupancy_grid
        
        # Planning components
        self.path_planner = AStarPlanner(occupancy_grid)
        
        # PRM workspace bounds (foot reachability)
        foot_workspace = (Point3D(-0.3, -0.3, -0.3), Point3D(0.3, 0.3, 0.1))
        self.foot_planner = PRMPlanner(foot_workspace, max_connection_distance=0.15)
        self.footstep_planner = FootstepPlanner(step_length=0.2, step_height=0.08)
        
        # State management
        self.state = NavigationState.IDLE
        self.current_command: Optional[NavigationCommand] = None
        self.current_path: Optional[List[Point2D]] = None
        self.current_footsteps: Dict[str, List[Point3D]] = {}
        
        # Position tracking
        self.current_position = Point2D(0.0, 0.0)
        self.goal_position: Optional[Point2D] = None
        
        # Control parameters
        self.replan_threshold = 0.5  # meters
        self.goal_tolerance = 0.1    # meters
        self.control_frequency = 50  # Hz
        
        # Performance monitoring
        self.planning_stats = {
            'total_plans': 0,
            'successful_plans': 0,
            'planning_time_avg': 0.0,
            'execution_time_avg': 0.0
        }
        
        # Integration with motion controller (will be set externally)
        self.motion_controller = None
        
        logger.info("Hierarchical controller initialized")
    
    def set_motion_controller(self, motion_controller):
        """Set reference to motion controller for low-level integration"""
        self.motion_controller = motion_controller
        logger.info("Motion controller integrated")
    
    def execute_navigation_command(self, command: NavigationCommand) -> bool:
        """
        Execute high-level navigation command
        
        Args:
            command: Navigation command to execute
            
        Returns:
            True if command accepted, False if rejected
        """
        if self.state == NavigationState.EXECUTING and self.current_command:
            if command.priority <= self.current_command.priority:
                logger.warning("Rejecting lower priority navigation command")
                return False
            else:
                logger.info("Preempting current navigation with higher priority command")
        
        self.current_command = command
        self.goal_position = command.goal_position
        self.state = NavigationState.PLANNING
        
        logger.info(f"Accepted navigation command to {command.goal_position}")
        
        # Start planning process
        success = self._plan_navigation()
        
        if success:
            self.state = NavigationState.EXECUTING
            self._start_execution()
        else:
            self.state = NavigationState.ERROR
        
        return success
    
    def _plan_navigation(self) -> bool:
        """Plan complete navigation solution"""
        if not self.current_command:
            return False
        
        start_time = time.time()
        self.planning_stats['total_plans'] += 1
        
        # High-level path planning
        logger.info("Planning high-level path...")
        path = self.path_planner.plan_path(self.current_position, self.goal_position)
        
        if not path:
            logger.error("High-level path planning failed")
            return False
        
        # Smooth the path
        self.current_path = self.path_planner.smooth_path(path)
        
        # Plan footsteps for each leg
        logger.info("Planning footstep sequences...")
        self._plan_footstep_sequences()
        
        # Update statistics
        planning_time = time.time() - start_time
        self.planning_stats['planning_time_avg'] = (
            (self.planning_stats['planning_time_avg'] * (self.planning_stats['total_plans'] - 1) + 
             planning_time) / self.planning_stats['total_plans']
        )
        
        self.planning_stats['successful_plans'] += 1
        
        logger.info(f"Navigation planning completed in {planning_time:.3f}s")
        return True
    
    def _plan_footstep_sequences(self):
        """Plan footstep sequences for all legs"""
        if not self.current_path:
            return
        
        # Leg names for hexapod
        leg_names = ['L1', 'R1', 'L2', 'R2', 'L3', 'R3']
        
        # Initialize footstep sequences
        self.current_footsteps = {leg: [] for leg in leg_names}
        
        # Plan footsteps along path for each leg
        for i, leg_name in enumerate(leg_names):
            # Get initial foot position (simplified - would use FK in real implementation)
            initial_pos = self._get_initial_foot_position(leg_name)
            
            # Plan footsteps along body path
            footsteps = []
            for j, waypoint in enumerate(self.current_path[::3]):  # Sample every 3rd waypoint
                # Calculate foot position relative to body
                foot_target = self._calculate_foot_target(leg_name, waypoint)
                
                # Plan footstep if needed
                if not footsteps or footsteps[-1].distance_to(foot_target) > 0.15:
                    planned_steps = self.footstep_planner.plan_footsteps(
                        footsteps[-1] if footsteps else initial_pos,
                        foot_target,
                        max_steps=3
                    )
                    if planned_steps and len(planned_steps) > 1:
                        footsteps.extend(planned_steps[1:])  # Exclude current position
            
            self.current_footsteps[leg_name] = footsteps
        
        logger.info(f"Planned footsteps for {len(leg_names)} legs")
    
    def _get_initial_foot_position(self, leg_name: str) -> Point3D:
        """Get initial foot position for leg (simplified)"""
        # This would use forward kinematics in real implementation
        leg_positions = {
            'L1': Point3D(0.15, 0.13, -0.2),
            'R1': Point3D(0.15, -0.13, -0.2),
            'L2': Point3D(0.0, 0.15, -0.2),
            'R2': Point3D(0.0, -0.15, -0.2),
            'L3': Point3D(-0.15, 0.13, -0.2),
            'R3': Point3D(-0.15, -0.13, -0.2)
        }
        return leg_positions.get(leg_name, Point3D(0.0, 0.0, -0.2))
    
    def _calculate_foot_target(self, leg_name: str, body_position: Point2D) -> Point3D:
        """Calculate target foot position for given body position"""
        # Get relative foot position
        initial_foot = self._get_initial_foot_position(leg_name)
        
        # Transform to world coordinates (simplified)
        target_x = body_position.x + initial_foot.x
        target_y = body_position.y + initial_foot.y
        target_z = initial_foot.z  # Assume flat terrain for now
        
        return Point3D(target_x, target_y, target_z)
    
    def _start_execution(self):
        """Start navigation execution"""
        if not self.motion_controller:
            logger.warning("No motion controller available for execution")
            return
        
        logger.info("Starting navigation execution")
        # This would integrate with motion controller to execute planned motion
        # For now, we'll simulate execution
    
    def update(self, current_position: Point2D) -> NavigationStatus:
        """
        Update navigation system (call at control frequency)
        
        Args:
            current_position: Current robot position
            
        Returns:
            Navigation status
        """
        self.current_position = current_position
        
        # Check if goal reached
        if (self.goal_position and 
            self.current_position.distance_to(self.goal_position) <= self.goal_tolerance):
            
            self.state = NavigationState.GOAL_REACHED
            logger.info("Navigation goal reached")
        
        # Check if replanning needed
        elif (self.state == NavigationState.EXECUTING and 
              self.current_path and 
              self._needs_replanning()):
            
            logger.info("Replanning required")
            self.state = NavigationState.REPLANNING
            self._plan_navigation()
            self.state = NavigationState.EXECUTING
        
        # Calculate progress
        progress = self._calculate_progress()
        
        return NavigationStatus(
            state=self.state,
            current_position=self.current_position,
            goal_position=self.goal_position,
            progress=progress,
            estimated_time_remaining=self._estimate_time_remaining()
        )
    
    def _needs_replanning(self) -> bool:
        """Check if replanning is needed"""
        if not self.current_path:
            return True
        
        # Check if significantly off path
        min_distance_to_path = float('inf')
        for waypoint in self.current_path:
            distance = self.current_position.distance_to(waypoint)
            min_distance_to_path = min(min_distance_to_path, distance)
        
        return min_distance_to_path > self.replan_threshold
    
    def _calculate_progress(self) -> float:
        """Calculate navigation progress [0-1]"""
        if not self.goal_position:
            return 0.0
        
        if not self.current_path:
            return 0.0
        
        start_pos = self.current_path[0]
        total_distance = start_pos.distance_to(self.goal_position)
        
        if total_distance == 0:
            return 1.0
        
        remaining_distance = self.current_position.distance_to(self.goal_position)
        progress = 1.0 - (remaining_distance / total_distance)
        
        return max(0.0, min(1.0, progress))
    
    def _estimate_time_remaining(self) -> Optional[float]:
        """Estimate time remaining for navigation"""
        if not self.goal_position or not self.current_command:
            return None
        
        remaining_distance = self.current_position.distance_to(self.goal_position)
        estimated_time = remaining_distance / self.current_command.max_velocity
        
        return estimated_time
    
    def stop_navigation(self):
        """Stop current navigation"""
        self.state = NavigationState.IDLE
        self.current_command = None
        self.current_path = None
        self.current_footsteps.clear()
        logger.info("Navigation stopped")
    
    def get_planning_statistics(self) -> Dict:
        """Get planning performance statistics"""
        return self.planning_stats.copy()
    
    def set_terrain_functions(self, terrain_funcs: Dict[str, Callable]):
        """Set terrain analysis functions for footstep planning"""
        if 'height' in terrain_funcs:
            height_func = terrain_funcs['height']
            slope_func = terrain_funcs.get('slope')
            stability_func = terrain_funcs.get('stability')
            
            self.footstep_planner.set_terrain_functions(
                height_func, slope_func, stability_func
            )
            
            logger.info("Terrain functions configured for footstep planning")
    
    def set_collision_checkers(self, collision_funcs: Dict[str, Callable]):
        """Set collision checking functions for PRM planning"""
        if 'point' in collision_funcs and 'line' in collision_funcs:
            self.foot_planner.set_collision_checkers(
                collision_funcs['point'], 
                collision_funcs['line']
            )
            
            # Build initial roadmap
            self.foot_planner.build_roadmap(300)
            
            logger.info("Collision checkers configured for PRM planning")


if __name__ == "__main__":
    # Example usage and testing
    print("Testing Path Planning System...")
    
    # Test A* Planner
    print("\n=== A* Path Planning ===")
    grid = OccupancyGrid(width=100, height=100, resolution=0.1)
    
    # Add some obstacles
    grid.add_obstacle(Point2D(3.0, 3.0), 0.5)
    grid.add_obstacle(Point2D(6.0, 6.0), 0.8)
    grid.add_rectangle_obstacle(Point2D(1.0, 7.0), Point2D(3.0, 9.0))
    
    # Create planner
    planner = AStarPlanner(grid, HeuristicType.EUCLIDEAN)
    
    # Plan path
    start = Point2D(0.5, 0.5)
    goal = Point2D(8.0, 8.0)
    
    path = planner.plan_path(start, goal)
    
    if path:
        print(f"A* path found with {len(path)} waypoints")
        print(f"Planning statistics: {planner.get_search_statistics()}")
        
        # Smooth the path
        smoothed = planner.smooth_path(path)
        print(f"Smoothed path has {len(smoothed)} waypoints")
        
        # Apply Bezier smoothing
        bezier_smoothed = PathSmoother.bezier_smooth(smoothed, 100)
        print(f"Bezier smoothed path has {len(bezier_smoothed)} waypoints")
    else:
        print("No A* path found!")
    
    # Test PRM Planner
    print("\n=== PRM Path Planning ===")
    workspace_bounds = (Point3D(-1.0, -1.0, -0.5), Point3D(1.0, 1.0, 0.5))
    prm = PRMPlanner(workspace_bounds, max_connection_distance=0.3)
    
    # Simple collision checker for testing
    def simple_collision_check(point: Point3D) -> bool:
        # Avoid center of workspace
        return (abs(point.x) < 0.2 and abs(point.y) < 0.2)
    
    def simple_line_collision_check(start: Point3D, end: Point3D) -> bool:
        # Check midpoint for simplicity
        mid = Point3D((start.x + end.x)/2, (start.y + end.y)/2, (start.z + end.z)/2)
        return simple_collision_check(mid)
    
    prm.set_collision_checkers(simple_collision_check, simple_line_collision_check)
    prm.build_roadmap(200)
    
    start_3d = Point3D(-0.8, -0.8, 0.0)
    goal_3d = Point3D(0.8, 0.8, 0.0)
    
    prm_path = prm.plan_path(start_3d, goal_3d)
    if prm_path:
        print(f"PRM path found with {len(prm_path)} waypoints")
    else:
        print("No PRM path found!")
    
    # Test Footstep Planner
    print("\n=== Footstep Planning ===")
    footstep_planner = FootstepPlanner(step_length=0.2, step_height=0.1)
    
    # Simple terrain function
    def terrain_height(x: float, y: float) -> float:
        return 0.1 * np.sin(x) * np.cos(y)  # Wavy terrain
    
    def terrain_slope(x: float, y: float) -> float:
        # Gradient of terrain height function
        dx = 0.1 * np.cos(x) * np.cos(y)
        dy = -0.1 * np.sin(x) * np.sin(y)
        return np.degrees(np.arctan(np.sqrt(dx**2 + dy**2)))
    
    footstep_planner.set_terrain_functions(terrain_height, terrain_slope)
    
    start_foot = Point3D(0.0, 0.0, 0.0)
    goal_foot = Point3D(1.0, 1.0, 0.1)
    
    footsteps = footstep_planner.plan_footsteps(start_foot, goal_foot, max_steps=10)
    print(f"Footstep plan with {len(footsteps)} steps")
    
    # Test Hierarchical Controller
    print("\n=== Hierarchical Navigation Controller ===")
    nav_controller = HierarchicalController(grid)
    
    # Set terrain and collision functions
    nav_controller.set_terrain_functions({
        'height': terrain_height,
        'slope': terrain_slope
    })
    
    nav_controller.set_collision_checkers({
        'point': simple_collision_check,
        'line': simple_line_collision_check
    })
    
    # Execute navigation command
    command = NavigationCommand(
        goal_position=Point2D(7.0, 7.0),
        max_velocity=0.5,
        tolerance=0.2
    )
    
    success = nav_controller.execute_navigation_command(command)
    print(f"Navigation command {'accepted' if success else 'rejected'}")
    
    if success:
        # Simulate navigation updates
        current_pos = Point2D(0.5, 0.5)
        for i in range(5):
            # Simulate movement toward goal
            current_pos.x += 0.1
            current_pos.y += 0.1
            
            status = nav_controller.update(current_pos)
            print(f"Step {i+1}: State={status.state.value}, Progress={status.progress:.2f}")
    
    print(f"Planning statistics: {nav_controller.get_planning_statistics()}")
    
    print("\nPath planning system test completed!")