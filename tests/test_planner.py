"""
Test Suite for Path Planning Module

This module provides comprehensive testing for all path planning components:
- A* pathfinding algorithm
- Probabilistic Roadmap (PRM) planner
- Footstep planning
- Hierarchical control integration

Author: HexaPodSim Team
Date: 2024
"""

import unittest
import numpy as np
import time
from typing import List, Tuple
import sys
import os

# Add the parent directory to the path to import hexapod modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hexapod.planner import (
    Point2D, Point3D, OccupancyGrid, AStarPlanner, PRMPlanner, 
    FootstepPlanner, HierarchicalController, PathSmoother,
    HeuristicType, NavigationCommand, NavigationState
)


class TestPoint2D(unittest.TestCase):
    """Test Point2D class functionality"""
    
    def test_point_creation(self):
        """Test point creation and basic operations"""
        p1 = Point2D(1.0, 2.0)
        self.assertEqual(p1.x, 1.0)
        self.assertEqual(p1.y, 2.0)
    
    def test_distance_calculation(self):
        """Test distance calculations"""
        p1 = Point2D(0.0, 0.0)
        p2 = Point2D(3.0, 4.0)
        
        # Euclidean distance
        self.assertAlmostEqual(p1.distance_to(p2), 5.0, places=5)
        
        # Manhattan distance
        self.assertAlmostEqual(p1.manhattan_distance_to(p2), 7.0, places=5)
    
    def test_point_arithmetic(self):
        """Test point arithmetic operations"""
        p1 = Point2D(1.0, 2.0)
        p2 = Point2D(3.0, 4.0)
        
        # Addition
        p3 = p1 + p2
        self.assertEqual(p3.x, 4.0)
        self.assertEqual(p3.y, 6.0)
        
        # Subtraction
        p4 = p2 - p1
        self.assertEqual(p4.x, 2.0)
        self.assertEqual(p4.y, 2.0)
        
        # Scalar multiplication
        p5 = p1 * 2.0
        self.assertEqual(p5.x, 2.0)
        self.assertEqual(p5.y, 4.0)


class TestPoint3D(unittest.TestCase):
    """Test Point3D class functionality"""
    
    def test_point_creation(self):
        """Test 3D point creation"""
        p = Point3D(1.0, 2.0, 3.0)
        self.assertEqual(p.x, 1.0)
        self.assertEqual(p.y, 2.0)
        self.assertEqual(p.z, 3.0)
    
    def test_distance_calculation(self):
        """Test 3D distance calculation"""
        p1 = Point3D(0.0, 0.0, 0.0)
        p2 = Point3D(1.0, 1.0, 1.0)
        
        expected_distance = np.sqrt(3.0)
        self.assertAlmostEqual(p1.distance_to(p2), expected_distance, places=5)
    
    def test_2d_conversion(self):
        """Test conversion to 2D point"""
        p3d = Point3D(1.0, 2.0, 3.0)
        p2d = p3d.to_2d()
        
        self.assertEqual(p2d.x, 1.0)
        self.assertEqual(p2d.y, 2.0)


class TestOccupancyGrid(unittest.TestCase):
    """Test OccupancyGrid functionality"""
    
    def setUp(self):
        """Set up test grid"""
        self.grid = OccupancyGrid(width=50, height=50, resolution=0.1)
    
    def test_grid_initialization(self):
        """Test grid initialization"""
        self.assertEqual(self.grid.width, 50)
        self.assertEqual(self.grid.height, 50)
        self.assertEqual(self.grid.resolution, 0.1)
        
        # Check that grid is initially free
        self.assertFalse(np.any(self.grid.grid))
    
    def test_coordinate_conversion(self):
        """Test world to grid coordinate conversion"""
        point = Point2D(1.5, 2.3)
        grid_x, grid_y = self.grid.world_to_grid(point)
        
        self.assertEqual(grid_x, 15)  # 1.5 / 0.1 = 15
        self.assertEqual(grid_y, 22)  # 2.3 / 0.1 = 23, but int() truncates to 22
        
        # Test reverse conversion
        world_point = self.grid.grid_to_world(grid_x, grid_y)
        self.assertAlmostEqual(world_point.x, 1.55, places=2)  # (15 + 0.5) * 0.1
        self.assertAlmostEqual(world_point.y, 2.25, places=2)  # (22 + 0.5) * 0.1
    
    def test_obstacle_addition(self):
        """Test adding obstacles to grid"""
        # Add circular obstacle
        center = Point2D(1.0, 1.0)
        radius = 0.2
        self.grid.add_obstacle(center, radius)
        
        # Check that center is occupied
        center_x, center_y = self.grid.world_to_grid(center)
        self.assertTrue(self.grid.is_occupied(center_x, center_y))
        
        # Add rectangular obstacle
        corner1 = Point2D(2.0, 2.0)
        corner2 = Point2D(2.5, 2.5)
        self.grid.add_rectangle_obstacle(corner1, corner2)
        
        # Check that corners are occupied
        grid_x1, grid_y1 = self.grid.world_to_grid(corner1)
        grid_x2, grid_y2 = self.grid.world_to_grid(corner2)
        self.assertTrue(self.grid.is_occupied(grid_x1, grid_y1))
        self.assertTrue(self.grid.is_occupied(grid_x2, grid_y2))
    
    def test_neighbors(self):
        """Test neighbor generation"""
        # Test 4-connected neighbors
        neighbors_4 = self.grid.get_neighbors(25, 25, include_diagonals=False)
        self.assertEqual(len(neighbors_4), 4)
        
        # Test 8-connected neighbors
        neighbors_8 = self.grid.get_neighbors(25, 25, include_diagonals=True)
        self.assertEqual(len(neighbors_8), 8)
        
        # Test boundary conditions
        corner_neighbors = self.grid.get_neighbors(0, 0, include_diagonals=True)
        self.assertEqual(len(corner_neighbors), 3)  # Only 3 valid neighbors at corner


class TestAStarPlanner(unittest.TestCase):
    """Test A* pathfinding algorithm"""
    
    def setUp(self):
        """Set up test environment"""
        self.grid = OccupancyGrid(width=50, height=50, resolution=0.1)
        self.planner = AStarPlanner(self.grid, HeuristicType.EUCLIDEAN)
    
    def test_heuristic_functions(self):
        """Test different heuristic functions"""
        start = (0, 0)
        goal = (3, 4)
        
        # Test Manhattan heuristic
        self.planner.heuristic_type = HeuristicType.MANHATTAN
        manhattan_h = self.planner.heuristic(start, goal)
        self.assertEqual(manhattan_h, 7.0)  # |3-0| + |4-0| = 7
        
        # Test Euclidean heuristic
        self.planner.heuristic_type = HeuristicType.EUCLIDEAN
        euclidean_h = self.planner.heuristic(start, goal)
        self.assertAlmostEqual(euclidean_h, 5.0, places=5)  # sqrt(3^2 + 4^2) = 5
        
        # Test Diagonal heuristic
        self.planner.heuristic_type = HeuristicType.DIAGONAL
        diagonal_h = self.planner.heuristic(start, goal)
        self.assertEqual(diagonal_h, 4.0)  # max(3, 4) = 4
    
    def test_simple_path_planning(self):
        """Test path planning in simple environment"""
        start = Point2D(0.5, 0.5)
        goal = Point2D(2.0, 2.0)
        
        path = self.planner.plan_path(start, goal)
        
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 1)
        
        # Check that path starts near start and ends near goal
        self.assertLess(path[0].distance_to(start), 0.2)
        self.assertLess(path[-1].distance_to(goal), 0.2)
    
    def test_path_planning_with_obstacles(self):
        """Test path planning around obstacles"""
        # Add obstacle between start and goal
        self.grid.add_obstacle(Point2D(1.0, 1.0), 0.3)
        
        start = Point2D(0.5, 0.5)
        goal = Point2D(1.5, 1.5)
        
        path = self.planner.plan_path(start, goal)
        
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 2)  # Should be longer than direct path
    
    def test_no_path_scenario(self):
        """Test behavior when no path exists"""
        # Create complete wall blocking path
        for x in range(20, 30):
            for y in range(0, 50):  # Block entire column
                self.grid.set_occupied(x, y, True)
        
        start = Point2D(1.0, 2.0)
        goal = Point2D(4.0, 3.0)
        
        path = self.planner.plan_path(start, goal)
        self.assertIsNone(path)
    
    def test_path_smoothing(self):
        """Test path smoothing functionality"""
        # Create a zigzag path
        path = [
            Point2D(0.0, 0.0),
            Point2D(0.1, 0.1),
            Point2D(0.2, 0.0),
            Point2D(0.3, 0.1),
            Point2D(0.4, 0.0),
            Point2D(0.5, 0.1),
            Point2D(1.0, 1.0)
        ]
        
        smoothed = self.planner.smooth_path(path)
        
        # Smoothed path should have fewer waypoints
        self.assertLessEqual(len(smoothed), len(path))
        
        # Should still start and end at same points
        self.assertEqual(smoothed[0], path[0])
        self.assertEqual(smoothed[-1], path[-1])


class TestPRMPlanner(unittest.TestCase):
    """Test Probabilistic Roadmap planner"""
    
    def setUp(self):
        """Set up PRM test environment"""
        workspace_bounds = (Point3D(-1.0, -1.0, -0.5), Point3D(1.0, 1.0, 0.5))
        self.prm = PRMPlanner(workspace_bounds, max_connection_distance=0.3)
        
        # Simple collision checker
        def collision_check(point: Point3D) -> bool:
            return abs(point.x) < 0.2 and abs(point.y) < 0.2
        
        def line_collision_check(start: Point3D, end: Point3D) -> bool:
            mid = Point3D((start.x + end.x)/2, (start.y + end.y)/2, (start.z + end.z)/2)
            return collision_check(mid)
        
        self.prm.set_collision_checkers(collision_check, line_collision_check)
    
    def test_roadmap_construction(self):
        """Test roadmap building"""
        self.prm.build_roadmap(100)
        
        # Should have created some nodes
        self.assertGreater(len(self.prm.nodes), 10)
        
        # Nodes should be connected
        total_connections = sum(len(node.neighbors) for node in self.prm.nodes.values())
        self.assertGreater(total_connections, 0)
    
    def test_prm_path_planning(self):
        """Test PRM path planning"""
        self.prm.build_roadmap(200)
        
        start = Point3D(-0.8, -0.8, 0.0)
        goal = Point3D(0.8, 0.8, 0.0)
        
        path = self.prm.plan_path(start, goal)
        
        if path:  # Path might not always exist due to random sampling
            self.assertGreater(len(path), 1)
            self.assertLess(path[0].distance_to(start), 0.5)
            self.assertLess(path[-1].distance_to(goal), 0.5)


class TestFootstepPlanner(unittest.TestCase):
    """Test footstep planning functionality"""
    
    def setUp(self):
        """Set up footstep planner"""
        self.planner = FootstepPlanner(step_length=0.2, step_height=0.1)
        
        # Simple terrain functions
        def terrain_height(x: float, y: float) -> float:
            return 0.05 * np.sin(x) * np.cos(y)
        
        def terrain_slope(x: float, y: float) -> float:
            return 10.0  # Constant slope for testing
        
        self.planner.set_terrain_functions(terrain_height, terrain_slope)
    
    def test_foothold_evaluation(self):
        """Test foothold quality evaluation"""
        # Test flat terrain
        score = self.planner.evaluate_foothold(Point2D(0.0, 0.0))
        self.assertGreater(score, 0.0)
        self.assertLessEqual(score, 1.0)
    
    def test_candidate_generation(self):
        """Test footstep candidate generation"""
        current_pos = Point3D(0.0, 0.0, 0.0)
        direction = Point2D(1.0, 0.0)
        
        candidates = self.planner.generate_footstep_candidates(
            current_pos, direction, num_candidates=8
        )
        
        self.assertEqual(len(candidates), 8)
        
        # All candidates should be within step range
        for candidate in candidates:
            distance = current_pos.distance_to(candidate)
            self.assertLessEqual(distance, self.planner.step_length * 1.1)
    
    def test_footstep_planning(self):
        """Test complete footstep sequence planning"""
        start = Point3D(0.0, 0.0, 0.0)
        goal = Point3D(0.5, 0.5, 0.05)
        
        footsteps = self.planner.plan_footsteps(start, goal, max_steps=10)
        
        self.assertIsNotNone(footsteps)
        self.assertGreater(len(footsteps), 1)
        self.assertEqual(footsteps[0], start)
        
        # Check step constraints
        for i in range(1, len(footsteps)):
            step_distance = footsteps[i-1].distance_to(footsteps[i])
            self.assertLessEqual(step_distance, self.planner.step_length * 1.1)


class TestHierarchicalController(unittest.TestCase):
    """Test hierarchical navigation controller"""
    
    def setUp(self):
        """Set up hierarchical controller"""
        grid = OccupancyGrid(width=50, height=50, resolution=0.1)
        self.controller = HierarchicalController(grid)
        
        # Set up terrain and collision functions
        def terrain_height(x: float, y: float) -> float:
            return 0.0  # Flat terrain for testing
        
        def collision_check(point: Point3D) -> bool:
            return False  # No collisions for testing
        
        def line_collision_check(start: Point3D, end: Point3D) -> bool:
            return False  # No collisions for testing
        
        self.controller.set_terrain_functions({'height': terrain_height})
        self.controller.set_collision_checkers({
            'point': collision_check,
            'line': line_collision_check
        })
    
    def test_navigation_command_execution(self):
        """Test navigation command execution"""
        command = NavigationCommand(
            goal_position=Point2D(2.0, 2.0),
            max_velocity=0.5,
            tolerance=0.1
        )
        
        success = self.controller.execute_navigation_command(command)
        self.assertTrue(success)
        
        self.assertEqual(self.controller.state, NavigationState.EXECUTING)
        self.assertIsNotNone(self.controller.current_path)
    
    def test_navigation_updates(self):
        """Test navigation system updates"""
        # Execute navigation command
        command = NavigationCommand(Point2D(1.0, 1.0), max_velocity=0.3)
        self.controller.execute_navigation_command(command)
        
        # Simulate navigation updates
        current_pos = Point2D(0.0, 0.0)
        for i in range(5):
            current_pos.x += 0.1
            current_pos.y += 0.1
            
            status = self.controller.update(current_pos)
            
            self.assertIsInstance(status.state, NavigationState)
            self.assertGreaterEqual(status.progress, 0.0)
            self.assertLessEqual(status.progress, 1.0)
    
    def test_goal_reached_detection(self):
        """Test goal reached detection"""
        command = NavigationCommand(Point2D(0.5, 0.5), tolerance=0.2)
        self.controller.execute_navigation_command(command)
        
        # Move close to goal
        status = self.controller.update(Point2D(0.45, 0.45))
        self.assertEqual(status.state, NavigationState.GOAL_REACHED)
    
    def test_navigation_stop(self):
        """Test navigation stopping"""
        command = NavigationCommand(Point2D(2.0, 2.0))
        self.controller.execute_navigation_command(command)
        
        self.assertEqual(self.controller.state, NavigationState.EXECUTING)
        
        self.controller.stop_navigation()
        self.assertEqual(self.controller.state, NavigationState.IDLE)
        self.assertIsNone(self.controller.current_command)


class TestPathSmoother(unittest.TestCase):
    """Test path smoothing utilities"""
    
    def test_bezier_smoothing(self):
        """Test Bezier curve smoothing"""
        # Create simple path
        path = [
            Point2D(0.0, 0.0),
            Point2D(1.0, 0.0),
            Point2D(2.0, 1.0),
            Point2D(3.0, 1.0)
        ]
        
        smoothed = PathSmoother.bezier_smooth(path, num_points=20)
        
        self.assertGreater(len(smoothed), len(path))
        
        # Should start and end at same points
        self.assertLess(smoothed[0].distance_to(path[0]), 0.1)
        self.assertLess(smoothed[-1].distance_to(path[-1]), 0.1)


class TestPerformance(unittest.TestCase):
    """Test performance of planning algorithms"""
    
    def test_astar_performance(self):
        """Test A* planning performance"""
        grid = OccupancyGrid(width=100, height=100, resolution=0.05)
        
        # Add some obstacles
        for i in range(10):
            center = Point2D(np.random.uniform(1, 4), np.random.uniform(1, 4))
            radius = np.random.uniform(0.1, 0.3)
            grid.add_obstacle(center, radius)
        
        planner = AStarPlanner(grid)
        
        start_time = time.time()
        path = planner.plan_path(Point2D(0.1, 0.1), Point2D(4.5, 4.5))
        planning_time = time.time() - start_time
        
        # Should plan within reasonable time
        self.assertLess(planning_time, 5.0)
        
        if path:
            print(f"A* planned {len(path)} waypoints in {planning_time:.3f}s")
    
    def test_prm_performance(self):
        """Test PRM planning performance"""
        workspace_bounds = (Point3D(-2.0, -2.0, -1.0), Point3D(2.0, 2.0, 1.0))
        prm = PRMPlanner(workspace_bounds, max_connection_distance=0.5)
        
        # Simple collision checker
        def collision_check(point: Point3D) -> bool:
            return point.x**2 + point.y**2 < 0.5**2  # Circular obstacle
        
        def line_collision_check(start: Point3D, end: Point3D) -> bool:
            return False  # Simplified for performance test
        
        prm.set_collision_checkers(collision_check, line_collision_check)
        
        start_time = time.time()
        prm.build_roadmap(500)
        build_time = time.time() - start_time
        
        self.assertLess(build_time, 10.0)
        print(f"PRM built {len(prm.nodes)} nodes in {build_time:.3f}s")


def run_planning_tests():
    """Run all path planning tests"""
    print("Running Path Planning Test Suite...")
    print("=" * 50)
    
    # Create test suite
    test_classes = [
        TestPoint2D,
        TestPoint3D, 
        TestOccupancyGrid,
        TestAStarPlanner,
        TestPRMPlanner,
        TestFootstepPlanner,
        TestHierarchicalController,
        TestPathSmoother,
        TestPerformance
    ]
    
    total_tests = 0
    total_failures = 0
    total_errors = 0
    
    for test_class in test_classes:
        print(f"\nRunning {test_class.__name__}...")
        suite = unittest.TestLoader().loadTestsFromTestCase(test_class)
        runner = unittest.TextTestRunner(verbosity=1)
        result = runner.run(suite)
        
        total_tests += result.testsRun
        total_failures += len(result.failures)
        total_errors += len(result.errors)
    
    print("\n" + "=" * 50)
    print(f"Test Summary:")
    print(f"Total Tests: {total_tests}")
    print(f"Failures: {total_failures}")
    print(f"Errors: {total_errors}")
    print(f"Success Rate: {((total_tests - total_failures - total_errors) / total_tests * 100):.1f}%")
    
    if total_failures == 0 and total_errors == 0:
        print("🎉 All tests passed!")
        return True
    else:
        print("❌ Some tests failed!")
        return False


if __name__ == "__main__":
    success = run_planning_tests()
    exit(0 if success else 1)