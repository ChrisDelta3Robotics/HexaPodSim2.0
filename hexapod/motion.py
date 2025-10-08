"""
Phase 3.4: Motion Control Integration System

This module provides the integration layer that coordinates all Phase 3 components:
- Gait Generation (Phase 3.1)
- PID Controllers (Phase 3.2) 
- Adaptive Control (Phase 3.3)

The motion controller acts as the central coordinator, managing the flow from 
high-level movement commands down to individual joint control signals.

Key Features:
- Unified motion command interface
- Real-time coordination of gait, adaptation, and control
- Smooth transitions between movement modes
- Performance monitoring and optimization
- Safety oversight and emergency handling
- Hardware abstraction for servo control

Author: GitHub Copilot
Date: October 2025
"""

import numpy as np
import time
from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Callable, Any
import logging
from concurrent.futures import ThreadPoolExecutor
import threading

from .kinematics import HexapodKinematics
from .gait import GaitGenerator, GaitType, GaitParameters
from .controller import JointControllerSystem, PIDGains
from .adaptive import AdaptiveGaitController, TerrainType, StabilityLevel, AdaptationMode

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MotionMode(Enum):
    """Motion control modes"""
    IDLE = "idle"                    # Stationary, servos active
    MANUAL = "manual"                # Direct joint control
    WALKING = "walking"              # Normal gait-based locomotion
    ADAPTIVE = "adaptive"            # Adaptive gait control
    EMERGENCY_STOP = "emergency_stop" # Emergency stop mode
    CALIBRATION = "calibration"      # Joint calibration mode


class MotionState(Enum):
    """Current motion system state"""
    INITIALIZING = "initializing"
    READY = "ready"
    ACTIVE = "active"
    STOPPING = "stopping"
    ERROR = "error"
    EMERGENCY = "emergency"


@dataclass
class MotionCommand:
    """High-level motion command"""
    # Movement parameters
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))  # [vx, vy, vz] m/s
    angular_velocity: float = 0.0    # rad/s around Z-axis
    
    # Gait preferences
    preferred_gait: Optional[GaitType] = None
    adaptation_mode: Optional[AdaptationMode] = None
    
    # Safety parameters
    max_speed: float = 0.3           # m/s
    max_angular_speed: float = 1.0   # rad/s
    
    # Execution parameters
    duration: Optional[float] = None  # seconds (None = continuous)
    priority: int = 1                # 1-10 (10 = highest)
    
    # Metadata
    command_id: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class MotionStatus:
    """Current motion system status"""
    # System state
    mode: MotionMode = MotionMode.IDLE
    state: MotionState = MotionState.INITIALIZING
    
    # Current motion
    current_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    current_angular_velocity: float = 0.0
    
    # Gait information
    current_gait: GaitType = GaitType.TRIPOD
    gait_phase: float = 0.0          # 0-1 current gait cycle position
    support_pattern: List[bool] = field(default_factory=lambda: [True] * 6)
    
    # Adaptation information
    terrain_type: TerrainType = TerrainType.UNKNOWN
    stability_level: StabilityLevel = StabilityLevel.HIGH
    adaptation_active: bool = False
    
    # Control information
    joint_angles: np.ndarray = field(default_factory=lambda: np.zeros(18))
    joint_targets: np.ndarray = field(default_factory=lambda: np.zeros(18))
    joint_errors: np.ndarray = field(default_factory=lambda: np.zeros(18))
    
    # Performance metrics
    update_frequency: float = 0.0    # Hz
    computation_time: float = 0.0    # ms
    stability_margin: float = 0.0    # meters
    
    # Safety status
    emergency_stop_active: bool = False
    safety_violations: List[str] = field(default_factory=list)
    
    # System health
    last_update: float = field(default_factory=time.time)
    uptime: float = 0.0             # seconds


class MotionController:
    """Integrated motion control system"""
    
    def __init__(self, kinematics: HexapodKinematics,
                 control_frequency: float = 100.0,
                 enable_adaptation: bool = True):
        """
        Initialize the integrated motion control system
        
        Args:
            kinematics: Hexapod kinematics system
            control_frequency: Target control loop frequency (Hz)
            enable_adaptation: Whether to enable adaptive gait control
        """
        self.kinematics = kinematics
        self.control_frequency = control_frequency
        self.control_period = 1.0 / control_frequency
        self.enable_adaptation = enable_adaptation
        
        # Initialize subsystems
        self.gait_generator = GaitGenerator(kinematics)
        self.controller_system = JointControllerSystem(kinematics)
        
        if enable_adaptation:
            self.adaptive_controller = AdaptiveGaitController(
                kinematics, self.gait_generator)
            self._setup_adaptation_callbacks()
        else:
            self.adaptive_controller = None
            
        # Motion state
        self.status = MotionStatus()
        self.current_command = MotionCommand()
        self.command_queue: List[MotionCommand] = []
        
        # Control loop management
        self.running = False
        self.control_thread: Optional[threading.Thread] = None
        self.thread_executor = ThreadPoolExecutor(max_workers=2)
        
        # Performance tracking
        self.start_time = time.time()
        self.loop_times: List[float] = []
        self.max_loop_history = 1000
        
        # Safety limits
        self.safety_limits = {
            'max_joint_error': 20.0,      # degrees
            'max_velocity': 0.5,          # m/s
            'min_stability_margin': 0.02, # meters
            'max_computation_time': 10.0  # ms
        }
        
        # Hardware interface callbacks
        self.servo_write_callback: Optional[Callable[[np.ndarray], None]] = None
        self.sensor_read_callback: Optional[Callable[[], Dict[str, Any]]] = None
        
        logger.info("Motion control system initialized")
        
    def _setup_adaptation_callbacks(self):
        """Setup callbacks for adaptive controller"""
        if self.adaptive_controller:
            self.adaptive_controller.set_callbacks(
                stability_fn=self._get_stability_margin,
                speed_fn=self._get_current_speed,
                energy_fn=self._get_energy_usage
            )
            
    def set_hardware_callbacks(self, 
                              servo_write: Callable[[np.ndarray], None],
                              sensor_read: Callable[[], Dict[str, Any]]):
        """Set hardware interface callbacks"""
        self.servo_write_callback = servo_write
        self.sensor_read_callback = sensor_read
        logger.info("Hardware callbacks configured")
        
    def start(self) -> bool:
        """Start the motion control system"""
        if self.running:
            logger.warning("Motion control system already running")
            return True
            
        try:
            # Initialize subsystems
            if not self._initialize_subsystems():
                return False
                
            # Start control thread
            self.running = True
            self.control_thread = threading.Thread(
                target=self._control_loop,
                name="MotionControl",
                daemon=True
            )
            self.control_thread.start()
            
            self.status.state = MotionState.READY
            self.status.mode = MotionMode.IDLE
            
            logger.info("Motion control system started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start motion control system: {e}")
            self.status.state = MotionState.ERROR
            return False
            
    def stop(self) -> bool:
        """Stop the motion control system"""
        if not self.running:
            return True
            
        logger.info("Stopping motion control system...")
        self.status.state = MotionState.STOPPING
        
        # Stop control loop
        self.running = False
        
        # Wait for control thread to finish
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
            
        # Stop subsystems
        self.controller_system.stop_control()
        if self.adaptive_controller:
            self.adaptive_controller.reset_adaptation()
            
        # Shutdown thread executor
        self.thread_executor.shutdown(wait=True)
        
        self.status.state = MotionState.READY
        self.status.mode = MotionMode.IDLE
        
        logger.info("Motion control system stopped")
        return True
        
    def emergency_stop(self):
        """Trigger emergency stop"""
        logger.critical("EMERGENCY STOP ACTIVATED")
        
        self.status.emergency_stop_active = True
        self.status.mode = MotionMode.EMERGENCY_STOP
        self.status.state = MotionState.EMERGENCY
        
        # Stop all motion immediately
        self.current_command = MotionCommand()  # Zero velocity command
        self.command_queue.clear()
        
        # Stop controllers
        self.controller_system.emergency_stop()
        
    def set_motion_command(self, command: MotionCommand) -> bool:
        """Set a new motion command"""
        if self.status.emergency_stop_active:
            logger.warning("Cannot accept commands during emergency stop")
            return False
            
        if self.status.state not in [MotionState.READY, MotionState.ACTIVE]:
            logger.warning(f"Cannot accept commands in state: {self.status.state}")
            return False
            
        # Validate command
        if not self._validate_command(command):
            return False
            
        # Set command
        command.command_id = f"cmd_{int(time.time()*1000)}"
        command.timestamp = time.time()
        self.current_command = command
        
        # Update mode based on command
        if np.linalg.norm(command.velocity) > 0.01 or abs(command.angular_velocity) > 0.01:
            if self.enable_adaptation and command.adaptation_mode is not None:
                self.status.mode = MotionMode.ADAPTIVE
            else:
                self.status.mode = MotionMode.WALKING
            self.status.state = MotionState.ACTIVE
        else:
            self.status.mode = MotionMode.IDLE
            
        logger.info(f"Motion command set: {command.command_id}")
        return True
        
    def get_status(self) -> MotionStatus:
        """Get current system status"""
        # Update uptime
        self.status.uptime = time.time() - self.start_time
        return self.status
        
    def _initialize_subsystems(self) -> bool:
        """Initialize all subsystems"""
        try:
            # Start gait generator
            self.gait_generator.start_gait()
            
            # Start controller system
            self.controller_system.start_control()
            
            # Initialize adaptive controller if enabled
            if self.adaptive_controller:
                logger.info("Adaptive gait control enabled")
                
            return True
            
        except Exception as e:
            logger.error(f"Subsystem initialization failed: {e}")
            return False
            
    def _control_loop(self):
        """Main control loop thread"""
        logger.info("Motion control loop started")
        
        last_time = time.time()
        
        while self.running:
            loop_start = time.time()
            
            try:
                # Calculate delta time
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time
                
                # Execute control cycle
                self._control_cycle(dt)
                
                # Track performance
                loop_time = (time.time() - loop_start) * 1000  # ms
                self._update_performance_metrics(loop_time)
                
                # Sleep for remaining time
                sleep_time = self.control_period - (time.time() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    logger.warning(f"Control loop overrun: {-sleep_time*1000:.1f}ms")
                    
            except Exception as e:
                logger.error(f"Control loop error: {e}")
                self._handle_control_error(e)
                
        logger.info("Motion control loop stopped")
        
    def _control_cycle(self, dt: float):
        """Execute one control cycle"""
        # Read sensors if available
        sensor_data = {}
        if self.sensor_read_callback:
            sensor_data = self.sensor_read_callback()
            
        # Update gait generation
        self._update_gait_generation(dt)
        
        # Update adaptive control if enabled
        if self.status.mode == MotionMode.ADAPTIVE and self.adaptive_controller:
            self._update_adaptive_control(sensor_data)
            
        # Generate target joint positions
        target_positions = self._generate_joint_targets()
        
        # Update PID controllers
        self._update_control_system(target_positions, dt)
        
        # Write to servos if available
        if self.servo_write_callback:
            self.servo_write_callback(self.status.joint_targets)
            
        # Update system status
        self._update_status()
        
        # Check safety conditions
        self._check_safety()
        
    def _update_gait_generation(self, dt: float):
        """Update gait generation system"""
        # Set gait velocity from current command
        self.gait_generator.set_velocity(
            self.current_command.velocity, 
            self.current_command.angular_velocity
        )
        
        # Set preferred gait if specified
        if self.current_command.preferred_gait:
            self.gait_generator.set_gait_type(self.current_command.preferred_gait)
            
        # Update gait generator
        gait_info = self.gait_generator.update(dt)
        
        # Update status
        self.status.current_gait = self.gait_generator.current_gait
        self.status.gait_phase = self.gait_generator.cycle_time / self.gait_generator.parameters.cycle_time
        
        # Extract support pattern from gait info
        if isinstance(gait_info, dict) and 'leg_phases' in gait_info:
            self.status.support_pattern = [
                phase == 'stance' for phase in gait_info['leg_phases']
            ]
        else:
            self.status.support_pattern = [True] * 6  # Default to all supporting
        
    def _update_adaptive_control(self, sensor_data: Dict[str, Any]):
        """Update adaptive gait control"""
        if not self.adaptive_controller:
            return
            
        # Create mock foot positions and contacts for adaptive controller
        # In a real implementation, these would come from sensors or gait generator
        foot_positions = {}
        foot_contacts = {}
        
        leg_names = ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']
        for i, leg_name in enumerate(leg_names):
            # Create default foot positions (mock data)
            foot_positions[leg_name] = np.array([0.15, 0.13 * (1 if i < 3 else -1), -0.2])
            foot_contacts[leg_name] = self.status.support_pattern[i] if i < len(self.status.support_pattern) else True
            
        # Update adaptive controller
        adaptation_status = self.adaptive_controller.update(foot_positions, foot_contacts)
        
        # Update status
        self.status.terrain_type = TerrainType(adaptation_status['terrain_type'])
        self.status.stability_level = StabilityLevel(adaptation_status['stability_level'])
        self.status.adaptation_active = not adaptation_status['in_recovery']
        
    def _generate_joint_targets(self) -> np.ndarray:
        """Generate target joint positions from gait"""
        # For now, use default foot positions and generate joint angles
        # In a full implementation, this would get actual foot targets from gait generator
        
        target_angles = []
        leg_names = ['L1', 'L2', 'L3', 'R1', 'R2', 'R3']
        
        for i, leg_name in enumerate(leg_names):
            try:
                # Use default foot position for now
                default_position = np.array([0.15, 0.13 * (1 if i < 3 else -1), -0.2])
                
                # Convert to joint angles using inverse kinematics
                joint_angles, success = self.kinematics.inverse_kinematics(default_position, leg_name)
                joint_angles_deg = np.degrees(joint_angles)
                target_angles.extend(joint_angles_deg)
            except Exception as e:
                logger.warning(f"IK failed for {leg_name}: {e}")
                # Use default angles as fallback
                default_angles = [0.0, 45.0, -90.0]  # Default joint angles
                target_angles.extend(default_angles)
                
        return np.array(target_angles)
        
    def _update_control_system(self, target_angles: np.ndarray, dt: float):
        """Update PID control system"""
        # Reshape target angles to 6x3 format
        target_angles_6x3 = target_angles.reshape(6, 3)
        
        # Set targets
        self.controller_system.set_target_angles(target_angles_6x3)
            
        # Update controllers
        control_outputs = self.controller_system.update(self.status.joint_angles.reshape(6, 3))
        
        # Store results
        self.status.joint_targets = target_angles
        self.status.joint_errors = target_angles - self.status.joint_angles
        
    def _update_status(self):
        """Update system status"""
        self.status.last_update = time.time()
        
        # Update velocity from command
        self.status.current_velocity = self.current_command.velocity.copy()
        self.status.current_angular_velocity = self.current_command.angular_velocity
        
        # Update controller status
        controller_status = self.controller_system.get_system_status()
        self.status.update_frequency = 1.0 / self.control_period
        
    def _check_safety(self):
        """Check safety conditions and constraints"""
        violations = []
        
        # Check joint errors
        max_error = np.max(np.abs(self.status.joint_errors))
        if max_error > self.safety_limits['max_joint_error']:
            violations.append(f"Joint error too high: {max_error:.1f}°")
            
        # Check velocity limits
        velocity_mag = np.linalg.norm(self.status.current_velocity)
        if velocity_mag > self.safety_limits['max_velocity']:
            violations.append(f"Velocity too high: {velocity_mag:.2f} m/s")
            
        # Check stability margin
        if self.status.stability_margin < self.safety_limits['min_stability_margin']:
            violations.append(f"Low stability margin: {self.status.stability_margin:.3f}m")
            
        # Check computation time
        if self.status.computation_time > self.safety_limits['max_computation_time']:
            violations.append(f"Computation overrun: {self.status.computation_time:.1f}ms")
            
        self.status.safety_violations = violations
        
        # Trigger emergency stop if critical violations
        if len(violations) > 2:
            logger.error(f"Multiple safety violations: {violations}")
            self.emergency_stop()
            
    def _update_performance_metrics(self, loop_time: float):
        """Update performance tracking"""
        self.status.computation_time = loop_time
        
        # Track loop times
        self.loop_times.append(loop_time)
        if len(self.loop_times) > self.max_loop_history:
            self.loop_times.pop(0)
            
        # Calculate frequency
        if len(self.loop_times) > 1:
            avg_period = np.mean(np.diff([time.time() - i*self.control_period 
                                        for i in range(len(self.loop_times))]))
            self.status.update_frequency = 1.0 / max(avg_period, 1e-6)
            
    def _validate_command(self, command: MotionCommand) -> bool:
        """Validate motion command"""
        # Check velocity limits
        vel_mag = np.linalg.norm(command.velocity)
        if vel_mag > command.max_speed:
            logger.warning(f"Command velocity too high: {vel_mag:.2f} > {command.max_speed:.2f}")
            return False
            
        # Check angular velocity limits
        if abs(command.angular_velocity) > command.max_angular_speed:
            logger.warning(f"Command angular velocity too high: {abs(command.angular_velocity):.2f}")
            return False
            
        return True
        
    def _handle_control_error(self, error: Exception):
        """Handle control loop errors"""
        self.status.state = MotionState.ERROR
        logger.error(f"Control error: {error}")
        
        # Try to recover
        try:
            self.current_command = MotionCommand()  # Stop motion
            self.status.mode = MotionMode.IDLE
            self.status.state = MotionState.READY
        except Exception as e:
            logger.critical(f"Error recovery failed: {e}")
            self.emergency_stop()
            
    def _get_stability_margin(self) -> float:
        """Get current stability margin for adaptive controller"""
        # Placeholder - would calculate from support polygon and COG
        return self.status.stability_margin if self.status.stability_margin > 0 else 0.15
        
    def _get_current_speed(self) -> Tuple[float, float]:
        """Get current and target speeds for adaptive controller"""
        actual_speed = np.linalg.norm(self.status.current_velocity)
        target_speed = np.linalg.norm(self.current_command.velocity)
        return actual_speed, target_speed
        
    def _get_energy_usage(self) -> float:
        """Get current energy usage for adaptive controller"""
        # Placeholder - would calculate from joint velocities and torques
        return 50.0  # Mock energy usage
        
    # Convenience methods for common operations
    def walk_forward(self, speed: float = 0.1) -> bool:
        """Simple forward walking command"""
        command = MotionCommand(velocity=np.array([speed, 0, 0]))
        return self.set_motion_command(command)
        
    def walk_backward(self, speed: float = 0.1) -> bool:
        """Simple backward walking command"""
        command = MotionCommand(velocity=np.array([-speed, 0, 0]))
        return self.set_motion_command(command)
        
    def turn_left(self, angular_speed: float = 0.5) -> bool:
        """Simple left turn command"""
        command = MotionCommand(angular_velocity=angular_speed)
        return self.set_motion_command(command)
        
    def turn_right(self, angular_speed: float = 0.5) -> bool:
        """Simple right turn command"""
        command = MotionCommand(angular_velocity=-angular_speed)
        return self.set_motion_command(command)
        
    def stop_motion(self) -> bool:
        """Stop all motion"""
        command = MotionCommand()  # Zero velocity
        return self.set_motion_command(command)
        
    def set_gait_type(self, gait_type: GaitType) -> bool:
        """Change gait type"""
        self.gait_generator.set_gait_type(gait_type)
        return True
        
    def get_performance_summary(self) -> Dict[str, Any]:
        """Get performance summary"""
        if not self.loop_times:
            return {}
            
        return {
            'average_loop_time': np.mean(self.loop_times),
            'max_loop_time': np.max(self.loop_times),
            'min_loop_time': np.min(self.loop_times),
            'loop_time_std': np.std(self.loop_times),
            'target_frequency': self.control_frequency,
            'actual_frequency': self.status.update_frequency,
            'uptime': self.status.uptime,
            'total_loops': len(self.loop_times)
        }


# Export main classes
__all__ = [
    'MotionController',
    'MotionCommand',
    'MotionStatus',
    'MotionMode',
    'MotionState'
]