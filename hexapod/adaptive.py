"""
Phase 3.3: Adaptive Gait Control System

This module implements intelligent gait adaptation that automatically adjusts
gait patterns based on terrain conditions, stability requirements, and robot state.

Key Features:
- Automatic gait switching based on stability margin
- Terrain-adaptive step length and height  
- Speed-dependent gait selection
- Recovery behaviors for stability loss
- Terrain slope compensation
- Real-time gait parameter adjustment
- Learning algorithms for gait optimization

Author: GitHub Copilot
Date: October 2025
"""

import numpy as np
import time
from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional, Callable
import logging

from .gait import GaitGenerator, GaitType, GaitParameters
from .kinematics import HexapodKinematics

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TerrainType(Enum):
    """Types of terrain for adaptive gait control"""
    FLAT = "flat"
    ROUGH = "rough"
    STEEP = "steep"
    SOFT = "soft"
    SLIPPERY = "slippery"
    UNKNOWN = "unknown"


class StabilityLevel(Enum):
    """Robot stability levels"""
    CRITICAL = "critical"      # < 0.05m stability margin
    LOW = "low"               # 0.05-0.1m stability margin
    MEDIUM = "medium"         # 0.1-0.2m stability margin
    HIGH = "high"             # > 0.2m stability margin


class AdaptationMode(Enum):
    """Adaptation modes for different scenarios"""
    CONSERVATIVE = "conservative"  # Prioritize stability
    BALANCED = "balanced"         # Balance speed and stability
    AGGRESSIVE = "aggressive"     # Prioritize speed
    RECOVERY = "recovery"         # Emergency recovery mode


@dataclass
class TerrainData:
    """Terrain analysis data"""
    terrain_type: TerrainType = TerrainType.UNKNOWN
    slope_angle: float = 0.0          # degrees
    roughness: float = 0.0            # 0-1 scale
    friction_coefficient: float = 0.8  # estimated friction
    soft_factor: float = 0.0          # 0-1 scale (1 = very soft)
    confidence: float = 0.0           # 0-1 confidence in analysis
    
    # Terrain history for learning
    history: List[Tuple[float, float, float]] = field(default_factory=list)  # (time, success_rate, stability)


@dataclass
class AdaptationParameters:
    """Parameters for adaptive gait control"""
    # Stability thresholds
    critical_stability_threshold: float = 0.05   # meters
    low_stability_threshold: float = 0.1         # meters
    medium_stability_threshold: float = 0.2      # meters
    
    # Speed adaptation
    min_speed_factor: float = 0.3    # Minimum speed multiplier
    max_speed_factor: float = 1.5    # Maximum speed multiplier
    
    # Step adaptation
    min_step_height: float = 0.02    # meters
    max_step_height: float = 0.08    # meters
    min_step_length: float = 0.05    # meters
    max_step_length: float = 0.15    # meters
    
    # Learning parameters
    learning_rate: float = 0.1
    adaptation_time_constant: float = 2.0  # seconds
    memory_decay: float = 0.95             # per second
    
    # Recovery parameters
    recovery_timeout: float = 5.0           # seconds
    emergency_stop_threshold: float = 0.03  # meters stability margin


@dataclass
class AdaptationState:
    """Current state of adaptive gait system"""
    current_mode: AdaptationMode = AdaptationMode.BALANCED
    current_gait: GaitType = GaitType.TRIPOD
    stability_level: StabilityLevel = StabilityLevel.HIGH
    
    # Adaptation factors
    speed_factor: float = 1.0
    step_height_factor: float = 1.0
    step_length_factor: float = 1.0
    
    # Learning state
    success_rate: float = 1.0
    adaptation_confidence: float = 0.5
    last_adaptation_time: float = 0.0
    
    # Recovery state
    in_recovery: bool = False
    recovery_start_time: float = 0.0
    consecutive_failures: int = 0


class TerrainAnalyzer:
    """Analyzes terrain conditions for adaptive gait control"""
    
    def __init__(self, kinematics: HexapodKinematics):
        self.kinematics = kinematics
        self.terrain_data = TerrainData()
        self.foot_contact_history: Dict[str, List[Tuple[float, np.ndarray, bool]]] = {}
        self.analysis_window = 2.0  # seconds
        
    def update_foot_contact(self, leg_name: str, position: np.ndarray, in_contact: bool):
        """Update foot contact information for terrain analysis"""
        current_time = time.time()
        
        if leg_name not in self.foot_contact_history:
            self.foot_contact_history[leg_name] = []
            
        # Add new contact data
        self.foot_contact_history[leg_name].append((current_time, position.copy(), in_contact))
        
        # Remove old data
        cutoff_time = current_time - self.analysis_window
        self.foot_contact_history[leg_name] = [
            (t, pos, contact) for t, pos, contact in self.foot_contact_history[leg_name]
            if t > cutoff_time
        ]
        
    def analyze_terrain(self) -> TerrainData:
        """Analyze current terrain conditions"""
        current_time = time.time()
        
        # Analyze slope
        slope_angle = self._analyze_slope()
        
        # Analyze roughness
        roughness = self._analyze_roughness()
        
        # Estimate friction
        friction = self._estimate_friction()
        
        # Detect soft surfaces
        soft_factor = self._detect_soft_surface()
        
        # Classify terrain type
        terrain_type = self._classify_terrain(slope_angle, roughness, friction, soft_factor)
        
        # Calculate confidence
        confidence = self._calculate_confidence()
        
        # Update terrain data
        self.terrain_data.terrain_type = terrain_type
        self.terrain_data.slope_angle = slope_angle
        self.terrain_data.roughness = roughness
        self.terrain_data.friction_coefficient = friction
        self.terrain_data.soft_factor = soft_factor
        self.terrain_data.confidence = confidence
        
        return self.terrain_data
    
    def _analyze_slope(self) -> float:
        """Analyze terrain slope from foot positions"""
        if len(self.foot_contact_history) < 3:
            return 0.0
            
        # Get recent contact points
        contact_points = []
        current_time = time.time()
        
        for leg_name, history in self.foot_contact_history.items():
            recent_contacts = [
                pos for t, pos, contact in history
                if contact and (current_time - t) < 1.0
            ]
            if recent_contacts:
                contact_points.append(recent_contacts[-1])
                
        if len(contact_points) < 3:
            return 0.0
            
        # Fit plane to contact points
        points = np.array(contact_points)
        if points.shape[0] >= 3:
            # Use SVD to fit plane
            centroid = np.mean(points, axis=0)
            points_centered = points - centroid
            
            if points_centered.shape[0] >= 3:
                try:
                    _, _, V = np.linalg.svd(points_centered)
                    normal = V[-1]  # Last row is normal to best-fit plane
                    
                    # Calculate slope angle
                    slope_rad = np.arccos(np.abs(normal[2]))
                    slope_deg = np.degrees(slope_rad)
                    
                    return min(slope_deg, 45.0)  # Cap at 45 degrees
                except np.linalg.LinAlgError:
                    return 0.0
                    
        return 0.0
    
    def _analyze_roughness(self) -> float:
        """Analyze terrain roughness from foot position variance"""
        if len(self.foot_contact_history) < 2:
            return 0.0
            
        z_variations = []
        
        for leg_name, history in self.foot_contact_history.items():
            contact_points = [
                pos[2] for t, pos, contact in history
                if contact
            ]
            if len(contact_points) > 1:
                z_variations.extend(contact_points)
                
        if len(z_variations) < 3:
            return 0.0
            
        # Calculate coefficient of variation
        z_array = np.array(z_variations)
        z_std = np.std(z_array)
        z_mean = np.abs(np.mean(z_array))
        
        if z_mean > 0.001:  # Avoid division by zero
            roughness = min(z_std / z_mean, 1.0)
        else:
            roughness = min(z_std * 100, 1.0)  # Scale small absolute variations
            
        return roughness
    
    def _estimate_friction(self) -> float:
        """Estimate surface friction coefficient"""
        # Placeholder - would use slip detection in real implementation
        base_friction = 0.8
        
        # Adjust based on terrain type indicators
        if self.terrain_data.soft_factor > 0.5:
            base_friction *= 0.7  # Soft surfaces have lower friction
            
        if self.terrain_data.roughness > 0.5:
            base_friction *= 0.8  # Rough surfaces may have variable friction
            
        return max(0.3, min(1.0, base_friction))
    
    def _detect_soft_surface(self) -> float:
        """Detect soft/deformable surfaces"""
        # Placeholder - would analyze foot penetration depth
        # For now, return low soft factor
        return 0.1
    
    def _classify_terrain(self, slope: float, roughness: float, friction: float, soft_factor: float) -> TerrainType:
        """Classify terrain type based on analyzed parameters"""
        if slope > 20.0:
            return TerrainType.STEEP
        elif roughness > 0.6:
            return TerrainType.ROUGH
        elif soft_factor > 0.5:
            return TerrainType.SOFT
        elif friction < 0.5:
            return TerrainType.SLIPPERY
        elif slope < 5.0 and roughness < 0.2:
            return TerrainType.FLAT
        else:
            return TerrainType.UNKNOWN
    
    def _calculate_confidence(self) -> float:
        """Calculate confidence in terrain analysis"""
        # Base confidence on amount of data
        total_contacts = sum(len(history) for history in self.foot_contact_history.values())
        data_confidence = min(total_contacts / 50.0, 1.0)
        
        # Reduce confidence for recent changes
        if len(self.terrain_data.history) > 1:
            recent_stability = np.mean([stability for _, _, stability in self.terrain_data.history[-5:]])
            stability_confidence = min(recent_stability, 1.0)
        else:
            stability_confidence = 0.5
            
        return (data_confidence + stability_confidence) / 2.0


class GaitOptimizer:
    """Optimizes gait parameters using learning algorithms"""
    
    def __init__(self, parameters: AdaptationParameters):
        self.parameters = parameters
        self.performance_history: List[Tuple[float, Dict, float]] = []  # (time, params, performance)
        self.optimization_weights = {
            'stability': 0.4,
            'speed': 0.3,
            'energy': 0.2,
            'smoothness': 0.1
        }
        
    def evaluate_performance(self, stability_margin: float, actual_speed: float, 
                           target_speed: float, energy_usage: float) -> float:
        """Evaluate current gait performance"""
        # Normalize metrics to 0-1 scale
        stability_score = min(stability_margin / 0.3, 1.0)  # 0.3m = perfect stability
        speed_score = min(actual_speed / max(target_speed, 0.01), 1.0)
        energy_score = max(0.0, 1.0 - energy_usage / 100.0)  # Assume 100 is max energy
        
        # Smoothness based on acceleration changes (simplified)
        smoothness_score = 0.8  # Placeholder
        
        # Weighted performance score
        performance = (
            self.optimization_weights['stability'] * stability_score +
            self.optimization_weights['speed'] * speed_score +
            self.optimization_weights['energy'] * energy_score +
            self.optimization_weights['smoothness'] * smoothness_score
        )
        
        return performance
    
    def optimize_parameters(self, current_params: GaitParameters, 
                          terrain_data: TerrainData, 
                          performance_score: float) -> GaitParameters:
        """Optimize gait parameters based on performance feedback"""
        current_time = time.time()
        
        # Store performance data
        param_dict = {
            'cycle_time': current_params.cycle_time,
            'duty_factor': current_params.duty_factor,
            'step_height': current_params.step_height,
            'stride_length': current_params.stride_length  # Use stride_length
        }
        self.performance_history.append((current_time, param_dict.copy(), performance_score))
        
        # Remove old history
        cutoff_time = current_time - 60.0  # Keep 1 minute of history
        self.performance_history = [
            (t, params, score) for t, params, score in self.performance_history
            if t > cutoff_time
        ]
        
        # Simple gradient-based optimization
        optimized_params = self._gradient_optimization(current_params, terrain_data, performance_score)
        
        return optimized_params
    
    def _gradient_optimization(self, current_params: GaitParameters, 
                              terrain_data: TerrainData, 
                              performance_score: float) -> GaitParameters:
        """Simple gradient-based parameter optimization"""
        if len(self.performance_history) < 3:
            return current_params
            
        # Calculate performance gradient
        recent_history = self.performance_history[-5:]
        if len(recent_history) < 2:
            return current_params
            
        # Simple adaptation rules based on terrain and performance
        new_params = GaitParameters(
            cycle_time=current_params.cycle_time,
            duty_factor=current_params.duty_factor,
            step_height=current_params.step_height,
            stride_length=current_params.stride_length
        )
        
        # Adapt based on terrain type
        if terrain_data.terrain_type == TerrainType.ROUGH:
            new_params.step_height = min(new_params.step_height * 1.1, 0.08)
            new_params.duty_factor = min(new_params.duty_factor * 1.05, 0.9)
        elif terrain_data.terrain_type == TerrainType.STEEP:
            new_params.stride_length = max(new_params.stride_length * 0.9, 0.05)
            new_params.duty_factor = min(new_params.duty_factor * 1.1, 0.9)
        elif terrain_data.terrain_type == TerrainType.SLIPPERY:
            new_params.stride_length = max(new_params.stride_length * 0.8, 0.05)
            new_params.cycle_time = max(new_params.cycle_time * 1.2, 0.5)
            
        # Adapt based on performance
        if performance_score < 0.6:  # Poor performance
            learning_rate = self.parameters.learning_rate
            if terrain_data.roughness > 0.5:
                new_params.step_height = min(new_params.step_height * (1 + learning_rate), 0.08)
            if terrain_data.slope_angle > 15.0:
                new_params.stride_length = max(new_params.stride_length * (1 - learning_rate), 0.05)
                
        return new_params


class AdaptiveGaitController:
    """Main adaptive gait control system"""
    
    def __init__(self, kinematics: HexapodKinematics, 
                 gait_generator: GaitGenerator,
                 parameters: Optional[AdaptationParameters] = None):
        self.kinematics = kinematics
        self.gait_generator = gait_generator
        self.parameters = parameters or AdaptationParameters()
        
        # Components
        self.terrain_analyzer = TerrainAnalyzer(kinematics)
        self.gait_optimizer = GaitOptimizer(self.parameters)
        
        # State
        self.state = AdaptationState()
        self.last_update_time = 0.0
        
        # Performance tracking
        self.performance_metrics = {
            'stability_margin': 0.0,
            'actual_speed': 0.0,
            'target_speed': 0.0,
            'energy_usage': 0.0,
            'step_success_rate': 1.0
        }
        
        # Callbacks for external integration
        self.stability_callback: Optional[Callable[[], float]] = None
        self.speed_callback: Optional[Callable[[], Tuple[float, float]]] = None
        self.energy_callback: Optional[Callable[[], float]] = None
        
    def set_callbacks(self, stability_fn: Callable[[], float],
                     speed_fn: Callable[[], Tuple[float, float]],
                     energy_fn: Callable[[], float]):
        """Set callback functions for external data"""
        self.stability_callback = stability_fn
        self.speed_callback = speed_fn
        self.energy_callback = energy_fn
    
    def update(self, foot_positions: Dict[str, np.ndarray], 
               foot_contacts: Dict[str, bool]) -> Dict[str, any]:
        """Main update loop for adaptive gait control"""
        current_time = time.time()
        dt = current_time - self.last_update_time if self.last_update_time > 0 else 0.01
        self.last_update_time = current_time
        
        # Update terrain analysis
        for leg_name, position in foot_positions.items():
            in_contact = foot_contacts.get(leg_name, False)
            self.terrain_analyzer.update_foot_contact(leg_name, position, in_contact)
            
        terrain_data = self.terrain_analyzer.analyze_terrain()
        
        # Update performance metrics
        self._update_performance_metrics()
        
        # Determine stability level
        stability_level = self._assess_stability()
        
        # Select adaptation mode
        adaptation_mode = self._select_adaptation_mode(stability_level, terrain_data)
        
        # Check for recovery mode
        if self._should_enter_recovery(stability_level):
            self._enter_recovery_mode()
        elif self.state.in_recovery and self._can_exit_recovery(stability_level):
            self._exit_recovery_mode()
            
        # Adapt gait if not in recovery
        if not self.state.in_recovery:
            self._adapt_gait(terrain_data, stability_level, adaptation_mode)
        else:
            self._recovery_gait()
            
        # Update state
        self.state.stability_level = stability_level
        self.state.current_mode = adaptation_mode
        
        # Return adaptation status
        return {
            'terrain_type': terrain_data.terrain_type.value,
            'slope_angle': terrain_data.slope_angle,
            'roughness': terrain_data.roughness,
            'stability_level': stability_level.value,
            'adaptation_mode': adaptation_mode.value,
            'current_gait': self.state.current_gait.value,
            'speed_factor': self.state.speed_factor,
            'step_height_factor': self.state.step_height_factor,
            'step_length_factor': self.state.step_length_factor,
            'in_recovery': self.state.in_recovery,
            'performance_score': self._calculate_performance_score()
        }
    
    def _update_performance_metrics(self):
        """Update performance metrics from callbacks"""
        if self.stability_callback:
            self.performance_metrics['stability_margin'] = self.stability_callback()
            
        if self.speed_callback:
            actual, target = self.speed_callback()
            self.performance_metrics['actual_speed'] = actual
            self.performance_metrics['target_speed'] = target
            
        if self.energy_callback:
            self.performance_metrics['energy_usage'] = self.energy_callback()
    
    def _assess_stability(self) -> StabilityLevel:
        """Assess current stability level"""
        margin = self.performance_metrics['stability_margin']
        
        if margin < self.parameters.critical_stability_threshold:
            return StabilityLevel.CRITICAL
        elif margin < self.parameters.low_stability_threshold:
            return StabilityLevel.LOW
        elif margin < self.parameters.medium_stability_threshold:
            return StabilityLevel.MEDIUM
        else:
            return StabilityLevel.HIGH
    
    def _select_adaptation_mode(self, stability_level: StabilityLevel, 
                               terrain_data: TerrainData) -> AdaptationMode:
        """Select appropriate adaptation mode"""
        if self.state.in_recovery:
            return AdaptationMode.RECOVERY
            
        if stability_level == StabilityLevel.CRITICAL:
            return AdaptationMode.CONSERVATIVE
        elif stability_level == StabilityLevel.LOW:
            return AdaptationMode.CONSERVATIVE
        elif terrain_data.terrain_type in [TerrainType.STEEP, TerrainType.ROUGH, TerrainType.SLIPPERY]:
            return AdaptationMode.CONSERVATIVE
        elif stability_level == StabilityLevel.HIGH and terrain_data.terrain_type == TerrainType.FLAT:
            return AdaptationMode.AGGRESSIVE
        else:
            return AdaptationMode.BALANCED
    
    def _should_enter_recovery(self, stability_level: StabilityLevel) -> bool:
        """Check if recovery mode should be entered"""
        if stability_level == StabilityLevel.CRITICAL:
            self.state.consecutive_failures += 1
            return self.state.consecutive_failures >= 3
        else:
            self.state.consecutive_failures = 0
            return False
    
    def _can_exit_recovery(self, stability_level: StabilityLevel) -> bool:
        """Check if recovery mode can be exited"""
        if not self.state.in_recovery:
            return False
            
        current_time = time.time()
        recovery_duration = current_time - self.state.recovery_start_time
        
        return (stability_level in [StabilityLevel.MEDIUM, StabilityLevel.HIGH] and 
                recovery_duration > 2.0)
    
    def _enter_recovery_mode(self):
        """Enter recovery mode"""
        self.state.in_recovery = True
        self.state.recovery_start_time = time.time()
        self.state.current_mode = AdaptationMode.RECOVERY
        logger.warning("Entering recovery mode due to stability issues")
    
    def _exit_recovery_mode(self):
        """Exit recovery mode"""
        self.state.in_recovery = False
        self.state.consecutive_failures = 0
        logger.info("Exiting recovery mode - stability restored")
    
    def _adapt_gait(self, terrain_data: TerrainData, stability_level: StabilityLevel, 
                   adaptation_mode: AdaptationMode):
        """Adapt gait based on current conditions"""
        current_time = time.time()
        
        # Don't adapt too frequently
        if current_time - self.state.last_adaptation_time < self.parameters.adaptation_time_constant:
            return
            
        # Select optimal gait type
        new_gait_type = self._select_optimal_gait(terrain_data, stability_level, adaptation_mode)
        
        # Calculate adaptation factors
        speed_factor = self._calculate_speed_factor(terrain_data, stability_level, adaptation_mode)
        step_height_factor = self._calculate_step_height_factor(terrain_data, stability_level)
        step_length_factor = self._calculate_step_length_factor(terrain_data, stability_level, adaptation_mode)
        
        # Apply adaptations
        if new_gait_type != self.state.current_gait:
            self.gait_generator.set_gait_type(new_gait_type)
            self.state.current_gait = new_gait_type
            logger.info(f"Switched to {new_gait_type.value} gait")
            
        # Update gait parameters
        current_params = self.gait_generator.parameters
        
        # Apply optimization
        performance_score = self._calculate_performance_score()
        optimized_params = self.gait_optimizer.optimize_parameters(
            current_params, terrain_data, performance_score)
        
        # Apply adaptation factors
        optimized_params.step_height *= step_height_factor
        optimized_params.stride_length *= step_length_factor  # Use stride_length instead of step_length
        
        # Clamp to safe ranges
        optimized_params.step_height = np.clip(
            optimized_params.step_height,
            self.parameters.min_step_height,
            self.parameters.max_step_height
        )
        optimized_params.stride_length = np.clip(
            optimized_params.stride_length,
            self.parameters.min_step_length,
            self.parameters.max_step_length
        )
        
        # Update parameters using the existing method
        self.gait_generator.update_parameters(
            step_height=optimized_params.step_height,
            stride_length=optimized_params.stride_length,
            cycle_time=optimized_params.cycle_time,
            duty_factor=optimized_params.duty_factor
        )
        
        # Update state
        self.state.speed_factor = speed_factor
        self.state.step_height_factor = step_height_factor
        self.state.step_length_factor = step_length_factor
        self.state.last_adaptation_time = current_time
    
    def _recovery_gait(self):
        """Apply recovery gait settings"""
        # Switch to most stable gait
        if self.state.current_gait != GaitType.WAVE:
            self.gait_generator.set_gait_type(GaitType.WAVE)
            self.state.current_gait = GaitType.WAVE
            
        # Conservative parameters
        self.gait_generator.update_parameters(
            cycle_time=2.0,           # Slow
            duty_factor=0.85,         # High stability
            step_height=0.03,         # Low steps
            stride_length=0.06        # Short steps
        )
        
        self.state.speed_factor = 0.3
        self.state.step_height_factor = 0.5
        self.state.step_length_factor = 0.5
    
    def _select_optimal_gait(self, terrain_data: TerrainData, 
                            stability_level: StabilityLevel,
                            adaptation_mode: AdaptationMode) -> GaitType:
        """Select optimal gait type for current conditions"""
        if adaptation_mode == AdaptationMode.RECOVERY:
            return GaitType.WAVE
            
        if terrain_data.terrain_type == TerrainType.STEEP:
            return GaitType.WAVE  # Most stable
        elif terrain_data.terrain_type == TerrainType.ROUGH:
            return GaitType.RIPPLE  # Good balance
        elif stability_level == StabilityLevel.HIGH and adaptation_mode == AdaptationMode.AGGRESSIVE:
            return GaitType.TRIPOD  # Fastest
        else:
            return GaitType.TRIPOD  # Default balanced choice
    
    def _calculate_speed_factor(self, terrain_data: TerrainData, 
                               stability_level: StabilityLevel,
                               adaptation_mode: AdaptationMode) -> float:
        """Calculate speed adaptation factor"""
        base_factor = 1.0
        
        # Terrain adjustments
        if terrain_data.terrain_type == TerrainType.STEEP:
            base_factor *= 0.6
        elif terrain_data.terrain_type == TerrainType.ROUGH:
            base_factor *= 0.8
        elif terrain_data.terrain_type == TerrainType.SLIPPERY:
            base_factor *= 0.7
        elif terrain_data.terrain_type == TerrainType.SOFT:
            base_factor *= 0.9
            
        # Stability adjustments
        if stability_level == StabilityLevel.CRITICAL:
            base_factor *= 0.3
        elif stability_level == StabilityLevel.LOW:
            base_factor *= 0.6
        elif stability_level == StabilityLevel.MEDIUM:
            base_factor *= 0.9
            
        # Mode adjustments
        if adaptation_mode == AdaptationMode.CONSERVATIVE:
            base_factor *= 0.7
        elif adaptation_mode == AdaptationMode.AGGRESSIVE:
            base_factor *= 1.3
        elif adaptation_mode == AdaptationMode.RECOVERY:
            base_factor *= 0.3
            
        return np.clip(base_factor, self.parameters.min_speed_factor, self.parameters.max_speed_factor)
    
    def _calculate_step_height_factor(self, terrain_data: TerrainData, 
                                     stability_level: StabilityLevel) -> float:
        """Calculate step height adaptation factor"""
        base_factor = 1.0
        
        # Terrain adjustments
        if terrain_data.terrain_type == TerrainType.ROUGH:
            base_factor *= 1.4
        elif terrain_data.terrain_type == TerrainType.STEEP:
            base_factor *= 1.2
        elif terrain_data.terrain_type == TerrainType.SOFT:
            base_factor *= 1.1
            
        # Stability adjustments
        if stability_level == StabilityLevel.CRITICAL:
            base_factor *= 0.6
        elif stability_level == StabilityLevel.LOW:
            base_factor *= 0.8
            
        return np.clip(base_factor, 0.5, 2.0)
    
    def _calculate_step_length_factor(self, terrain_data: TerrainData, 
                                     stability_level: StabilityLevel,
                                     adaptation_mode: AdaptationMode) -> float:
        """Calculate step length adaptation factor"""
        base_factor = 1.0
        
        # Terrain adjustments
        if terrain_data.terrain_type == TerrainType.STEEP:
            base_factor *= 0.7
        elif terrain_data.terrain_type == TerrainType.SLIPPERY:
            base_factor *= 0.6
        elif terrain_data.terrain_type == TerrainType.SOFT:
            base_factor *= 0.8
            
        # Stability adjustments
        if stability_level == StabilityLevel.CRITICAL:
            base_factor *= 0.5
        elif stability_level == StabilityLevel.LOW:
            base_factor *= 0.7
            
        # Mode adjustments
        if adaptation_mode == AdaptationMode.CONSERVATIVE:
            base_factor *= 0.8
        elif adaptation_mode == AdaptationMode.AGGRESSIVE:
            base_factor *= 1.2
        elif adaptation_mode == AdaptationMode.RECOVERY:
            base_factor *= 0.5
            
        return np.clip(base_factor, 0.5, 1.5)
    
    def _calculate_performance_score(self) -> float:
        """Calculate overall performance score"""
        return self.gait_optimizer.evaluate_performance(
            self.performance_metrics['stability_margin'],
            self.performance_metrics['actual_speed'],
            self.performance_metrics['target_speed'],
            self.performance_metrics['energy_usage']
        )
    
    def get_adaptation_status(self) -> Dict[str, any]:
        """Get current adaptation status"""
        return {
            'terrain_type': self.terrain_analyzer.terrain_data.terrain_type.value,
            'terrain_confidence': self.terrain_analyzer.terrain_data.confidence,
            'slope_angle': self.terrain_analyzer.terrain_data.slope_angle,
            'roughness': self.terrain_analyzer.terrain_data.roughness,
            'friction_coefficient': self.terrain_analyzer.terrain_data.friction_coefficient,
            'stability_level': self.state.stability_level.value,
            'adaptation_mode': self.state.current_mode.value,
            'current_gait': self.state.current_gait.value,
            'speed_factor': self.state.speed_factor,
            'step_height_factor': self.state.step_height_factor,
            'step_length_factor': self.state.step_length_factor,
            'in_recovery': self.state.in_recovery,
            'adaptation_confidence': self.state.adaptation_confidence,
            'performance_score': self._calculate_performance_score(),
            'consecutive_failures': self.state.consecutive_failures
        }
    
    def force_adaptation_mode(self, mode: AdaptationMode):
        """Force a specific adaptation mode (for testing/debugging)"""
        self.state.current_mode = mode
        logger.info(f"Forced adaptation mode to {mode.value}")
    
    def reset_adaptation(self):
        """Reset adaptation system to default state"""
        self.state = AdaptationState()
        self.terrain_analyzer = TerrainAnalyzer(self.kinematics)
        self.gait_optimizer = GaitOptimizer(self.parameters)
        logger.info("Adaptive gait control system reset")


# Export main classes
__all__ = [
    'AdaptiveGaitController',
    'TerrainAnalyzer', 
    'GaitOptimizer',
    'TerrainType',
    'StabilityLevel', 
    'AdaptationMode',
    'TerrainData',
    'AdaptationParameters',
    'AdaptationState'
]