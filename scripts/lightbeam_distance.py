#!/usr/bin/env python3
"""
OSGT四类物体LightBeam避障系统
实时读取8个传感器距离数据，提供高级决策算法避障
"""

import numpy as np
import time
import math
from collections import deque
from typing import Dict, List, Tuple, Optional
from isaacsim.sensors.physx import _range_sensor

try:
    import cupy as cp
    GPU_AVAILABLE = True
except ImportError:
    GPU_AVAILABLE = False
    cp = np

class OSGTLightBeamAvoidanceSystem:
    """OSGT四类物体LightBeam避障系统"""
    
    def __init__(self, config):
        self.config = config
        self.robot_prim_path = config.PATHS["robot_prim_path"]
        
        # 传感器配置
        self.sensors = {
            "front_bottom": {"layer": "bottom", "direction": "front", "weight": 1.0},
            "back_bottom": {"layer": "bottom", "direction": "back", "weight": 0.6},
            "left_bottom": {"layer": "bottom", "direction": "left", "weight": 0.8},
            "right_bottom": {"layer": "bottom", "direction": "right", "weight": 0.8},
            "front_top": {"layer": "top", "direction": "front", "weight": 0.7},
            "back_top": {"layer": "top", "direction": "back", "weight": 0.4},
            "left_top": {"layer": "top", "direction": "left", "weight": 0.5},
            "right_top": {"layer": "top", "direction": "right", "weight": 0.5}
        }
        
        # 优化的避障参数
        self.min_valid_distance = 0.2  # 最小有效距离，过滤地板干扰
        self.max_valid_distance = 8.0  # 最大有效距离
        self.critical_distance = 1.5
        self.warning_distance = 3.0
        self.safe_distance = 4.5
        
        # 速度控制参数
        self.max_linear_speed = config.ROBOT_CONTROL["max_linear_velocity"]
        self.max_angular_speed = config.ROBOT_CONTROL["max_angular_velocity"]
        
        # 传感器接口
        self.lightbeam_interface = None
        self.timeline = None
        
        # 距离数据缓存（增加缓存大小以提高稳定性）
        self.distance_buffer = {}
        for sensor_name in self.sensors.keys():
            self.distance_buffer[sensor_name] = deque(maxlen=7)
        
        # 避障状态
        self.current_distances = {}
        self.last_avoidance_action = "直行"
        self.last_display_time = 0
        
        # 运动稳定性控制
        self.movement_history = deque(maxlen=10)
        self.last_angular_command = 0.0
        self.angular_change_threshold = 0.8
        self.direction_stability_counter = 0
        self.stable_direction_threshold = 3
        
        # 速度平滑（增强版）
        self.velocity_smoother = VelocitySmoother(alpha=0.25)
        
        # GPU加速
        self.use_gpu = GPU_AVAILABLE
        
        # 地板过滤
        self.ground_filter_enabled = True
        self.ground_detection_threshold = 0.15
        
        # 初始化
        self.initialize()
    
    def initialize(self):
        """初始化传感器接口"""
        try:
            self.lightbeam_interface = _range_sensor.acquire_lightbeam_sensor_interface()
            import omni.timeline
            self.timeline = omni.timeline.get_timeline_interface()
            return True
        except Exception as e:
            return False
    
    def get_sensor_distance(self, sensor_name: str) -> Optional[float]:
        """获取单个传感器的距离数据（带地板过滤）"""
        if not self.timeline or not self.timeline.is_playing():
            return None
        
        sensor_path = f"{self.robot_prim_path}/create3_robot/create_3/base_link/{sensor_name}"
        
        try:
            linear_depth = self.lightbeam_interface.get_linear_depth_data(sensor_path)
            beam_hit = self.lightbeam_interface.get_beam_hit_data(sensor_path)
            
            if linear_depth is not None and beam_hit is not None:
                valid_distances = []
                for i in range(len(linear_depth)):
                    if beam_hit[i] and linear_depth[i] > self.min_valid_distance:
                        # 地板过滤：底层传感器忽略过近的检测
                        if "bottom" in sensor_name and linear_depth[i] < self.ground_detection_threshold:
                            continue
                        
                        # 有效距离范围过滤
                        if linear_depth[i] <= self.max_valid_distance:
                            valid_distances.append(linear_depth[i])
                
                if valid_distances:
                    return min(valid_distances)
            
            return None
            
        except Exception:
            return None
    
    def update_all_sensor_data(self):
        """更新所有传感器数据"""
        for sensor_name in self.sensors.keys():
            distance = self.get_sensor_distance(sensor_name)
            if distance is not None:
                self.distance_buffer[sensor_name].append(distance)
                self.current_distances[sensor_name] = self.get_smoothed_distance(sensor_name)
            else:
                self.current_distances[sensor_name] = None
    
    def get_smoothed_distance(self, sensor_name: str) -> Optional[float]:
        """获取平滑后的距离数据（增强滤波）"""
        buffer = self.distance_buffer[sensor_name]
        if len(buffer) < 2:
            return None
        
        # 使用加权平均，最新数据权重更高
        distances = list(buffer)
        weights = np.linspace(0.5, 1.0, len(distances))
        weights = weights / np.sum(weights)
        
        return np.average(distances, weights=weights)
    
    def calculate_avoidance_command(self, target_linear: float, target_angular: float, 
                                   current_pos: np.ndarray, target_pos: np.ndarray) -> Tuple[float, float]:
        """计算避障后的运动命令（优化版）"""
        # 确保输入参数是numpy数组
        if not isinstance(current_pos, np.ndarray):
            current_pos = np.array(current_pos)
        if not isinstance(target_pos, np.ndarray):
            target_pos = np.array(target_pos)
        
        # 更新传感器数据
        self.update_all_sensor_data()
        
        # 计算到目标的方向向量
        direction_to_target = target_pos[:2] - current_pos[:2]
        distance_to_target = np.linalg.norm(direction_to_target)
        
        if distance_to_target > 0.1:
            target_direction = direction_to_target / distance_to_target
            target_angle = np.arctan2(target_direction[1], target_direction[0])
        else:
            target_angle = 0.0
        
        # 使用优化的避障算法
        avoidance_linear, avoidance_angular = self._calculate_stable_avoidance(
            target_linear, target_angular, target_angle, distance_to_target
        )
        
        # 运动稳定性检查
        avoidance_linear, avoidance_angular = self._apply_stability_filter(
            avoidance_linear, avoidance_angular
        )
        
        # 显示传感器状态（控制频率）
        self._display_sensor_status()
        
        return self.velocity_smoother.smooth(avoidance_linear, avoidance_angular)
    
    def _calculate_stable_avoidance(self, target_linear: float, target_angular: float,
                                   target_angle: float, distance_to_target: float) -> Tuple[float, float]:
        """稳定的避障计算"""
        # 获取方向距离
        front_dist = self._get_direction_distance("front")
        back_dist = self._get_direction_distance("back")
        left_dist = self._get_direction_distance("left")
        right_dist = self._get_direction_distance("right")
        
        # 计算最小距离
        valid_distances = [d for d in [front_dist, back_dist, left_dist, right_dist] if d is not None]
        min_distance = min(valid_distances) if valid_distances else float('inf')
        
        # 基础避障逻辑
        avoidance_linear = target_linear
        avoidance_angular = target_angular
        
        if min_distance < self.critical_distance:
            # 紧急避障 - 但保持最小前进速度
            avoidance_linear = max(0.08, target_linear * 0.2)
            
            if front_dist and front_dist < self.critical_distance:
                # 前方紧急避障
                if left_dist and right_dist:
                    # 选择较安全的方向，但不要急转
                    turn_intensity = min(1.5, 3.0 / max(front_dist, 0.1))
                    avoidance_angular = turn_intensity if left_dist > right_dist else -turn_intensity
                else:
                    avoidance_angular = 1.2
            
            self.last_avoidance_action = "紧急避障"
            
        elif min_distance < self.warning_distance:
            # 警告区域避障
            speed_factor = (min_distance - self.critical_distance) / (self.warning_distance - self.critical_distance)
            avoidance_linear = target_linear * (0.3 + 0.4 * speed_factor)
            
            if front_dist and front_dist < self.warning_distance:
                # 前方警告避障
                turn_factor = (self.warning_distance - front_dist) / self.warning_distance
                if left_dist and right_dist:
                    turn_intensity = 0.8 * turn_factor
                    avoidance_angular = turn_intensity if left_dist > right_dist else -turn_intensity
                else:
                    avoidance_angular = 0.6 * turn_factor
            
            # 侧面避障微调
            if left_dist and left_dist < self.warning_distance:
                side_factor = (self.warning_distance - left_dist) / self.warning_distance * 0.3
                avoidance_angular -= side_factor
            if right_dist and right_dist < self.warning_distance:
                side_factor = (self.warning_distance - right_dist) / self.warning_distance * 0.3
                avoidance_angular += side_factor
                
            self.last_avoidance_action = "减速避障"
        else:
            # 正常区域 - 轻微调整以保持安全距离
            if front_dist and front_dist < self.safe_distance:
                gentle_factor = (self.safe_distance - front_dist) / self.safe_distance * 0.2
                avoidance_linear = target_linear * (1.0 - gentle_factor)
                
                if left_dist and right_dist:
                    turn_intensity = 0.3 * gentle_factor
                    avoidance_angular = turn_intensity if left_dist > right_dist else -turn_intensity
            
            self.last_avoidance_action = "正常行驶"
        
        # 后方避障处理
        if back_dist and back_dist < self.warning_distance and avoidance_linear < 0:
            avoidance_linear = max(0.0, avoidance_linear)
            avoidance_angular = 1.0 if left_dist and left_dist > right_dist else -1.0
            self.last_avoidance_action = "后方避障"
        
        # 限制速度
        avoidance_linear = np.clip(avoidance_linear, 0.0, self.max_linear_speed)
        avoidance_angular = np.clip(avoidance_angular, -self.max_angular_speed, self.max_angular_speed)
        
        return avoidance_linear, avoidance_angular
    
    def _apply_stability_filter(self, linear_vel: float, angular_vel: float) -> Tuple[float, float]:
        """应用稳定性过滤器，减少摇晃"""
        # 角速度变化平滑
        angular_change = abs(angular_vel - self.last_angular_command)
        
        if angular_change > self.angular_change_threshold:
            # 角度变化过大，进行平滑
            smooth_factor = 0.6
            angular_vel = smooth_factor * self.last_angular_command + (1 - smooth_factor) * angular_vel
            
            # 重置方向稳定计数器
            self.direction_stability_counter = 0
        else:
            # 角度变化较小，增加稳定性
            self.direction_stability_counter += 1
            
            if self.direction_stability_counter >= self.stable_direction_threshold:
                # 方向稳定，可以更积极地调整
                pass
            else:
                # 方向不稳定，保守调整
                angular_vel *= 0.8
        
        # 记录运动历史
        self.movement_history.append((linear_vel, angular_vel, time.time()))
        
        # 检查是否有摇晃模式
        if len(self.movement_history) >= 8:
            recent_angular = [cmd[1] for cmd in list(self.movement_history)[-6:]]
            
            # 检测前后摇晃（角速度频繁变号）
            sign_changes = 0
            for i in range(1, len(recent_angular)):
                if recent_angular[i] * recent_angular[i-1] < 0:
                    sign_changes += 1
            
            if sign_changes >= 3:
                # 检测到摇晃，强制稳定
                angular_vel *= 0.4
                if abs(angular_vel) < 0.2:
                    angular_vel = 0.0
        
        self.last_angular_command = angular_vel
        
        return linear_vel, angular_vel
    
    def _get_direction_distance(self, direction: str) -> Optional[float]:
        """获取某个方向的最小距离"""
        distances = []
        for sensor_name, sensor_info in self.sensors.items():
            if sensor_info["direction"] == direction:
                dist = self.current_distances.get(sensor_name)
                if dist is not None:
                    distances.append(dist)
        
        return min(distances) if distances else None
    
    def _display_sensor_status(self):
        """显示传感器状态（2秒间隔，单行格式）"""
        current_time = time.time()
        
        if current_time - self.last_display_time < 2.0:
            return
        
        self.last_display_time = current_time
        
        # 构建单行输出
        sensor_info = []
        for sensor_name in self.sensors.keys():
            distance = self.current_distances.get(sensor_name)
            if distance is not None:
                status = "安全"
                if distance < self.critical_distance:
                    status = "危险"
                elif distance < self.warning_distance:
                    status = "警告"
                sensor_info.append(f"{sensor_name}: {distance:.2f}m ({status})")
            else:
                sensor_info.append(f"{sensor_name}: 无数据")
        
        # 单行输出
        sensors_line = " | ".join(sensor_info)
        print(f"传感器距离: {sensors_line}")
        print(f"避障操作: {self.last_avoidance_action}")
        print("-" * 100)


class VelocitySmoother:
    """增强版速度平滑器"""
    
    def __init__(self, alpha=0.25):
        self.alpha = alpha
        self.prev_linear = 0.0
        self.prev_angular = 0.0
        self.velocity_history = deque(maxlen=5)
    
    def smooth(self, target_linear: float, target_angular: float) -> Tuple[float, float]:
        """平滑速度变化"""
        # 基础平滑
        smooth_linear = self.alpha * self.prev_linear + (1 - self.alpha) * target_linear
        smooth_angular = self.alpha * self.prev_angular + (1 - self.alpha) * target_angular
        
        # 记录历史
        self.velocity_history.append((smooth_linear, smooth_angular))
        
        # 额外的摇晃检测
        if len(self.velocity_history) >= 4:
            angular_velocities = [v[1] for v in self.velocity_history]
            
            # 检测高频振荡
            oscillation_count = 0
            for i in range(1, len(angular_velocities)):
                if angular_velocities[i] * angular_velocities[i-1] < 0:
                    oscillation_count += 1
            
            if oscillation_count >= 2:
                # 检测到振荡，强制平滑
                smooth_angular *= 0.5
        
        self.prev_linear = smooth_linear
        self.prev_angular = smooth_angular
        
        return smooth_linear, smooth_angular


class OSGTObjectManager:
    """OSGT物体管理器"""
    
    def __init__(self, system):
        self.system = system
        self.collected_objects = set()
    
    def mark_object_collected(self, object_name: str, osgt_type: str):
        """标记物体已收集"""
        self.collected_objects.add(object_name)
        
        if osgt_type == "sweepable":
            self._handle_sweepable_collection(object_name)
        elif osgt_type == "graspable":
            self._handle_graspable_collection(object_name)
    
    def _handle_sweepable_collection(self, object_name: str):
        """处理S类物体收集"""
        for obj in self.system.sweepable_objects:
            if obj.name == object_name:
                underground_pos = obj.get_world_pose()[0]
                if not isinstance(underground_pos, np.ndarray):
                    underground_pos = np.array(underground_pos)
                
                underground_pos = underground_pos.copy()
                underground_pos[2] = -10.0
                obj.set_world_pose(underground_pos, obj.get_world_pose()[1])
                print(f"✅ S类物体 {object_name} 已消失")
                break
    
    def _handle_graspable_collection(self, object_name: str):
        """处理G类物体收集"""
        for obj in self.system.graspable_objects:
            if obj.name == object_name:
                underground_pos = obj.get_world_pose()[0]
                if not isinstance(underground_pos, np.ndarray):
                    underground_pos = np.array(underground_pos)
                
                underground_pos = underground_pos.copy()
                underground_pos[2] = -10.0
                obj.set_world_pose(underground_pos, obj.get_world_pose()[1])
                print(f"✅ G类物体 {object_name} 已消失")
                break
    
    def is_object_collected(self, object_name: str) -> bool:
        """检查物体是否已收集"""
        return object_name in self.collected_objects
    
    def get_nearest_uncollected_object(self, robot_pos: np.ndarray) -> Tuple[Optional[object], str]:
        """获取最近的未收集物体"""
        # 确保robot_pos是numpy数组
        if not isinstance(robot_pos, np.ndarray):
            robot_pos = np.array(robot_pos)
        
        nearest_obj = None
        nearest_dist = float('inf')
        nearest_type = ""
        
        # 检查S类物体
        for obj in self.system.sweepable_objects:
            if not self.is_object_collected(obj.name):
                obj_pos = obj.get_world_pose()[0]
                if not isinstance(obj_pos, np.ndarray):
                    obj_pos = np.array(obj_pos)
                
                if obj_pos[2] > -5.0:
                    dist = np.linalg.norm(obj_pos[:2] - robot_pos[:2])
                    if dist < nearest_dist:
                        nearest_dist = dist
                        nearest_obj = obj
                        nearest_type = "sweepable"
        
        # 检查G类物体
        for obj in self.system.graspable_objects:
            if not self.is_object_collected(obj.name):
                obj_pos = obj.get_world_pose()[0]
                if not isinstance(obj_pos, np.ndarray):
                    obj_pos = np.array(obj_pos)
                
                if obj_pos[2] > -5.0:
                    dist = np.linalg.norm(obj_pos[:2] - robot_pos[:2])
                    if dist < nearest_dist:
                        nearest_dist = dist
                        nearest_obj = obj
                        nearest_type = "graspable"
        
        return nearest_obj, nearest_type
    
    def get_object_positions_for_avoidance(self) -> List[np.ndarray]:
        """获取用于避障的物体位置"""
        positions = []
        
        # 添加障碍物位置
        for obj in self.system.obstacles_objects:
            obj_pos = obj.get_world_pose()[0]
            if not isinstance(obj_pos, np.ndarray):
                obj_pos = np.array(obj_pos)
            if obj_pos[2] > -5.0:
                positions.append(obj_pos[:2])
        
        # 添加未收集的物体位置
        for obj in self.system.sweepable_objects:
            if not self.is_object_collected(obj.name):
                obj_pos = obj.get_world_pose()[0]
                if not isinstance(obj_pos, np.ndarray):
                    obj_pos = np.array(obj_pos)
                if obj_pos[2] > -5.0:
                    positions.append(obj_pos[:2])
        
        for obj in self.system.graspable_objects:
            if not self.is_object_collected(obj.name):
                obj_pos = obj.get_world_pose()[0]
                if not isinstance(obj_pos, np.ndarray):
                    obj_pos = np.array(obj_pos)
                if obj_pos[2] > -5.0:
                    positions.append(obj_pos[:2])
        
        return positions