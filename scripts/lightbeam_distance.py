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

class OSGTLightBeamAvoidanceSystem:
    """OSGT四类物体LightBeam避障系统"""
    
    def __init__(self, config):
        self.config = config
        self.robot_prim_path = config.PATHS["robot_prim_path"]
        
        # 传感器配置
        self.sensors = {
            "front_bottom": {"layer": "bottom", "direction": "front", "priority": 1.0},
            "back_bottom": {"layer": "bottom", "direction": "back", "priority": 0.8},
            "left_bottom": {"layer": "bottom", "direction": "left", "priority": 0.9},
            "right_bottom": {"layer": "bottom", "direction": "right", "priority": 0.9},
            "front_top": {"layer": "top", "direction": "front", "priority": 0.8},
            "back_top": {"layer": "top", "direction": "back", "priority": 0.6},
            "left_top": {"layer": "top", "direction": "left", "priority": 0.7},
            "right_top": {"layer": "top", "direction": "right", "priority": 0.7}
        }
        
        # 避障参数
        self.critical_distance = 2
        self.warning_distance = 4
        self.safe_distance = 6
        
        # 速度控制参数
        self.max_linear_speed = config.ROBOT_CONTROL["max_linear_velocity"]
        self.max_angular_speed = config.ROBOT_CONTROL["max_angular_velocity"]
        
        # 传感器接口
        self.lightbeam_interface = None
        self.timeline = None
        
        # 距离数据缓存
        self.distance_buffer = {}
        for sensor_name in self.sensors.keys():
            self.distance_buffer[sensor_name] = deque(maxlen=5)
        
        # 避障状态
        self.avoidance_active = False
        self.last_avoidance_time = 0
        self.avoidance_direction = 0
        
        # 速度平滑
        self.velocity_smoother = VelocitySmoother(alpha=0.15)
        
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
        """获取单个传感器的距离数据"""
        if not self.timeline or not self.timeline.is_playing():
            return None
        
        sensor_path = f"{self.robot_prim_path}/create3_robot/create_3/base_link/{sensor_name}"
        
        try:
            linear_depth = self.lightbeam_interface.get_linear_depth_data(sensor_path)
            beam_hit = self.lightbeam_interface.get_beam_hit_data(sensor_path)
            
            if linear_depth is not None and beam_hit is not None:
                valid_distances = []
                for i in range(len(linear_depth)):
                    if beam_hit[i] and linear_depth[i] > 0.1:
                        valid_distances.append(linear_depth[i])
                
                if valid_distances:
                    return min(valid_distances)
            
            return None
            
        except Exception:
            return None
    
    def update_sensor_data(self):
        """更新所有传感器数据"""
        for sensor_name in self.sensors.keys():
            distance = self.get_sensor_distance(sensor_name)
            if distance is not None:
                self.distance_buffer[sensor_name].append(distance)
    
    def get_smoothed_distance(self, sensor_name: str) -> Optional[float]:
        """获取平滑后的距离数据"""
        buffer = self.distance_buffer[sensor_name]
        if len(buffer) == 0:
            return None
        
        # 使用中位数滤波减少噪声
        distances = list(buffer)
        distances.sort()
        return distances[len(distances) // 2]
    
    def get_directional_distances(self) -> Dict[str, float]:
        """获取四个方向的综合距离"""
        directions = {"front": [], "back": [], "left": [], "right": []}
        
        for sensor_name, sensor_info in self.sensors.items():
            distance = self.get_smoothed_distance(sensor_name)
            if distance is not None:
                direction = sensor_info["direction"]
                priority = sensor_info["priority"]
                directions[direction].append(distance * priority)
        
        # 计算每个方向的最小距离
        result = {}
        for direction, distances in directions.items():
            if distances:
                result[direction] = min(distances)
            else:
                result[direction] = float('inf')
        
        return result
    
    def calculate_avoidance_command(self, target_linear: float, target_angular: float, 
                                   current_pos: np.ndarray, target_pos: np.ndarray) -> Tuple[float, float]:
        """计算避障后的运动命令"""
        # 更新传感器数据
        self.update_sensor_data()
        
        # 获取方向距离
        dir_distances = self.get_directional_distances()
        
        # 计算到目标的方向
        direction_to_target = target_pos[:2] - current_pos[:2]
        distance_to_target = np.linalg.norm(direction_to_target)
        
        if distance_to_target > 0.1:
            direction_to_target = direction_to_target / distance_to_target
        else:
            direction_to_target = np.array([1.0, 0.0])
        
        # 判断是否需要避障
        need_avoidance = False
        min_distance = float('inf')
        
        for direction, distance in dir_distances.items():
            if distance < self.warning_distance:
                need_avoidance = True
                min_distance = min(min_distance, distance)
        
        if not need_avoidance:
            # 无需避障，返回平滑后的目标速度
            return self.velocity_smoother.smooth(target_linear, target_angular)
        
        # 计算避障策略
        avoidance_linear, avoidance_angular = self._calculate_avoidance_strategy(
            dir_distances, target_linear, target_angular, direction_to_target
        )
        
        # 根据最小距离调整速度
        if min_distance < self.critical_distance:
            # 紧急避障
            speed_factor = 0.1
        elif min_distance < self.warning_distance:
            # 警告区域
            speed_factor = 0.3 + 0.4 * (min_distance - self.critical_distance) / (self.warning_distance - self.critical_distance)
        else:
            speed_factor = 1.0
        
        final_linear = avoidance_linear * speed_factor
        final_angular = avoidance_angular
        
        # 显示避障信息
        if self.config.DEBUG["show_navigation_progress"]:
            self._display_avoidance_status(dir_distances, min_distance, speed_factor)
        
        return self.velocity_smoother.smooth(final_linear, final_angular)
    
    def _calculate_avoidance_strategy(self, dir_distances: Dict[str, float], 
                                    target_linear: float, target_angular: float,
                                    direction_to_target: np.ndarray) -> Tuple[float, float]:
        """计算避障策略"""
        front_dist = dir_distances["front"]
        back_dist = dir_distances["back"]
        left_dist = dir_distances["left"]
        right_dist = dir_distances["right"]
        
        # 基础避障逻辑
        avoidance_linear = target_linear
        avoidance_angular = target_angular
        
        # 前方障碍物处理
        if front_dist < self.warning_distance:
            if front_dist < self.critical_distance:
                # 紧急停止或后退
                avoidance_linear = -0.2
            else:
                # 减速并转向
                avoidance_linear = target_linear * 0.3
                
            # 选择转向方向
            if left_dist > right_dist:
                avoidance_angular = 1.0
            else:
                avoidance_angular = -1.0
        
        # 后方障碍物处理
        if back_dist < self.warning_distance and avoidance_linear < 0:
            avoidance_linear = 0.0
            avoidance_angular = 1.5 if left_dist > right_dist else -1.5
        
        # 左右障碍物处理
        if left_dist < self.warning_distance:
            avoidance_angular -= 0.5
        if right_dist < self.warning_distance:
            avoidance_angular += 0.5
        
        # 限制速度范围
        avoidance_linear = np.clip(avoidance_linear, -self.max_linear_speed, self.max_linear_speed)
        avoidance_angular = np.clip(avoidance_angular, -self.max_angular_speed, self.max_angular_speed)
        
        return avoidance_linear, avoidance_angular
    
    def _display_avoidance_status(self, dir_distances: Dict[str, float], 
                                 min_distance: float, speed_factor: float):
        """显示避障状态"""
        status_symbols = {
            "front": "↑", "back": "↓", "left": "←", "right": "→"
        }
        
        status_line = "避障: "
        for direction, distance in dir_distances.items():
            if distance < self.critical_distance:
                status = "危险"
            elif distance < self.warning_distance:
                status = "警告"
            else:
                status = "安全"
            
            if distance != float('inf'):
                status_line += f"{status_symbols[direction]}{distance:.2f}m({status}) "
        
        print(f"{status_line}| 最小距离: {min_distance:.2f}m | 速度: {speed_factor:.2f}")
    
    def check_object_collision(self, object_position: np.ndarray, 
                              robot_position: np.ndarray) -> bool:
        """检查是否与物体发生碰撞"""
        relative_pos = object_position[:2] - robot_position[:2]
        distance = np.linalg.norm(relative_pos)
        
        if distance > 3.0:
            return False
        
        # 判断物体在哪个方向
        robot_yaw = 0.0
        
        # 将相对位置转换到机器人坐标系
        cos_yaw = np.cos(robot_yaw)
        sin_yaw = np.sin(robot_yaw)
        
        local_x = relative_pos[0] * cos_yaw + relative_pos[1] * sin_yaw
        local_y = -relative_pos[0] * sin_yaw + relative_pos[1] * cos_yaw
        
        # 判断方向并检查对应传感器
        if local_x > 0.5:
            sensor_dist = self.get_smoothed_distance("front_bottom")
            if sensor_dist and sensor_dist < distance + 0.3:
                return True
        elif local_x < -0.5:
            sensor_dist = self.get_smoothed_distance("back_bottom")
            if sensor_dist and sensor_dist < distance + 0.3:
                return True
        
        if local_y > 0.5:
            sensor_dist = self.get_smoothed_distance("left_bottom")
            if sensor_dist and sensor_dist < distance + 0.3:
                return True
        elif local_y < -0.5:
            sensor_dist = self.get_smoothed_distance("right_bottom")
            if sensor_dist and sensor_dist < distance + 0.3:
                return True
        
        return False


class VelocitySmoother:
    """速度平滑器"""
    
    def __init__(self, alpha=0.15):
        self.alpha = alpha
        self.prev_linear = 0.0
        self.prev_angular = 0.0
    
    def smooth(self, target_linear: float, target_angular: float) -> Tuple[float, float]:
        """平滑速度变化"""
        smooth_linear = self.alpha * self.prev_linear + (1 - self.alpha) * target_linear
        smooth_angular = self.alpha * self.prev_angular + (1 - self.alpha) * target_angular
        
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
        
        # 根据类型处理物体消失
        if osgt_type == "sweepable":
            self._handle_sweepable_collection(object_name)
        elif osgt_type == "graspable":
            self._handle_graspable_collection(object_name)
    
    def _handle_sweepable_collection(self, object_name: str):
        """处理S类物体收集"""
        for obj in self.system.sweepable_objects:
            if obj.name == object_name:
                underground_pos = obj.get_world_pose()[0].copy()
                underground_pos[2] = -10.0
                obj.set_world_pose(underground_pos, obj.get_world_pose()[1])
                break
    
    def _handle_graspable_collection(self, object_name: str):
        """处理G类物体收集"""
        for obj in self.system.graspable_objects:
            if obj.name == object_name:
                underground_pos = obj.get_world_pose()[0].copy()
                underground_pos[2] = -10.0
                obj.set_world_pose(underground_pos, obj.get_world_pose()[1])
                break
    
    def is_object_collected(self, object_name: str) -> bool:
        """检查物体是否已收集"""
        return object_name in self.collected_objects
    
    def get_nearest_uncollected_object(self, robot_pos: np.ndarray) -> Tuple[Optional[object], str]:
        """获取最近的未收集物体"""
        nearest_obj = None
        nearest_dist = float('inf')
        nearest_type = ""
        
        # 检查S类物体
        for obj in self.system.sweepable_objects:
            if not self.is_object_collected(obj.name):
                obj_pos = obj.get_world_pose()[0]
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
                if obj_pos[2] > -5.0:
                    dist = np.linalg.norm(obj_pos[:2] - robot_pos[:2])
                    if dist < nearest_dist:
                        nearest_dist = dist
                        nearest_obj = obj
                        nearest_type = "graspable"
        
        return nearest_obj, nearest_type