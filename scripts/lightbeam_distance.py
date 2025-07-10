#!/usr/bin/env python3
"""
OSGT简化版LightBeam避障系统
6个传感器双层配置：前方+左前45度+右前45度
简单直接的避障算法，支持后退脱困
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
    """OSGT简化版LightBeam避障系统（6传感器双层配置+脱困功能）"""
    
    def __init__(self, config):
        self.config = config
        self.robot_prim_path = config.PATHS["robot_prim_path"]
        
        # 简化的传感器配置（6个传感器双层）
        self.sensors = {
            # 底盘层传感器
            "front_bottom": {"layer": "bottom", "direction": "front", "angle": 0},
            "left_bottom": {"layer": "bottom", "direction": "left", "angle": -45},  # 左前45度
            "right_bottom": {"layer": "bottom", "direction": "right", "angle": 45}, # 右前45度
            
            # 机械臂层传感器  
            "front_top": {"layer": "top", "direction": "front", "angle": 0},
            "left_top": {"layer": "top", "direction": "left", "angle": -45},        # 左前45度
            "right_top": {"layer": "top", "direction": "right", "angle": 45}        # 右前45度
        }
        
        # 简化的避障参数
        self.min_valid_distance = 0.15  # 最小有效距离
        self.max_valid_distance = 6.0   # 最大有效距离
        self.safe_distance = 1.5        # 安全距离
        self.warning_distance = 2.5     # 警告距离
        
        # 脱困参数
        self.stuck_threshold = 1.0       # 被困检测阈值
        self.backup_distance = 2.0       # 后退距离（增加到2米）
        self.escape_turn_angle = 75.0    # 脱困转向角度（默认75度，动态调整）
        
        # 速度控制参数
        self.max_linear_speed = config.ROBOT_CONTROL["max_linear_velocity"]
        self.max_angular_speed = config.ROBOT_CONTROL["max_angular_velocity"]
        
        # 传感器接口
        self.lightbeam_interface = None
        self.timeline = None
        
        # 简化的距离数据缓存（只保留3个值）
        self.distance_buffer = {}
        for sensor_name in self.sensors.keys():
            self.distance_buffer[sensor_name] = deque(maxlen=3)
        
        # 当前距离数据
        self.current_distances = {}
        self.last_display_time = 0
        
        # 简化的速度平滑
        self.prev_linear = 0.0
        self.prev_angular = 0.0
        self.smooth_factor = 0.3
        
        # 脱困状态管理
        self.escape_mode = False
        self.escape_stage = "none"  # "none", "backing", "turning", "checking"
        self.escape_start_time = 0
        self.escape_backup_time = 4.0    # 后退时间（增加到4秒）
        self.escape_turn_time = 5.0      # 转向时间（增加到5秒，确保完成转向）
        self.escape_check_time = 1.0     # 检查时间（秒）
        self.escape_direction = 1        # 1=左转，-1=右转
        self.stuck_count = 0             # 被困计数器
        self.stuck_detection_threshold = 5  # 连续检测到被困的次数阈值（减少到5）
        
        # 初始化
        self.initialize()
    
    def initialize(self):
        """初始化传感器接口"""
        try:
            self.lightbeam_interface = _range_sensor.acquire_lightbeam_sensor_interface()
            import omni.timeline
            self.timeline = omni.timeline.get_timeline_interface()
            print("✅ LightBeam传感器接口初始化成功（6传感器双层配置+脱困功能）")
            return True
        except Exception as e:
            print(f"❌ LightBeam传感器接口初始化失败: {e}")
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
                    if beam_hit[i] and linear_depth[i] > self.min_valid_distance:
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
                # 简单平均滤波
                buffer_data = list(self.distance_buffer[sensor_name])
                self.current_distances[sensor_name] = sum(buffer_data) / len(buffer_data)
            else:
                self.current_distances[sensor_name] = None
    
    def get_direction_distances(self) -> Tuple[float, float, float]:
        """获取三个方向的合成距离（双层传感器取最小值）"""
        # 前方距离：取前上前下最小值
        front_distances = []
        if self.current_distances.get("front_bottom") is not None:
            front_distances.append(self.current_distances["front_bottom"])
        if self.current_distances.get("front_top") is not None:
            front_distances.append(self.current_distances["front_top"])
        front_dist = min(front_distances) if front_distances else float('inf')
        
        # 左前45度距离：取左上左下最小值
        left_distances = []
        if self.current_distances.get("left_bottom") is not None:
            left_distances.append(self.current_distances["left_bottom"])
        if self.current_distances.get("left_top") is not None:
            left_distances.append(self.current_distances["left_top"])
        left_dist = min(left_distances) if left_distances else float('inf')
        
        # 右前45度距离：取右上右下最小值
        right_distances = []
        if self.current_distances.get("right_bottom") is not None:
            right_distances.append(self.current_distances["right_bottom"])
        if self.current_distances.get("right_top") is not None:
            right_distances.append(self.current_distances["right_top"])
        right_dist = min(right_distances) if right_distances else float('inf')
        
        return front_dist, left_dist, right_dist
    
    def check_if_stuck(self, front_dist: float, left_dist: float, right_dist: float) -> bool:
        """检查是否被困住（包括贴墙、夹缝和转向困难情况）"""
        front_stuck = front_dist < self.stuck_threshold
        left_stuck = left_dist < self.stuck_threshold
        right_stuck = right_dist < self.stuck_threshold
        
        # 被困条件1：前方危险 AND (左方危险 OR 右方危险)
        condition1 = front_stuck and (left_stuck or right_stuck)
        
        # 被困条件2：贴墙情况 - 一侧非常近（贴墙卡住）
        wall_stuck_threshold = 0.6  # 贴墙检测阈值
        condition2 = (left_dist < wall_stuck_threshold and front_dist < 1.2) or \
                    (right_dist < wall_stuck_threshold and front_dist < 1.2)
        
        # 被困条件3：夹缝困境 - 左右两侧都危险，无法转弯
        gap_stuck_threshold = 0.9  # 夹缝检测阈值
        condition3 = (left_dist < gap_stuck_threshold and right_dist < gap_stuck_threshold)
        
        # 被困条件4：转向困难 - 前方警告级别 + 一侧危险（虽然另一侧安全但转不过去）
        turn_difficulty_threshold = 1.8  # 前方转向困难阈值
        condition4 = (front_dist < turn_difficulty_threshold) and \
                    (left_stuck or right_stuck)
        
        # 任一条件满足即被困
        if condition1 or condition2 or condition3 or condition4:
            self.stuck_count += 1
            if self.stuck_count >= self.stuck_detection_threshold:
                return True
        else:
            self.stuck_count = 0
            
        return False
    
    def execute_escape_sequence(self, front_dist: float, left_dist: float, right_dist: float) -> Tuple[float, float]:
        """执行脱困序列"""
        current_time = time.time()
        elapsed_time = current_time - self.escape_start_time
        
        if self.escape_stage == "backing":
            # 第一阶段：后退
            if elapsed_time < self.escape_backup_time:
                print(f"   ⬅️ 后退中... ({elapsed_time:.1f}/{self.escape_backup_time:.1f}s)")
                return -0.4, 0.0  # 后退速度稍快一些
            else:
                # 切换到转向阶段
                self.escape_stage = "turning"
                self.escape_start_time = current_time
                print(f"   🔄 开始转向{self.escape_turn_angle}度...")
                
        elif self.escape_stage == "turning":
            # 第二阶段：转向（增强版，确保完成目标角度）
            if elapsed_time < self.escape_turn_time:
                # 计算需要的角速度，确保在时间内完成转向
                required_angular_speed = math.radians(self.escape_turn_angle) / self.escape_turn_time
                # 增加安全系数，确保转向充分
                actual_angular_speed = required_angular_speed * 10  # 增加100%的速度
                # 限制在最大角速度范围内
                actual_angular_speed = min(actual_angular_speed, self.max_angular_speed * 8)
                
                turn_vel = self.escape_direction * actual_angular_speed
                print(f"   🔄 转向中... ({elapsed_time:.1f}/{self.escape_turn_time:.1f}s) 角速度: {math.degrees(actual_angular_speed):.1f}°/s")
                return 0.0, turn_vel
            else:
                # 切换到检查阶段
                self.escape_stage = "checking"
                self.escape_start_time = current_time
                print("   👀 检查脱困效果...")
                
        elif self.escape_stage == "checking":
            # 第三阶段：检查是否脱困
            if elapsed_time < self.escape_check_time:
                return 0.0, 0.0  # 停止，观察环境
            else:
                # 检查是否成功脱困
                if front_dist > self.safe_distance and min(left_dist, right_dist) > 0.8:
                    print("   ✅ 脱困成功！")
                    self.escape_mode = False
                    self.escape_stage = "none"
                    self.stuck_count = 0
                else:
                    print("   ⚠️ 脱困失败，重新开始...")
                    self.escape_stage = "backing"
                    self.escape_start_time = current_time
                    # 切换脱困方向
                    self.escape_direction *= -1
                    print(f"   🔄 切换到{'左' if self.escape_direction > 0 else '右'}侧脱困")
        
        return 0.0, 0.0
    
    def calculate_avoidance_command(self, target_linear: float, target_angular: float, 
                                   current_pos: np.ndarray, target_pos: np.ndarray) -> Tuple[float, float]:
        """简化的避障算法（支持脱困功能）"""
        # 更新传感器数据
        self.update_all_sensor_data()
        
        # 获取三个方向的合成距离
        front_dist, left_dist, right_dist = self.get_direction_distances()
        
        # 检查是否需要脱困
        is_stuck = self.check_if_stuck(front_dist, left_dist, right_dist)
        if not self.escape_mode and is_stuck:
            # 强制开始脱困序列
            # 判断被困类型
            if front_dist < self.stuck_threshold and (left_dist < self.stuck_threshold or right_dist < self.stuck_threshold):
                print("🚨 检测到前方+侧面被困，启动脱困序列...")
                self.escape_turn_angle = 75.0
            elif (left_dist < 0.6 and front_dist < 1.2) or (right_dist < 0.6 and front_dist < 1.2):
                print("🚨 检测到贴墙被困，启动脱困序列...")
                self.escape_turn_angle = 75.0
            elif left_dist < 0.9 and right_dist < 0.9:
                print("🚨 检测到夹缝困境，启动脱困序列...")
                self.escape_turn_angle = 90.0
            elif front_dist < 1.8 and (left_dist < self.stuck_threshold or right_dist < self.stuck_threshold):
                print("🚨 检测到转向困难，启动脱困序列...")
                self.escape_turn_angle = 85.0  # 转向困难需要较大角度
            else:
                print("🚨 检测到被困，启动脱困序列...")
                self.escape_turn_angle = 75.0
            
            self.escape_mode = True
            self.escape_stage = "backing"
            self.escape_start_time = time.time()
            
            # 选择脱困方向（向较安全的一侧）
            if left_dist > right_dist:
                self.escape_direction = 1  # 左转
                print(f"   📍 选择左侧脱困（左侧距离: {left_dist:.2f}m > 右侧距离: {right_dist:.2f}m）")
            else:
                self.escape_direction = -1  # 右转
                print(f"   📍 选择右侧脱困（右侧距离: {right_dist:.2f}m > 左侧距离: {left_dist:.2f}m）")
            
            print(f"   🔄 设置转向角度: {self.escape_turn_angle}度")
        
        # 如果正在脱困，执行脱困序列
        if self.escape_mode:
            linear_vel, angular_vel = self.execute_escape_sequence(front_dist, left_dist, right_dist)
        else:
            # 正常避障逻辑
            linear_vel, angular_vel = self._normal_avoidance(target_linear, target_angular, 
                                                           front_dist, left_dist, right_dist)
        
        # 限制速度范围
        linear_vel = np.clip(linear_vel, -self.max_linear_speed, self.max_linear_speed)
        angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)
        
        # 简单的速度平滑
        smooth_linear = self.smooth_factor * self.prev_linear + (1 - self.smooth_factor) * linear_vel
        smooth_angular = self.smooth_factor * self.prev_angular + (1 - self.smooth_factor) * angular_vel
        
        self.prev_linear = smooth_linear
        self.prev_angular = smooth_angular
        
        # 显示状态
        self._display_sensor_status(front_dist, left_dist, right_dist)
        
        return smooth_linear, smooth_angular
    
    def _normal_avoidance(self, target_linear: float, target_angular: float,
                         front_dist: float, left_dist: float, right_dist: float) -> Tuple[float, float]:
        """正常避障逻辑"""
        output_linear = target_linear
        output_angular = target_angular
        
        if front_dist < self.safe_distance:
            # 前方有障碍物
            if front_dist < 0.8:  # 非常近，紧急避障
                output_linear = 0.1  # 几乎停止，但保持微小前进
                
                # 选择较安全的方向转向
                if left_dist > right_dist:
                    output_angular = 0.8  # 左转
                elif right_dist > left_dist:
                    output_angular = -0.8  # 右转
                else:
                    output_angular = 0.8  # 默认左转
                    
            else:  # 中等距离，减速+转向
                # 根据距离调整速度
                speed_factor = (front_dist - 0.8) / (self.safe_distance - 0.8)
                output_linear = target_linear * max(0.3, speed_factor)
                
                # 温和转向
                if left_dist > right_dist:
                    output_angular = 0.4
                elif right_dist > left_dist:
                    output_angular = -0.4
                else:
                    output_angular = 0.4
        
        else:
            # 前方安全，检查侧面
            side_adjustment = 0.0
            
            # 左侧调整
            if left_dist < self.warning_distance:
                side_adjustment -= 0.2 * (self.warning_distance - left_dist) / self.warning_distance
                
            # 右侧调整
            if right_dist < self.warning_distance:
                side_adjustment += 0.2 * (self.warning_distance - right_dist) / self.warning_distance
                
            output_angular = target_angular + side_adjustment
        
        return output_linear, output_angular
    
    def _display_sensor_status(self, front_dist: float, left_dist: float, right_dist: float):
        """显示传感器状态（增强版，显示脱困状态）"""
        current_time = time.time()
        
        if current_time - self.last_display_time < 2.0:
            return
        
        self.last_display_time = current_time
        
        # 状态判断
        def get_status(dist):
            if dist < 0.8:
                return "危险"
            elif dist < self.safe_distance:
                return "警告"
            elif dist < self.warning_distance:
                return "注意"
            else:
                return "安全"
        
        # 基本状态显示
        status_msg = (f"📡 LightBeam状态: 前方{front_dist:.2f}m({get_status(front_dist)}) | "
                     f"左前45°{left_dist:.2f}m({get_status(left_dist)}) | "
                     f"右前45°{right_dist:.2f}m({get_status(right_dist)})")
        
        # 脱困状态显示
        if self.escape_mode:
            status_msg += f" | 🚨 脱困模式: {self.escape_stage}"
        elif self.stuck_count > 0:
            # 显示被困类型
            if front_dist < self.stuck_threshold and (left_dist < self.stuck_threshold or right_dist < self.stuck_threshold):
                stuck_type = "前方+侧面"
            elif (left_dist < 0.6 and front_dist < 1.2) or (right_dist < 0.6 and front_dist < 1.2):
                stuck_type = "贴墙"
            elif left_dist < 0.9 and right_dist < 0.9:
                stuck_type = "夹缝困境"
            elif front_dist < 1.8 and (left_dist < self.stuck_threshold or right_dist < self.stuck_threshold):
                stuck_type = "转向困难"
            else:
                stuck_type = "其他"
            status_msg += f" | ⚠️ 被困检测({stuck_type}): {self.stuck_count}/{self.stuck_detection_threshold}"
        
        print(status_msg)


class OSGTObjectManager:
    """OSGT物体管理器（保持不变）"""
    
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