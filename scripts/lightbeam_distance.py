#!/usr/bin/env python3
"""
OSGT LightBeam距离检测和避障系统 - 修复版
附着在Create-3机械臂上的光束传感器，用于检测O类障碍物并执行三级避障
"""

import numpy as np
import time
import math
from typing import List, Tuple, Dict, Optional, Any
from collections import deque
from enum import Enum

# Isaac Sim API
import omni
import omni.timeline
from pxr import Gf, UsdPhysics, Sdf
from isaacsim.core.utils.extensions import enable_extension
import omni.graph.core as og


class OSGTAvoidanceLevel(Enum):
    """OSGT避障级别枚举"""
    SAFE = "safe"                    # 安全距离
    CAUTION = "caution"             # 较近距离，需要注意
    DANGER = "danger"               # 即将碰撞，紧急避障


class OSGTLightBeamSensorSystem:
    """OSGT LightBeam光束传感器系统 - 修复版"""
    
    def __init__(self, config: Dict[str, Any], world, robot_prim_path: str):
        self.config = config
        self.world = world
        self.robot_prim_path = robot_prim_path
        
        # 从配置读取参数
        lightbeam_config = config.LIGHTBEAM_CONFIG
        self.sensor_configs = lightbeam_config["sensors"]
        
        # 获取当前物体类型的距离阈值
        self.current_object_type = "environment"  # 默认环境类型
        self.distance_thresholds = lightbeam_config["distance_thresholds"]["environment"]
        self.avoidance_params = lightbeam_config["avoidance_parameters"]
        self.visualization_enabled = lightbeam_config["enable_visualization"]
        
        # LightBeam传感器接口
        self.lightbeam_interface = None
        self.timeline = None
        self.sensor_paths = []
        self.sensor_data = {}
        
        # 避障状态
        self.current_avoidance_level = OSGTAvoidanceLevel.SAFE
        self.obstacle_directions = []
        self.min_distance = float('inf')
        self.raw_distances = []  # 新增：存储原始距离数据
        self.avoidance_velocity_modifier = (1.0, 1.0)  # (linear_factor, angular_factor)
        
        # 避障历史和平滑
        self.avoidance_history = deque(maxlen=10)
        self.last_avoidance_command = (0.0, 0.0)
        
        # 统计数据
        self.detection_stats = {
            'total_detections': 0,
            'safe_detections': 0,
            'caution_detections': 0,
            'danger_detections': 0,
            'avoidance_activations': 0,
            'min_distance_recorded': float('inf'),
            'data_read_failures': 0,
            'successful_reads': 0
        }
        
        print(f"🔦 OSGT LightBeam系统初始化: {len(self.sensor_configs)}个传感器")
    
    def set_object_type_context(self, object_type: str):
        """设置当前处理的物体类型，调整距离阈值"""
        self.current_object_type = object_type
        
        lightbeam_config = self.config.LIGHTBEAM_CONFIG
        if object_type in ["sweepable", "graspable", "task_areas"]:
            # S/G/T类物体使用精确距离阈值
            self.distance_thresholds = lightbeam_config["distance_thresholds"]["sgt_objects"]
            if self.config.DEBUG["show_lightbeam_status"]:
                print(f"🎯 LightBeam切换到S/G/T物体模式: {object_type}")
        else:
            # 环境/O类障碍物使用环境距离阈值
            self.distance_thresholds = lightbeam_config["distance_thresholds"]["environment"]
            if self.config.DEBUG["show_lightbeam_status"]:
                print(f"🏠 LightBeam切换到环境/障碍物模式: {object_type}")
    
    def initialize_sensors(self) -> bool:
        """初始化所有LightBeam传感器"""
        try:
            print("📡 初始化OSGT LightBeam传感器...")
            
            # 启用PhysX传感器扩展
            enable_extension("isaacsim.sensors.physx")
            
            # 获取timeline接口
            self.timeline = omni.timeline.get_timeline_interface()
            
            # 获取LightBeam接口（使用正确的方法）
            from isaacsim.sensors.physx import _range_sensor
            self.lightbeam_interface = _range_sensor.acquire_lightbeam_sensor_interface()
            
            if self.lightbeam_interface is None:
                print("❌ 无法获取LightBeam传感器接口")
                return False
            
            # 确保物理场景存在
            stage = self.world.stage
            physics_scene_path = "/World/physicsScene"
            if not stage.GetPrimAtPath(physics_scene_path).IsValid():
                UsdPhysics.Scene.Define(stage, Sdf.Path(physics_scene_path))
            
            # 为每个配置创建传感器
            for i, sensor_config in enumerate(self.sensor_configs):
                sensor_path = f"/World/LightBeam_Sensor_{i}"  # 修正路径
                success = self._create_lightbeam_sensor(sensor_path, sensor_config)
                
                if success:
                    self.sensor_paths.append(sensor_path)
                    self.sensor_data[sensor_path] = {
                        'config': sensor_config,
                        'last_detection': None,
                        'status': 'active'
                    }
                    print(f"   ✅ 传感器 {i}: {sensor_config['name']} -> {sensor_path}")
                else:
                    print(f"   ❌ 传感器 {i}: {sensor_config['name']} 创建失败")
                    return False
            
            # 设置可视化
            if self.visualization_enabled:
                self._setup_visualization()
            
            print(f"✅ OSGT LightBeam系统初始化完成: {len(self.sensor_paths)}个传感器")
            return True
            
        except Exception as e:
            print(f"❌ LightBeam传感器初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _create_lightbeam_sensor(self, sensor_path: str, sensor_config: Dict) -> bool:
        """创建单个LightBeam传感器"""
        try:
            # 传感器相对位置（从配置读取）
            relative_pos = sensor_config.get("relative_position", [0.0, 0.0, 0.5])
            relative_rot = sensor_config.get("relative_rotation", [0.0, 0.0, 0.0])
            
            # 初始位置（相对于世界原点）
            sensor_position = np.array(relative_pos)
            
            # 创建传感器方向四元数
            orientation_quat = self._euler_to_quaternion(relative_rot)
            
            # 传感器参数
            min_range = sensor_config.get("min_range", 0.1)
            max_range = sensor_config.get("max_range", 5.0)
            num_rays = sensor_config.get("num_rays", 5)
            curtain_length = sensor_config.get("curtain_length", 0.5)
            forward_axis = sensor_config.get("forward_axis", [1, 0, 0])
            
            # 创建LightBeam传感器
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateLightBeamSensor",
                path=sensor_path,
                parent=None,
                min_range=min_range,
                max_range=max_range,
                translation=Gf.Vec3d(sensor_position[0], sensor_position[1], sensor_position[2]),
                orientation=Gf.Quatd(orientation_quat[3], orientation_quat[0], orientation_quat[1], orientation_quat[2]),
                forward_axis=Gf.Vec3d(forward_axis[0], forward_axis[1], forward_axis[2]),
                num_rays=num_rays,
                curtain_length=curtain_length,
            )
            
            return result
            
        except Exception as e:
            print(f"创建LightBeam传感器失败: {e}")
            return False
    
    def _setup_visualization(self) -> bool:
        """设置光束可视化"""
        try:
            print("🎨 设置LightBeam可视化...")
            
            # 为每个传感器创建可视化
            for i, sensor_path in enumerate(self.sensor_paths):
                graph_path = f"/ActionGraph_LightBeam_{i}"
                
                try:
                    (action_graph, new_nodes, _, _) = og.Controller.edit(
                        {"graph_path": graph_path, "evaluator_name": "execution"},
                        {
                            og.Controller.Keys.CREATE_NODES: [
                                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                                ("IsaacReadLightBeam", "isaacsim.sensors.physx.IsaacReadLightBeam"),
                                ("DebugDrawRayCast", "isaacsim.util.debug_draw.DebugDrawRayCast"),
                            ],
                            og.Controller.Keys.SET_VALUES: [
                                ("IsaacReadLightBeam.inputs:lightbeamPrim", sensor_path),
                            ],
                            og.Controller.Keys.CONNECT: [
                                ("OnPlaybackTick.outputs:tick", "IsaacReadLightBeam.inputs:execIn"),
                                ("IsaacReadLightBeam.outputs:execOut", "DebugDrawRayCast.inputs:exec"),
                                ("IsaacReadLightBeam.outputs:beamOrigins", "DebugDrawRayCast.inputs:beamOrigins"),
                                ("IsaacReadLightBeam.outputs:beamEndPoints", "DebugDrawRayCast.inputs:beamEndPoints"),
                                ("IsaacReadLightBeam.outputs:numRays", "DebugDrawRayCast.inputs:numRays"),
                            ],
                        },
                    )
                    print(f"   ✅ 传感器 {i} 可视化设置完成")
                except Exception as e:
                    print(f"   ⚠️ 传感器 {i} 可视化设置失败: {e}")
            
            print("✅ LightBeam可视化设置完成")
            return True
            
        except Exception as e:
            print(f"⚠️ LightBeam可视化设置失败: {e}")
            return False
    
    def update_sensor_positions(self, robot_position: np.ndarray, robot_yaw: float):
        """更新传感器位置（跟随机器人）"""
        try:
            stage = self.world.stage
            
            for i, sensor_path in enumerate(self.sensor_paths):
                sensor_config = self.sensor_data[sensor_path]['config']
                
                # 计算传感器在机器人坐标系中的相对位置
                relative_pos = np.array(sensor_config.get("relative_position", [0.0, 0.0, 0.5]))
                relative_rot = sensor_config.get("relative_rotation", [0.0, 0.0, 0.0])
                
                # 考虑机器人朝向的位置变换
                cos_yaw = np.cos(robot_yaw)
                sin_yaw = np.sin(robot_yaw)
                
                # 旋转相对位置
                rotated_pos = np.array([
                    relative_pos[0] * cos_yaw - relative_pos[1] * sin_yaw,
                    relative_pos[0] * sin_yaw + relative_pos[1] * cos_yaw,
                    relative_pos[2]
                ])
                
                # 世界坐标位置
                world_position = robot_position + rotated_pos
                
                # 传感器方向（加上机器人朝向）
                sensor_yaw = robot_yaw + relative_rot[2]
                orientation_quat = self._euler_to_quaternion([relative_rot[0], relative_rot[1], sensor_yaw])
                
                # 更新传感器位置
                sensor_prim = stage.GetPrimAtPath(sensor_path)
                if sensor_prim.IsValid():
                    translate_attr = sensor_prim.GetAttribute("xformOp:translate")
                    if translate_attr:
                        translate_attr.Set(Gf.Vec3d(world_position[0], world_position[1], world_position[2]))
                    
                    orient_attr = sensor_prim.GetAttribute("xformOp:orient")
                    if orient_attr:
                        orient_attr.Set(Gf.Quatd(orientation_quat[3], orientation_quat[0], orientation_quat[1], orientation_quat[2]))
                        
        except Exception as e:
            if self.config.DEBUG["enable_debug_output"]:
                print(f"更新传感器位置失败: {e}")
    
    def get_distance_measurements(self) -> Dict[str, Any]:
        """获取所有传感器的距离测量数据 - 修复版"""
        measurements = {
            'min_distance': float('inf'),
            'raw_distances': [],
            'sensor_data': {},
            'obstacle_detected': False,
            'avoidance_level': OSGTAvoidanceLevel.SAFE,
            'obstacle_directions': [],
            'data_valid': False
        }
        
        try:
            # 确保仿真正在运行
            if not self.timeline or not self.timeline.is_playing():
                if self.config.DEBUG["show_lightbeam_status"]:
                    print("📡 LightBeam: 仿真未运行")
                return measurements
            
            if not self.lightbeam_interface:
                print("❌ LightBeam接口未初始化")
                return measurements
            
            all_distances = []
            raw_distances = []
            obstacle_directions = []
            valid_data_count = 0
            
            for sensor_path in self.sensor_paths:
                sensor_config = self.sensor_data[sensor_path]['config']
                
                try:
                    # 获取传感器数据（使用正确的方法）
                    linear_depth = self.lightbeam_interface.get_linear_depth_data(sensor_path)
                    beam_hit = self.lightbeam_interface.get_beam_hit_data(sensor_path)
                    hit_pos = self.lightbeam_interface.get_hit_pos_data(sensor_path)
                    
                    if linear_depth is not None and beam_hit is not None:
                        # 转换为numpy数组并确保类型正确
                        linear_depth = np.array(linear_depth)
                        beam_hit = np.array(beam_hit).astype(bool)
                        
                        if len(linear_depth) > 0 and len(beam_hit) > 0:
                            valid_distances = []
                            
                            for i in range(min(len(linear_depth), len(beam_hit))):
                                distance = linear_depth[i]
                                hit = beam_hit[i]
                                
                                # 记录所有原始距离数据
                                raw_distances.append({
                                    'sensor': sensor_config['name'],
                                    'beam': i,
                                    'distance': distance,
                                    'hit': hit
                                })
                                
                                if hit and 0.1 < distance < 100.0:  # 扩大有效范围
                                    valid_distances.append(distance)
                                    all_distances.append(distance)
                                    valid_data_count += 1
                                    
                                    # 计算障碍物方向
                                    if hit_pos is not None and len(hit_pos) > i:
                                        try:
                                            hit_position = hit_pos[i]
                                            robot_pos = self._get_robot_position()
                                            
                                            direction = np.array([hit_position[0] - robot_pos[0], 
                                                                hit_position[1] - robot_pos[1]])
                                            if np.linalg.norm(direction) > 0:
                                                direction = direction / np.linalg.norm(direction)
                                                obstacle_directions.append(direction)
                                        except:
                                            pass
                            
                            # 记录传感器数据
                            if valid_distances:
                                measurements['sensor_data'][sensor_path] = {
                                    'name': sensor_config['name'],
                                    'min_distance': min(valid_distances),
                                    'distances': valid_distances,
                                    'num_hits': len(valid_distances)
                                }
                            
                            self.detection_stats['successful_reads'] += 1
                            
                        else:
                            if self.config.DEBUG["show_lightbeam_status"]:
                                print(f"   {sensor_config['name']}: 空数据")
                    else:
                        self.detection_stats['data_read_failures'] += 1
                        if self.config.DEBUG["show_lightbeam_status"]:
                            print(f"   {sensor_config['name']}: 数据读取失败")
                            
                except Exception as e:
                    self.detection_stats['data_read_failures'] += 1
                    if self.config.DEBUG["show_lightbeam_status"]:
                        print(f"   {sensor_config['name']}: 异常 - {e}")
            
            # 设置原始距离数据
            measurements['raw_distances'] = raw_distances
            self.raw_distances = raw_distances
            
            # 计算整体最小距离
            if all_distances:
                measurements['min_distance'] = min(all_distances)
                measurements['obstacle_detected'] = True
                measurements['obstacle_directions'] = obstacle_directions
                measurements['data_valid'] = True
                
                # 确定避障级别（使用当前物体类型的阈值）
                min_dist = measurements['min_distance']
                if min_dist <= self.distance_thresholds['danger']:
                    measurements['avoidance_level'] = OSGTAvoidanceLevel.DANGER
                elif min_dist <= self.distance_thresholds['caution']:
                    measurements['avoidance_level'] = OSGTAvoidanceLevel.CAUTION
                else:
                    measurements['avoidance_level'] = OSGTAvoidanceLevel.SAFE
                
                # 更新统计
                self._update_detection_stats(measurements['avoidance_level'], min_dist)
            else:
                # 没有检测到有效距离
                measurements['data_valid'] = valid_data_count > 0
            
            return measurements
            
        except Exception as e:
            self.detection_stats['data_read_failures'] += 1
            if self.config.DEBUG["enable_debug_output"]:
                print(f"获取距离测量失败: {e}")
            return measurements
    
    def compute_avoidance_velocity(self, desired_linear_vel: float, desired_angular_vel: float) -> Tuple[float, float]:
        """计算避障速度修正"""
        try:
            # 获取最新的距离测量
            measurements = self.get_distance_measurements()
            
            if not measurements['obstacle_detected'] or not measurements['data_valid']:
                self.current_avoidance_level = OSGTAvoidanceLevel.SAFE
                return desired_linear_vel, desired_angular_vel
            
            min_distance = measurements['min_distance']
            avoidance_level = measurements['avoidance_level']
            obstacle_directions = measurements['obstacle_directions']
            
            self.current_avoidance_level = avoidance_level
            self.min_distance = min_distance
            self.obstacle_directions = obstacle_directions
            
            # 根据避障级别计算速度修正
            if avoidance_level == OSGTAvoidanceLevel.SAFE:
                return desired_linear_vel, desired_angular_vel
            
            elif avoidance_level == OSGTAvoidanceLevel.CAUTION:
                return self._compute_caution_avoidance(desired_linear_vel, desired_angular_vel, 
                                                     min_distance, obstacle_directions)
            
            elif avoidance_level == OSGTAvoidanceLevel.DANGER:
                return self._compute_danger_avoidance(desired_linear_vel, desired_angular_vel, 
                                                    min_distance, obstacle_directions)
            
        except Exception as e:
            print(f"计算避障速度失败: {e}")
            return desired_linear_vel, desired_angular_vel
    
    def _compute_caution_avoidance(self, linear_vel: float, angular_vel: float, 
                                  min_distance: float, obstacle_directions: List[np.ndarray]) -> Tuple[float, float]:
        """计算谨慎避障（较近距离）"""
        params = self.avoidance_params['caution']
        
        # 速度衰减因子
        distance_factor = (min_distance - self.distance_thresholds['danger']) / \
                         (self.distance_thresholds['caution'] - self.distance_thresholds['danger'])
        distance_factor = np.clip(distance_factor, 0.0, 1.0)
        
        # 线速度衰减
        speed_reduction = params['speed_reduction_factor'] * (1.0 - distance_factor)
        modified_linear_vel = linear_vel * (1.0 - speed_reduction)
        
        # 计算避障转向
        avoidance_angular = 0.0
        if obstacle_directions:
            # 计算主要障碍物方向
            avg_direction = np.mean(obstacle_directions, axis=0)
            robot_heading = np.array([1.0, 0.0])  # 假设机器人朝向X轴正方向
            
            # 计算垂直方向（右转或左转）
            cross_product = np.cross(robot_heading, avg_direction)
            turn_direction = 1.0 if cross_product > 0 else -1.0
            
            # 避障角速度
            avoidance_strength = params['avoidance_strength'] * (1.0 - distance_factor)
            avoidance_angular = turn_direction * avoidance_strength
        
        # 结合原始角速度和避障角速度
        modified_angular_vel = angular_vel * params['angular_response'] + avoidance_angular
        
        # 平滑处理
        modified_linear_vel, modified_angular_vel = self._smooth_avoidance_command(
            modified_linear_vel, modified_angular_vel, params['smoothing_factor']
        )
        
        if self.config.DEBUG["show_navigation_progress"]:
            print(f"   🟡 谨慎避障: 距离={min_distance:.2f}m, 速度={modified_linear_vel:.2f}, 转向={modified_angular_vel:.2f}")
        
        return modified_linear_vel, modified_angular_vel
    
    def _compute_danger_avoidance(self, linear_vel: float, angular_vel: float, 
                                 min_distance: float, obstacle_directions: List[np.ndarray]) -> Tuple[float, float]:
        """计算紧急避障（即将碰撞）"""
        params = self.avoidance_params['danger']
        
        # 紧急制动
        emergency_factor = min_distance / self.distance_thresholds['danger']
        emergency_factor = np.clip(emergency_factor, 0.0, 1.0)
        
        # 强制减速
        modified_linear_vel = linear_vel * emergency_factor * params['emergency_speed_factor']
        
        # 强制转向避障
        avoidance_angular = 0.0
        if obstacle_directions:
            # 计算紧急转向方向
            avg_direction = np.mean(obstacle_directions, axis=0)
            robot_heading = np.array([1.0, 0.0])
            
            # 选择转向方向（远离障碍物）
            cross_product = np.cross(robot_heading, avg_direction)
            turn_direction = 1.0 if cross_product > 0 else -1.0
            
            # 紧急转向
            avoidance_angular = turn_direction * params['emergency_turn_rate']
        
        # 紧急情况下优先避障
        modified_angular_vel = avoidance_angular + angular_vel * params['angular_override']
        
        # 限制在安全范围内
        max_linear = self.config.ROBOT_CONTROL["max_linear_velocity"] * params['max_speed_limit']
        max_angular = self.config.ROBOT_CONTROL["max_angular_velocity"] * params['max_angular_limit']
        
        modified_linear_vel = np.clip(modified_linear_vel, -max_linear, max_linear)
        modified_angular_vel = np.clip(modified_angular_vel, -max_angular, max_angular)
        
        # 轻微平滑（紧急情况下响应要快）
        modified_linear_vel, modified_angular_vel = self._smooth_avoidance_command(
            modified_linear_vel, modified_angular_vel, params['smoothing_factor']
        )
        
        if self.config.DEBUG["show_navigation_progress"]:
            print(f"   🔴 紧急避障: 距离={min_distance:.2f}m, 速度={modified_linear_vel:.2f}, 转向={modified_angular_vel:.2f}")
        
        # 更新统计
        self.detection_stats['avoidance_activations'] += 1
        
        return modified_linear_vel, modified_angular_vel
    
    def _smooth_avoidance_command(self, linear_vel: float, angular_vel: float, smoothing: float) -> Tuple[float, float]:
        """平滑避障命令"""
        # 与历史命令平滑
        smoothed_linear = smoothing * self.last_avoidance_command[0] + (1 - smoothing) * linear_vel
        smoothed_angular = smoothing * self.last_avoidance_command[1] + (1 - smoothing) * angular_vel
        
        # 更新历史
        self.last_avoidance_command = (smoothed_linear, smoothed_angular)
        self.avoidance_history.append((smoothed_linear, smoothed_angular))
        
        return smoothed_linear, smoothed_angular
    
    def _update_detection_stats(self, avoidance_level: OSGTAvoidanceLevel, distance: float):
        """更新检测统计"""
        self.detection_stats['total_detections'] += 1
        
        if avoidance_level == OSGTAvoidanceLevel.SAFE:
            self.detection_stats['safe_detections'] += 1
        elif avoidance_level == OSGTAvoidanceLevel.CAUTION:
            self.detection_stats['caution_detections'] += 1
        elif avoidance_level == OSGTAvoidanceLevel.DANGER:
            self.detection_stats['danger_detections'] += 1
        
        if distance < self.detection_stats['min_distance_recorded']:
            self.detection_stats['min_distance_recorded'] = distance
    
    def print_detection_status(self):
        """打印检测状态"""
        measurements = self.get_distance_measurements()
        
        print(f"\n📡 OSGT LightBeam检测状态 ({self.current_object_type}模式):")
        print(f"   最小距离: {measurements['min_distance']:.3f}m")
        print(f"   避障级别: {measurements['avoidance_level'].value}")
        print(f"   障碍物检测: {'是' if measurements['obstacle_detected'] else '否'}")
        print(f"   数据有效: {'是' if measurements['data_valid'] else '否'}")
        
        # 显示当前阈值
        print(f"   当前阈值: 安全>{self.distance_thresholds['safe']:.1f}m, "
              f"谨慎>{self.distance_thresholds['caution']:.1f}m, "
              f"危险>{self.distance_thresholds['danger']:.1f}m")
        
        # 显示原始距离数据
        if measurements['raw_distances']:
            print(f"   原始距离数据:")
            for data in measurements['raw_distances'][:16]:  # 只显示前10个
                status = "命中" if data['hit'] else "未命中"
                print(f"     {data['sensor']}-光束{data['beam']}: {data['distance']:.3f}m ({status})")
            if len(measurements['raw_distances']) > 16:
                print(f"     ... 还有{len(measurements['raw_distances'])-16}个数据点")
        
        if measurements['sensor_data']:
            print(f"   传感器详情:")
            for sensor_path, data in measurements['sensor_data'].items():
                print(f"     {data['name']}: {data['min_distance']:.3f}m ({data['num_hits']}个命中)")
    
    def print_detection_stats(self):
        """打印检测统计"""
        stats = self.detection_stats
        total = stats['total_detections']
        total_reads = stats['successful_reads'] + stats['data_read_failures']
        
        print(f"\n📊 OSGT LightBeam统计:")
        if total > 0:
            print(f"   总检测次数: {total}")
            print(f"   安全检测: {stats['safe_detections']} ({stats['safe_detections']/total*100:.1f}%)")
            print(f"   谨慎检测: {stats['caution_detections']} ({stats['caution_detections']/total*100:.1f}%)")
            print(f"   危险检测: {stats['danger_detections']} ({stats['danger_detections']/total*100:.1f}%)")
            print(f"   避障激活: {stats['avoidance_activations']}")
            print(f"   最近距离: {stats['min_distance_recorded']:.3f}m")
        
        if total_reads > 0:
            success_rate = stats['successful_reads'] / total_reads * 100
            print(f"   数据读取成功率: {success_rate:.1f}% ({stats['successful_reads']}/{total_reads})")
    
    def _get_robot_position(self) -> np.ndarray:
        """获取机器人位置"""
        try:
            # 简化版本，假设机器人在原点附近
            return np.array([0.0, 0.0, 0.0])
        except:
            return np.array([0.0, 0.0, 0.0])
    
    def _euler_to_quaternion(self, euler_angles: List[float]) -> np.ndarray:
        """欧拉角转四元数 [roll, pitch, yaw] -> [x, y, z, w]"""
        roll, pitch, yaw = euler_angles
        
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return np.array([qx, qy, qz, qw])
    
    def is_safe_to_move(self) -> bool:
        """检查是否安全移动"""
        return self.current_avoidance_level == OSGTAvoidanceLevel.SAFE
    
    def get_current_avoidance_level(self) -> OSGTAvoidanceLevel:
        """获取当前避障级别"""
        return self.current_avoidance_level
    
    def reset_stats(self):
        """重置统计数据"""
        self.detection_stats = {
            'total_detections': 0,
            'safe_detections': 0,
            'caution_detections': 0,
            'danger_detections': 0,
            'avoidance_activations': 0,
            'min_distance_recorded': float('inf'),
            'data_read_failures': 0,
            'successful_reads': 0
        }
        self.avoidance_history.clear()
        self.last_avoidance_command = (0.0, 0.0)


def create_osgt_lightbeam_system(config: Dict[str, Any], world, robot_prim_path: str) -> OSGTLightBeamSensorSystem:
    """创建OSGT LightBeam系统"""
    return OSGTLightBeamSensorSystem(config, world, robot_prim_path)