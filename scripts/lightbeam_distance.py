#!/usr/bin/env python3
"""
OSGT四类物体LightBeam光束传感器避障系统
8个方向的传感器配置，支持三级距离阈值检测
避障对象：O类障碍物和环境场景，不包括S/G/T三类物体
参考官方教程create_robot_obstacle_test.py的实现方式
"""

import numpy as np
import time
from typing import Dict, List, Tuple, Optional
from pxr import Gf, Sdf, UsdPhysics
import omni
import omni.timeline
import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension

class LightBeamSensorManager:
    """LightBeam传感器管理器 - 8方向避障系统（官方教程优化版）"""
    
    def __init__(self, config, robot_prim_path="/World/create3_robot"):
        self.config = config
        self.robot_prim_path = robot_prim_path
        
        # 8个传感器配置（重新设计，避免检测到机器人自身）
        self.sensors_config = {
            "sensors": [
                # 底部高度传感器 - 4个方向（增加距离，避免检测到机器人）
                {
                    "name": "front_bottom",
                    "relative_position": [1.2, 0, -0.2],       # 前移1.2m，避免检测到机器人
                    "min_range": 1.0,  # 增加最小距离
                    "max_range": 8.0,
                    "num_rays": 3,
                    "curtain_length": 0.3,
                    "forward_axis": [1, 0, 0]
                },
                {
                    "name": "back_bottom", 
                    "relative_position": [-1.2, 0, -0.2],      # 后移1.2m
                    "min_range": 1.0,
                    "max_range": 8.0,
                    "num_rays": 3,
                    "curtain_length": 0.3,
                    "forward_axis": [-1, 0, 0]
                },
                {
                    "name": "left_bottom",
                    "relative_position": [0, 1.2, -0.2],       # 左移1.2m
                    "min_range": 1.0,
                    "max_range": 8.0,
                    "num_rays": 3,
                    "curtain_length": 0.3,
                    "forward_axis": [0, 1, 0]
                },
                {
                    "name": "right_bottom",
                    "relative_position": [0, -1.2, -0.2],      # 右移1.2m
                    "min_range": 1.0,
                    "max_range": 8.0,
                    "num_rays": 3,
                    "curtain_length": 0.3,
                    "forward_axis": [0, -1, 0]
                },
                # 顶部高度传感器 - 4个方向（增加距离和高度）
                {
                    "name": "front_top",
                    "relative_position": [1.0, 0, 0.5],        # 前移1.0m，高度0.5
                    "min_range": 0.8,
                    "max_range": 8.0,
                    "num_rays": 3,
                    "curtain_length": 0.3,
                    "forward_axis": [1, 0, 0]
                },
                {
                    "name": "back_top",
                    "relative_position": [-1.0, 0, 0.5],       # 后移1.0m
                    "min_range": 0.8,
                    "max_range": 8.0,
                    "num_rays": 3,
                    "curtain_length": 0.3,
                    "forward_axis": [-1, 0, 0]
                },
                {
                    "name": "left_top",
                    "relative_position": [0, 1.0, 0.5],        # 左移1.0m
                    "min_range": 0.8,
                    "max_range": 8.0,
                    "num_rays": 3,
                    "curtain_length": 0.3,
                    "forward_axis": [0, 1, 0]
                },
                {
                    "name": "right_top",
                    "relative_position": [0, -1.0, 0.5],       # 右移1.0m
                    "min_range": 0.8,
                    "max_range": 8.0,
                    "num_rays": 3,
                    "curtain_length": 0.3,
                    "forward_axis": [0, -1, 0]
                }
            ]
        }
        
        # 三级距离阈值配置（调整为更合理的值）
        self.distance_thresholds = {
            "safe_distance": 4.0,        # 安全距离 - 绿色状态
            "warning_distance": 2.5,     # 警告距离 - 黄色状态  
            "critical_distance": 1.5     # 危险距离 - 红色状态
        }
        
        # 传感器状态
        self.sensors = {}
        self.sensor_interfaces = {}
        self.last_distances = {}
        self.obstacle_status = "safe"
        
        # 使用官方教程的接口方式
        self.lightbeam_interface = None
        self.timeline = None
        
        # 避障状态
        self.avoidance_active = False
        self.last_avoidance_time = 0
        self.avoidance_cooldown = 0.5
        
        # 初始化标志
        self.initialized = False
        self.visualization_setup = False
        
        # 统计信息
        self.stats = {
            "obstacle_detections": 0,
            "avoidance_actions": 0,
            "safe_readings": 0,
            "warning_readings": 0,
            "critical_readings": 0
        }
    
    def initialize_sensors(self, world):
        """初始化所有LightBeam传感器（官方教程优化版）"""
        try:
            print("📡 初始化8方向LightBeam传感器系统（官方教程方式）...")
            
            # 启用PhysX传感器扩展
            enable_extension("isaacsim.sensors.physx")
            world.render()
            time.sleep(0.2)
            
            # 获取stage
            stage = omni.usd.get_context().get_stage()
            
            # 确保物理场景存在
            physics_scene_path = "/World/physicsScene"
            physics_scene_prim = stage.GetPrimAtPath(physics_scene_path)
            
            if not physics_scene_prim or not physics_scene_prim.IsValid():
                print("🔧 创建物理场景...")
                physics_scene = UsdPhysics.Scene.Define(stage, Sdf.Path(physics_scene_path))
                physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
                physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
                print("✅ 物理场景已创建")
            
            # 强制更新物理场景
            world.render()
            for _ in range(15):
                world.step(render=True)
                time.sleep(0.016)
            
            # 使用官方教程的接口获取方式
            from isaacsim.sensors.physx import _range_sensor
            self.lightbeam_interface = _range_sensor.acquire_lightbeam_sensor_interface()
            self.timeline = omni.timeline.get_timeline_interface()
            
            print("✅ LightBeam接口获取成功")
            
            # 创建所有8个传感器
            success_count = 0
            for sensor_config in self.sensors_config["sensors"]:
                success = self._create_single_sensor(sensor_config, stage)
                if success:
                    success_count += 1
                else:
                    print(f"❌ 传感器 {sensor_config['name']} 创建失败")
            
            if success_count < 8:
                print(f"⚠️ 只有 {success_count}/8 个传感器创建成功")
                return False
            
            # 再次稳定物理场景
            for _ in range(25):
                world.step(render=True)
                time.sleep(0.016)
            
            self.initialized = True
            print(f"✅ LightBeam传感器系统初始化完成，共 {len(self.sensors)} 个传感器")
            print(f"🎯 三级阈值: 安全{self.distance_thresholds['safe_distance']}m, "
                  f"警告{self.distance_thresholds['warning_distance']}m, "
                  f"危险{self.distance_thresholds['critical_distance']}m")
            return True
            
        except Exception as e:
            print(f"❌ LightBeam传感器初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _create_single_sensor(self, sensor_config, stage):
        """创建单个传感器（官方教程方式）"""
        try:
            sensor_name = sensor_config["name"]
            sensor_path = f"/World/LightBeam_{sensor_name}"
            
            # 计算传感器在世界坐标中的初始位置（相对机器人中心的偏移）
            robot_pos = np.array([0.0, 0.0, 0.0])  # 机器人初始位置
            sensor_offset = np.array(sensor_config["relative_position"])
            sensor_position = robot_pos + sensor_offset
            
            # 使用官方教程的创建方式
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateLightBeamSensor",
                path=sensor_path,
                parent=None,
                min_range=sensor_config["min_range"],
                max_range=sensor_config["max_range"],
                translation=Gf.Vec3d(sensor_position[0], sensor_position[1], sensor_position[2]),
                orientation=Gf.Quatd(1, 0, 0, 0),
                forward_axis=Gf.Vec3d(*sensor_config["forward_axis"]),
                num_rays=sensor_config["num_rays"],
                curtain_length=sensor_config["curtain_length"],
            )
            
            if result:
                self.sensors[sensor_name] = {
                    "path": sensor_path,
                    "config": sensor_config,
                    "prim": stage.GetPrimAtPath(sensor_path)
                }
                self.last_distances[sensor_name] = None
                
                if self.config.DEBUG["enable_debug_output"]:
                    print(f"   ✅ 传感器 {sensor_name} 创建成功: {sensor_path}")
                return True
            else:
                print(f"   ❌ 传感器 {sensor_name} 创建失败")
                return False
                
        except Exception as e:
            print(f"创建传感器 {sensor_config['name']} 失败: {e}")
            return False
    
    def setup_visualization(self):
        """设置传感器可视化"""
        if self.visualization_setup:
            return True
            
        try:
            print("🎨 设置LightBeam传感器可视化...")
            
            # 为每个传感器创建可视化节点
            viz_count = 0
            for sensor_name, sensor_info in self.sensors.items():
                if self._create_sensor_visualization(sensor_name, sensor_info):
                    viz_count += 1
            
            self.visualization_setup = True
            print(f"✅ LightBeam传感器可视化设置完成，{viz_count}/{len(self.sensors)} 个传感器可视化")
            return True
            
        except Exception as e:
            print(f"⚠️ 传感器可视化设置失败: {e}")
            return False
    
    def _create_sensor_visualization(self, sensor_name, sensor_info):
        """为单个传感器创建可视化"""
        try:
            graph_path = f"/ActionGraph_{sensor_name}"
            sensor_path = sensor_info["path"]
            
            # 使用官方教程的可视化方式
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
            
            if self.config.DEBUG["enable_debug_output"]:
                print(f"   ✅ 传感器 {sensor_name} 可视化创建成功")
            return True
            
        except Exception as e:
            if self.config.DEBUG["enable_debug_output"]:
                print(f"传感器 {sensor_name} 可视化创建失败: {e}")
            return False
    
    def update_sensor_positions(self, robot_position, robot_yaw):
        """更新所有传感器位置和方向（官方教程优化版）"""
        if not self.initialized:
            return
        
        try:
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            
            # 计算旋转矩阵（绕Z轴旋转robot_yaw角度）
            cos_yaw = np.cos(robot_yaw)
            sin_yaw = np.sin(robot_yaw)
            rotation_matrix = np.array([
                [cos_yaw, -sin_yaw, 0],
                [sin_yaw, cos_yaw, 0],
                [0, 0, 1]
            ])
            
            for sensor_name, sensor_info in self.sensors.items():
                try:
                    sensor_config = sensor_info["config"]
                    sensor_prim = sensor_info["prim"]
                    
                    if sensor_prim and sensor_prim.IsValid():
                        # 计算传感器相对位置（考虑机器人旋转）
                        relative_pos = np.array(sensor_config["relative_position"])
                        rotated_offset = rotation_matrix @ relative_pos
                        new_position = robot_position + rotated_offset
                        
                        # 使用官方教程的位置更新方式
                        translate_attr = sensor_prim.GetAttribute("xformOp:translate")
                        if translate_attr:
                            translate_attr.Set(Gf.Vec3d(new_position[0], new_position[1], new_position[2]))
                        else:
                            # 如果没有translate属性，创建一个
                            from pxr import UsdGeom
                            xform = UsdGeom.Xform(sensor_prim)
                            translate_op = xform.AddTranslateOp()
                            translate_op.Set(Gf.Vec3d(new_position[0], new_position[1], new_position[2]))
                        
                        # 更新方向（四元数旋转）
                        try:
                            from scipy.spatial.transform import Rotation as R
                            r = R.from_euler('z', robot_yaw)
                            quat = r.as_quat()  # [x, y, z, w]
                            # USD使用 [w, x, y, z] 格式，并且需要 GfQuatf 类型
                            orientation = Gf.Quatf(float(quat[3]), float(quat[0]), float(quat[1]), float(quat[2]))
                            
                            orient_attr = sensor_prim.GetAttribute("xformOp:orient")
                            if orient_attr:
                                orient_attr.Set(orientation)
                            else:
                                from pxr import UsdGeom
                                xform = UsdGeom.Xform(sensor_prim)
                                orient_op = xform.AddOrientOp()
                                orient_op.Set(orientation)
                                
                        except ImportError:
                            # 如果没有scipy，使用简单的Z轴旋转
                            half_angle = robot_yaw / 2.0
                            quat = Gf.Quatf(float(np.cos(half_angle)), 0.0, 0.0, float(np.sin(half_angle)))
                            
                            orient_attr = sensor_prim.GetAttribute("xformOp:orient")
                            if orient_attr:
                                orient_attr.Set(quat)
                                
                except Exception as sensor_update_error:
                    if self.config.DEBUG["enable_debug_output"]:
                        print(f"更新传感器 {sensor_name} 位置失败: {sensor_update_error}")
                    continue
                
        except Exception as e:
            if self.config.DEBUG["enable_debug_output"]:
                print(f"更新传感器位置失败: {e}")
    
    def get_distance_readings(self):
        """获取所有传感器的距离读数（官方教程方式）"""
        if not self.initialized or not self.lightbeam_interface or not self.timeline:
            return {}
        
        readings = {}
        
        try:
            # 使用官方教程的timeline检查方式
            if not self.timeline.is_playing():
                return readings
        except Exception as timeline_error:
            if self.config.DEBUG["enable_debug_output"]:
                print(f"获取timeline失败: {timeline_error}")
            return readings
        
        try:
            for sensor_name, sensor_info in self.sensors.items():
                sensor_path = sensor_info["path"]
                sensor_config = sensor_info["config"]
                
                try:
                    # 使用官方教程的数据获取方式
                    linear_depth = self.lightbeam_interface.get_linear_depth_data(sensor_path)
                    beam_hit = self.lightbeam_interface.get_beam_hit_data(sensor_path)
                    
                    if linear_depth is not None and len(linear_depth) > 0 and beam_hit is not None:
                        # 转换为布尔类型
                        beam_hit_bool = beam_hit.astype(bool)
                        valid_distances = []
                        
                        # 使用官方教程的距离过滤方式
                        for i in range(len(linear_depth)):
                            if beam_hit_bool[i]:  # 如果光束命中
                                distance = linear_depth[i]
                                # 过滤有效距离范围（参考官方教程的过滤方式）
                                if sensor_config["min_range"] < distance < sensor_config["max_range"]:
                                    valid_distances.append(distance)
                        
                        if valid_distances:
                            min_distance = min(valid_distances)
                            status = self._get_distance_status(min_distance)
                            readings[sensor_name] = {
                                "distance": min_distance,
                                "status": status,
                                "all_distances": valid_distances,
                                "beam_count": len(valid_distances)
                            }
                            self.last_distances[sensor_name] = min_distance
                            
                            # 更新统计
                            if status == "safe":
                                self.stats["safe_readings"] += 1
                            elif status == "warning":
                                self.stats["warning_readings"] += 1
                            elif status == "critical":
                                self.stats["critical_readings"] += 1
                        else:
                            readings[sensor_name] = {
                                "distance": None,
                                "status": "no_detection",
                                "all_distances": [],
                                "beam_count": 0
                            }
                    else:
                        readings[sensor_name] = {
                            "distance": None,
                            "status": "no_detection", 
                            "all_distances": [],
                            "beam_count": 0
                        }
                        
                except Exception as sensor_error:
                    # 单个传感器错误不影响其他传感器
                    if self.config.DEBUG["enable_debug_output"]:
                        print(f"传感器 {sensor_name} 读取失败: {sensor_error}")
                    readings[sensor_name] = {
                        "distance": None,
                        "status": "error",
                        "all_distances": [],
                        "beam_count": 0
                    }
                
        except Exception as e:
            if self.config.DEBUG["enable_debug_output"]:
                print(f"获取传感器读数失败: {e}")
        
        return readings
    
    def _get_distance_status(self, distance):
        """根据距离返回三级状态"""
        if distance >= self.distance_thresholds["safe_distance"]:
            return "safe"
        elif distance >= self.distance_thresholds["warning_distance"]:
            return "warning"
        else:
            return "critical"
    
    def get_obstacle_analysis(self):
        """获取障碍物分析结果"""
        readings = self.get_distance_readings()
        
        analysis = {
            "overall_status": "safe",
            "min_distance": float('inf'),
            "critical_directions": [],
            "warning_directions": [],
            "safe_directions": [],
            "avoidance_recommendation": None,
            "direction_analysis": {
                "front": "safe",
                "back": "safe", 
                "left": "safe",
                "right": "safe"
            }
        }
        
        # 按方向分组分析
        direction_groups = {
            "front": ["front_bottom", "front_top"],
            "back": ["back_bottom", "back_top"],
            "left": ["left_bottom", "left_top"],
            "right": ["right_bottom", "right_top"]
        }
        
        for direction, sensor_names in direction_groups.items():
            direction_status = "safe"
            min_dir_distance = float('inf')
            
            for sensor_name in sensor_names:
                if sensor_name in readings and readings[sensor_name]["distance"] is not None:
                    distance = readings[sensor_name]["distance"]
                    status = readings[sensor_name]["status"]
                    
                    if distance < min_dir_distance:
                        min_dir_distance = distance
                    
                    if status == "critical":
                        direction_status = "critical"
                    elif status == "warning" and direction_status == "safe":
                        direction_status = "warning"
            
            analysis["direction_analysis"][direction] = direction_status
            
            if direction_status == "critical":
                analysis["critical_directions"].append(direction)
                analysis["overall_status"] = "critical"
            elif direction_status == "warning":
                analysis["warning_directions"].append(direction)
                if analysis["overall_status"] == "safe":
                    analysis["overall_status"] = "warning"
            else:
                analysis["safe_directions"].append(direction)
            
            if min_dir_distance < analysis["min_distance"]:
                analysis["min_distance"] = min_dir_distance
        
        # 生成避障建议
        analysis["avoidance_recommendation"] = self._generate_avoidance_recommendation(analysis)
        
        # 更新统计
        if analysis["overall_status"] != "safe":
            self.stats["obstacle_detections"] += 1
        
        return analysis
    
    def _generate_avoidance_recommendation(self, analysis):
        """生成避障建议"""
        if analysis["overall_status"] == "safe":
            return {
                "action": "continue", 
                "linear_scale": 1.0, 
                "angular_scale": 1.0,
                "description": "路径畅通，继续前进"
            }
        
        elif analysis["overall_status"] == "warning":
            return {
                "action": "slow_down", 
                "linear_scale": 0.6, 
                "angular_scale": 0.8,
                "description": "检测到障碍物，预防性减速"
            }
        
        elif analysis["overall_status"] == "critical":
            # 危险状态：根据方向选择避障策略
            direction_analysis = analysis["direction_analysis"]
            
            front_blocked = direction_analysis["front"] == "critical"
            back_blocked = direction_analysis["back"] == "critical"
            left_blocked = direction_analysis["left"] == "critical"
            right_blocked = direction_analysis["right"] == "critical"
            
            # 智能避障策略
            if front_blocked and not back_blocked:
                if not left_blocked and not right_blocked:
                    return {
                        "action": "turn_right",
                        "linear_scale": 0.2,
                        "angular_scale": -0.8,
                        "description": "前方受阻，右转避障"
                    }
                elif not left_blocked:
                    return {
                        "action": "turn_left",
                        "linear_scale": 0.15,
                        "angular_scale": 0.8,
                        "description": "前方右侧受阻，左转避障"
                    }
                elif not right_blocked:
                    return {
                        "action": "turn_right",
                        "linear_scale": 0.15,
                        "angular_scale": -0.8,
                        "description": "前方左侧受阻，右转避障"
                    }
                else:
                    return {
                        "action": "reverse",
                        "linear_scale": -0.3,
                        "angular_scale": 0.0,
                        "description": "前方完全受阻，后退重新规划"
                    }
            
            elif left_blocked and not right_blocked:
                return {
                    "action": "turn_right",
                    "linear_scale": 0.1,
                    "angular_scale": -0.7,
                    "description": "左侧受阻，右转避障"
                }
            
            elif right_blocked and not left_blocked:
                return {
                    "action": "turn_left",
                    "linear_scale": 0.1,
                    "angular_scale": 0.7,
                    "description": "右侧受阻，左转避障"
                }
            
            else:
                # 四面受阻，紧急停止
                return {
                    "action": "emergency_stop",
                    "linear_scale": 0.0,
                    "angular_scale": 0.0,
                    "description": "四面受阻，紧急停止"
                }
        
        return {
            "action": "stop", 
            "linear_scale": 0.0, 
            "angular_scale": 0.0,
            "description": "未知状态，停止运动"
        }
    
    def apply_avoidance_control(self, base_linear_vel, base_angular_vel):
        """应用避障控制，返回修正后的速度"""
        if not self.initialized:
            return base_linear_vel, base_angular_vel
        
        analysis = self.get_obstacle_analysis()
        recommendation = analysis["avoidance_recommendation"]
        
        if recommendation is None:
            return base_linear_vel, base_angular_vel
        
        current_time = time.time()
        
        # 检查避障冷却时间
        if (self.avoidance_active and 
            current_time - self.last_avoidance_time < self.avoidance_cooldown):
            return base_linear_vel * 0.5, base_angular_vel * 0.5
        
        action = recommendation["action"]
        linear_scale = recommendation["linear_scale"]
        angular_scale = recommendation["angular_scale"]
        
        if action == "continue":
            return base_linear_vel, base_angular_vel
        
        elif action == "slow_down":
            return base_linear_vel * linear_scale, base_angular_vel * angular_scale
        
        elif action in ["turn_left", "turn_right"]:
            self.avoidance_active = True
            self.last_avoidance_time = current_time
            self.stats["avoidance_actions"] += 1
            
            if self.config.DEBUG["show_navigation_progress"]:
                print(f"🔄 执行避障动作: {recommendation['description']}")
            
            return base_linear_vel * linear_scale, base_angular_vel + (angular_scale * 1.5)
        
        elif action == "reverse":
            self.avoidance_active = True
            self.last_avoidance_time = current_time
            self.stats["avoidance_actions"] += 1
            
            if self.config.DEBUG["show_navigation_progress"]:
                print(f"⬅️ 执行避障动作: {recommendation['description']}")
            
            return linear_scale, angular_scale
        
        elif action in ["emergency_stop", "stop"]:
            self.avoidance_active = True
            self.last_avoidance_time = current_time
            
            if self.config.DEBUG["show_navigation_progress"]:
                print(f"🛑 执行避障动作: {recommendation['description']}")
            
            return 0.0, 0.0
        
        return base_linear_vel, base_angular_vel
    
    def is_path_clear(self, direction="front", threshold="warning"):
        """检查指定方向是否畅通"""
        if not self.initialized:
            return True
        
        try:
            readings = self.get_distance_readings()
            
            direction_sensors = {
                "front": ["front_bottom", "front_top"],
                "back": ["back_bottom", "back_top"],
                "left": ["left_bottom", "left_top"],
                "right": ["right_bottom", "right_top"]
            }
            
            relevant_sensors = direction_sensors.get(direction, [])
            
            for sensor_name in relevant_sensors:
                if sensor_name in readings:
                    reading = readings[sensor_name]
                    if reading["distance"] is not None:
                        status = reading["status"]
                        if threshold == "warning" and status in ["warning", "critical"]:
                            return False
                        elif threshold == "critical" and status == "critical":
                            return False
            
            return True
            
        except Exception as e:
            if self.config.DEBUG["enable_debug_output"]:
                print(f"路径检查失败: {e}")
            return True
    
    def print_sensor_status(self, detailed=False):
        """打印传感器状态"""
        if not self.initialized:
            print("⚠️ LightBeam传感器系统未初始化")
            return
        
        readings = self.get_distance_readings()
        analysis = self.get_obstacle_analysis()
        
        print(f"\n📡 LightBeam传感器状态 (8方向):")
        print(f"   整体状态: {self._get_status_emoji(analysis['overall_status'])} {analysis['overall_status'].upper()}")
        
        if analysis['min_distance'] < float('inf'):
            print(f"   最小距离: {analysis['min_distance']:.2f}m")
        else:
            print(f"   最小距离: 无检测")
        
        if detailed:
            # 详细模式：显示所有传感器
            for sensor_name, reading in readings.items():
                if reading["distance"] is not None:
                    status_emoji = self._get_status_emoji(reading["status"])
                    print(f"   {sensor_name:15}: {status_emoji} {reading['distance']:.2f}m ({reading['beam_count']}束)")
                else:
                    print(f"   {sensor_name:15}: ⚫ 无检测")
        else:
            # 简化模式：按方向显示
            for direction, status in analysis["direction_analysis"].items():
                status_emoji = self._get_status_emoji(status)
                dir_sensors = [name for name in readings.keys() if direction in name]
                min_dist = min([readings[name]["distance"] for name in dir_sensors 
                              if name in readings and readings[name]["distance"] is not None], 
                              default=None)
                if min_dist is not None:
                    print(f"   {direction:6}: {status_emoji} {min_dist:.2f}m")
                else:
                    print(f"   {direction:6}: ⚫ 无检测")
        
        if analysis["avoidance_recommendation"]:
            rec = analysis["avoidance_recommendation"]
            print(f"   🤖 建议: {rec['description']}")
            print(f"   ⚙️ 调整: 线性{rec['linear_scale']:.1f} 角度{rec['angular_scale']:.1f}")
    
    def _get_status_emoji(self, status):
        """获取状态表情符号"""
        status_emojis = {
            "safe": "🟢",
            "warning": "🟡", 
            "critical": "🔴",
            "no_detection": "⚫",
            "error": "❓"
        }
        return status_emojis.get(status, "❓")
    
    def print_statistics(self):
        """打印统计信息"""
        total_readings = sum([self.stats["safe_readings"], 
                             self.stats["warning_readings"], 
                             self.stats["critical_readings"]])
        
        print(f"\n📊 LightBeam传感器统计:")
        print(f"   总检测次数: {total_readings}")
        print(f"   🟢 安全读数: {self.stats['safe_readings']}")
        print(f"   🟡 警告读数: {self.stats['warning_readings']}")
        print(f"   🔴 危险读数: {self.stats['critical_readings']}")
        print(f"   🚨 障碍检测: {self.stats['obstacle_detections']}")
        print(f"   🔄 避障动作: {self.stats['avoidance_actions']}")
        
        if total_readings > 0:
            safe_rate = (self.stats["safe_readings"] / total_readings) * 100
            print(f"   📈 安全率: {safe_rate:.1f}%")
    
    def cleanup(self):
        """清理传感器资源"""
        try:
            if self.initialized:
                print("🧹 清理LightBeam传感器资源...")
                # 这里可以添加清理代码，如果需要的话
                self.initialized = False
                print("✅ LightBeam传感器清理完成")
        except Exception as e:
            print(f"清理LightBeam传感器失败: {e}")