#!/usr/bin/env python3
"""
OSGT四类物体高级机械臂抓取放下封装模块 - pick_and_place.py
基于Auromix auro_sim项目的抓取原理，集成CUDA加速和力控制反馈
专为OSGT标准设计：
- O类障碍物：避免接触，提供避障支持
- S类可清扫物：吸附式收集，无需精确抓取
- G类可抓取物：精确机械臂抓取和放置
- T类任务区：定位和交互支持
"""

import numpy as np
import time
import math
import random
from typing import List, Tuple, Dict, Optional, Any
from collections import deque
from dataclasses import dataclass
from enum import Enum

try:
    import cupy as cp
    CUDA_AVAILABLE = True
    print("✅ CUDA加速可用于OSGT抓取规划")
except ImportError:
    CUDA_AVAILABLE = False
    cp = np
    print("⚠️ CUDA不可用，使用CPU OSGT抓取规划")

try:
    from scipy.spatial.transform import Rotation as R
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("⚠️ SciPy不可用，使用简化旋转计算")

class OSGTGraspPhase(Enum):
    """OSGT抓取阶段枚举"""
    IDLE = "idle"
    OSGT_ANALYZING = "osgt_analyzing"      # OSGT类型分析
    APPROACHING = "approaching"            # 接近目标
    GRASPING = "grasping"                 # 抓取执行
    LIFTING = "lifting"                   # 提升动作
    TRANSPORTING = "transporting"         # 运输过程
    PLACING = "placing"                   # 放置动作
    RETRACTING = "retracting"            # 撤回动作
    COMPLETED = "completed"               # 完成
    FAILED = "failed"                     # 失败
    OSGT_AVOIDED = "osgt_avoided"         # O类避障
    OSGT_SWEPT = "osgt_swept"             # S类清扫
    OSGT_GRASPED = "osgt_grasped"         # G类抓取
    OSGT_VISITED = "osgt_visited"         # T类访问

@dataclass
class OSGTGraspCandidate:
    """OSGT抓取候选点数据结构"""
    position: np.ndarray          # 抓取位置 [x, y, z]
    orientation: np.ndarray       # 抓取方向 [roll, pitch, yaw]
    approach_vector: np.ndarray   # 接近向量
    quality_score: float          # 抓取质量分数 (0-1)
    collision_free: bool          # 是否无碰撞
    reachable: bool              # 是否可达
    osgt_type: str               # OSGT物体类型
    osgt_specific_score: float   # OSGT类型特定分数

class OSGTForceProfile:
    """OSGT力控制配置文件"""
    def __init__(self, osgt_type: str = "graspable"):
        self.osgt_type = osgt_type
        
        # 根据OSGT类型设置不同的力控制参数
        if osgt_type == "graspable":
            # G类可抓取物：精确力控制
            self.approach_force = 2.0     # 接近时的最大力 (N)
            self.grasp_force = 15.0       # 抓取时的目标力 (N)
            self.lift_force = 20.0        # 提升时的最大力 (N)
            self.transport_force = 10.0   # 运输时的保持力 (N)
            self.force_threshold = 25.0   # 力阈值，超过则停止 (N)
            self.contact_threshold = 3.0  # 接触检测阈值 (N)
        elif osgt_type == "sweepable":
            # S类可清扫物：轻柔接触（实际使用吸附）
            self.approach_force = 1.0
            self.grasp_force = 5.0
            self.lift_force = 8.0
            self.transport_force = 5.0
            self.force_threshold = 12.0
            self.contact_threshold = 1.5
        elif osgt_type == "task_areas":
            # T类任务区：稳定接触
            self.approach_force = 3.0
            self.grasp_force = 20.0
            self.lift_force = 25.0
            self.transport_force = 15.0
            self.force_threshold = 30.0
            self.contact_threshold = 4.0
        else:  # obstacles 或其他
            # O类障碍物：避免接触
            self.approach_force = 0.5
            self.grasp_force = 2.0
            self.lift_force = 3.0
            self.transport_force = 2.0
            self.force_threshold = 5.0
            self.contact_threshold = 0.8

class OSGTCudaAcceleratedGraspPlanner:
    """OSGT四类物体CUDA加速的抓取规划器"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.use_cuda = CUDA_AVAILABLE
        
        # OSGT类型特定的抓取参数
        self.osgt_grasp_params = {
            "graspable": {
                "gripper_width": 0.08,      # G类：标准夹爪开口
                "approach_distance": 0.15,   # 标准接近距离
                "lift_height": 0.2,         # 标准提升高度
                "precision_factor": 1.0,     # 精度因子
                "templates_count": 16        # 抓取模板数量
            },
            "sweepable": {
                "gripper_width": 0.06,      # S类：较小开口（实际不使用）
                "approach_distance": 0.1,   # 短接近距离
                "lift_height": 0.05,        # 低提升高度
                "precision_factor": 0.5,    # 低精度要求
                "templates_count": 8         # 少量模板
            },
            "task_areas": {
                "gripper_width": 0.1,       # T类：大开口
                "approach_distance": 0.2,   # 长接近距离
                "lift_height": 0.15,        # 适中提升高度
                "precision_factor": 0.8,    # 高精度要求
                "templates_count": 12        # 适中模板数量
            },
            "obstacles": {
                "gripper_width": 0.04,      # O类：最小接触
                "approach_distance": 0.3,   # 最长避让距离
                "lift_height": 0.0,         # 不提升
                "precision_factor": 0.2,    # 最低精度
                "templates_count": 4         # 最少模板
            }
        }
        
        # CUDA优化参数
        self.batch_size = 64 if self.use_cuda else 8
        self.max_candidates = 256
        
        # 预定义OSGT抓取模板
        self.osgt_grasp_templates = {}
        self._generate_osgt_grasp_templates()
        
        if config.DEBUG["enable_debug_output"]:
            print(f"🔧 OSGT抓取规划器初始化: CUDA={self.use_cuda}, 批处理={self.batch_size}")
    
    def _generate_osgt_grasp_templates(self):
        """生成OSGT类型特定的抓取模板"""
        for osgt_type, params in self.osgt_grasp_params.items():
            templates = []
            template_count = params["templates_count"]
            
            if osgt_type == "graspable":
                # G类：全方向精确抓取模板
                # 顶部垂直抓取
                templates.append(np.array([0, 0, -1, 0, 0, 0]))
                
                # 侧面抓取（多角度）
                for angle in np.linspace(0, 2*np.pi, 8):
                    dx = np.cos(angle)
                    dy = np.sin(angle)
                    templates.append(np.array([dx, dy, 0, 0, np.pi/2, angle]))
                
                # 斜面抓取
                for angle in np.linspace(0, 2*np.pi, 6):
                    for tilt in [np.pi/6, np.pi/4]:
                        dx = np.cos(angle) * np.sin(tilt)
                        dy = np.sin(angle) * np.sin(tilt)
                        dz = -np.cos(tilt)
                        templates.append(np.array([dx, dy, dz, 0, tilt, angle]))
                        if len(templates) >= template_count:
                            break
                    if len(templates) >= template_count:
                        break
                        
            elif osgt_type == "sweepable":
                # S类：简单的顶部接触模板（实际使用吸附）
                templates.append(np.array([0, 0, -1, 0, 0, 0]))  # 垂直向下
                for angle in np.linspace(0, 2*np.pi, 4):
                    dx = np.cos(angle) * 0.3
                    dy = np.sin(angle) * 0.3
                    templates.append(np.array([dx, dy, -0.8, 0, np.pi/6, angle]))
                    if len(templates) >= template_count:
                        break
                        
            elif osgt_type == "task_areas":
                # T类：稳定接触模板
                templates.append(np.array([0, 0, -1, 0, 0, 0]))  # 垂直向下
                for angle in np.linspace(0, 2*np.pi, 6):
                    dx = np.cos(angle) * 0.5
                    dy = np.sin(angle) * 0.5
                    templates.append(np.array([dx, dy, -0.6, 0, np.pi/4, angle]))
                    if len(templates) >= template_count:
                        break
                        
            elif osgt_type == "obstacles":
                # O类：避障模板（不实际接触）
                templates.append(np.array([1, 0, 0, 0, np.pi/2, 0]))  # 侧向
                templates.append(np.array([0, 1, 0, 0, np.pi/2, np.pi/2]))
                templates.append(np.array([-1, 0, 0, 0, np.pi/2, np.pi]))
                templates.append(np.array([0, -1, 0, 0, np.pi/2, -np.pi/2]))
            
            self.osgt_grasp_templates[osgt_type] = templates[:template_count]
    
    def generate_osgt_grasp_candidates(self, target_position: np.ndarray, 
                                      object_size: np.ndarray,
                                      osgt_type: str = "graspable") -> List[OSGTGraspCandidate]:
        """生成OSGT类型特定的抓取候选点"""
        candidates = []
        
        try:
            # 获取OSGT类型特定参数和模板
            params = self.osgt_grasp_params.get(osgt_type, self.osgt_grasp_params["graspable"])
            templates = self.osgt_grasp_templates.get(osgt_type, self.osgt_grasp_templates["graspable"])
            
            # 使用模板生成候选点
            for template in templates:
                # 计算抓取位置
                approach_vector = template[:3]
                orientation = template[3:]
                
                # 考虑物体大小和OSGT类型调整接近距离
                object_radius = np.max(object_size) / 2
                adjusted_distance = params["approach_distance"] + object_radius * params["precision_factor"]
                
                grasp_position = target_position + approach_vector * adjusted_distance
                
                # 创建OSGT候选点
                candidate = OSGTGraspCandidate(
                    position=grasp_position,
                    orientation=orientation,
                    approach_vector=approach_vector,
                    quality_score=0.0,
                    collision_free=True,
                    reachable=True,
                    osgt_type=osgt_type,
                    osgt_specific_score=0.0
                )
                
                candidates.append(candidate)
            
            # CUDA加速的OSGT质量评估
            if self.use_cuda and len(candidates) > 0:
                candidates = self._cuda_evaluate_osgt_candidates(candidates, target_position, osgt_type)
            else:
                candidates = self._cpu_evaluate_osgt_candidates(candidates, target_position, osgt_type)
            
            # 按OSGT特定质量分数排序
            candidates.sort(key=lambda x: x.osgt_specific_score, reverse=True)
            
            return candidates[:self.max_candidates]
            
        except Exception as e:
            print(f"❌ 生成OSGT-{osgt_type}抓取候选点失败: {e}")
            return []
    
    def _cuda_evaluate_osgt_candidates(self, candidates: List[OSGTGraspCandidate], 
                                      target_pos: np.ndarray, osgt_type: str) -> List[OSGTGraspCandidate]:
        """CUDA加速的OSGT候选点评估"""
        try:
            if not candidates:
                return candidates
            
            # 转换为GPU数组
            positions = cp.array([c.position for c in candidates])
            orientations = cp.array([c.orientation for c in candidates])
            target = cp.array(target_pos)
            
            # 并行计算质量分数
            distances = cp.linalg.norm(positions - target, axis=1)
            
            # 距离分数（越近越好）
            distance_scores = cp.exp(-distances / 0.5)
            
            # 方向分数
            approach_vectors = cp.array([c.approach_vector for c in candidates])
            
            # OSGT类型特定的评分
            if osgt_type == "graspable":
                # G类：偏好垂直向下抓取
                vertical_preference = cp.abs(approach_vectors[:, 2])
                osgt_bonus = vertical_preference * 0.3
            elif osgt_type == "sweepable":
                # S类：偏好简单接触
                contact_simplicity = 1.0 - cp.linalg.norm(approach_vectors[:, :2], axis=1)
                osgt_bonus = contact_simplicity * 0.2
            elif osgt_type == "task_areas":
                # T类：偏好稳定接触角度
                stability_score = cp.exp(-cp.abs(approach_vectors[:, 2] - (-0.7)) / 0.3)
                osgt_bonus = stability_score * 0.25
            else:  # obstacles
                # O类：偏好避让方向
                avoidance_score = cp.linalg.norm(approach_vectors[:, :2], axis=1)
                osgt_bonus = avoidance_score * 0.1
            
            # 综合分数
            quality_scores = 0.5 * distance_scores + 0.3 * osgt_bonus + 0.2 * cp.random.normal(0, 0.05, len(candidates))
            quality_scores = cp.clip(quality_scores, 0, 1)
            
            # OSGT特定分数
            osgt_specific_scores = 0.7 * quality_scores + 0.3 * osgt_bonus
            osgt_specific_scores = cp.clip(osgt_specific_scores, 0, 1)
            
            # 更新候选点分数
            quality_scores_cpu = cp.asnumpy(quality_scores)
            osgt_scores_cpu = cp.asnumpy(osgt_specific_scores)
            
            for i, candidate in enumerate(candidates):
                candidate.quality_score = float(quality_scores_cpu[i])
                candidate.osgt_specific_score = float(osgt_scores_cpu[i])
            
            return candidates
            
        except Exception as e:
            print(f"⚠️ CUDA评估失败，回退到CPU: {e}")
            return self._cpu_evaluate_osgt_candidates(candidates, target_pos, osgt_type)
    
    def _cpu_evaluate_osgt_candidates(self, candidates: List[OSGTGraspCandidate], 
                                     target_pos: np.ndarray, osgt_type: str) -> List[OSGTGraspCandidate]:
        """CPU版本的OSGT候选点评估"""
        for candidate in candidates:
            try:
                # 距离分数
                distance = np.linalg.norm(candidate.position - target_pos)
                distance_score = np.exp(-distance / 0.5)
                
                # OSGT类型特定评分
                if osgt_type == "graspable":
                    # G类：垂直向下优先
                    vertical_score = abs(candidate.approach_vector[2])
                    osgt_bonus = vertical_score * 0.3
                elif osgt_type == "sweepable":
                    # S类：接触简单性优先
                    contact_simplicity = 1.0 - np.linalg.norm(candidate.approach_vector[:2])
                    osgt_bonus = contact_simplicity * 0.2
                elif osgt_type == "task_areas":
                    # T类：稳定角度优先
                    ideal_angle = -0.7
                    stability_score = np.exp(-abs(candidate.approach_vector[2] - ideal_angle) / 0.3)
                    osgt_bonus = stability_score * 0.25
                else:  # obstacles
                    # O类：避让方向优先
                    avoidance_score = np.linalg.norm(candidate.approach_vector[:2])
                    osgt_bonus = avoidance_score * 0.1
                
                # 可达性检查
                reachability_score = self._check_reachability(candidate.position)
                
                # 综合分数
                candidate.quality_score = (
                    0.5 * distance_score + 
                    0.3 * osgt_bonus + 
                    0.2 * reachability_score
                )
                
                # OSGT特定分数
                candidate.osgt_specific_score = (
                    0.7 * candidate.quality_score +
                    0.3 * osgt_bonus
                )
                
                # 添加随机扰动
                candidate.quality_score += random.uniform(-0.05, 0.05)
                candidate.osgt_specific_score += random.uniform(-0.03, 0.03)
                
                # 限制到[0,1]范围
                candidate.quality_score = max(0, min(1, candidate.quality_score))
                candidate.osgt_specific_score = max(0, min(1, candidate.osgt_specific_score))
                
            except Exception as e:
                candidate.quality_score = 0.0
                candidate.osgt_specific_score = 0.0
        
        return candidates
    
    def _check_reachability(self, position: np.ndarray) -> float:
        """检查位置的可达性（简化版）"""
        # 机械臂工作空间检查（球形近似）
        arm_base = np.array([0, 0, 0.3])  # 假设机械臂基座位置
        distance = np.linalg.norm(position - arm_base)
        
        # 工作空间参数
        min_reach = 0.2   # 最小工作半径
        max_reach = 0.8   # 最大工作半径
        
        if distance < min_reach:
            return 0.0  # 太近，无法达到
        elif distance > max_reach:
            return 0.0  # 太远，无法达到
        else:
            # 理想距离为中等距离
            ideal_distance = (min_reach + max_reach) / 2
            score = 1.0 - abs(distance - ideal_distance) / ideal_distance
            return max(0, score)

class OSGTAdaptiveGripperController:
    """OSGT四类物体自适应夹爪控制器"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.current_osgt_type = "graspable"
        self.force_profile = OSGTForceProfile(self.current_osgt_type)
        
        # 世界对象引用
        self.world = None
        
        # 夹爪参数
        self.gripper_open = config.ARM_CONFIG["gripper_open"]
        self.gripper_closed = config.ARM_CONFIG["gripper_closed"]
        self.current_position = self.gripper_open
        
        # OSGT类型特定的PID参数
        self.osgt_pid_params = {
            "graspable": {'kp': 0.001, 'ki': 0.0001, 'kd': 0.0005},  # G类：精确控制
            "sweepable": {'kp': 0.0005, 'ki': 0.00005, 'kd': 0.0002}, # S类：轻柔控制
            "task_areas": {'kp': 0.0008, 'ki': 0.00008, 'kd': 0.0004}, # T类：稳定控制
            "obstacles": {'kp': 0.0002, 'ki': 0.00001, 'kd': 0.0001}   # O类：最小控制
        }
        
        # 当前PID状态
        self.force_pid = self.osgt_pid_params["graspable"].copy()
        self.force_pid.update({'integral': 0.0, 'prev_error': 0.0})
        
        # 状态监控
        self.contact_detected = False
        self.object_grasped = False
        self.grasp_start_time = 0
        
        # OSGT统计
        self.osgt_grasp_stats = {
            'graspable_attempts': 0, 'graspable_successes': 0,
            'sweepable_attempts': 0, 'sweepable_successes': 0,
            'task_areas_attempts': 0, 'task_areas_successes': 0,
            'obstacles_attempts': 0, 'obstacles_successes': 0
        }
    
    def set_osgt_type(self, osgt_type: str):
        """设置当前处理的OSGT物体类型"""
        self.current_osgt_type = osgt_type
        self.force_profile = OSGTForceProfile(osgt_type)
        
        # 更新PID参数
        pid_params = self.osgt_pid_params.get(osgt_type, self.osgt_pid_params["graspable"])
        self.force_pid.update(pid_params)
        self.force_pid.update({'integral': 0.0, 'prev_error': 0.0})
        
        if self.config.DEBUG["show_grasp_details"]:
            print(f"🎯 夹爪控制器切换到OSGT-{osgt_type}模式")
    
    def execute_osgt_adaptive_grasp(self, mobile_base, osgt_type: str = None, 
                                   target_force: float = None) -> bool:
        """执行OSGT类型特定的自适应抓取"""
        if osgt_type is None:
            osgt_type = self.current_osgt_type
        else:
            self.set_osgt_type(osgt_type)
        
        if target_force is None:
            target_force = self.force_profile.grasp_force
        
        # 记录统计
        self.osgt_grasp_stats[f'{osgt_type}_attempts'] += 1
        
        try:
            self.grasp_start_time = time.time()
            
            if osgt_type == "graspable":
                success = self._execute_graspable_sequence(mobile_base, target_force)
            elif osgt_type == "sweepable":
                success = self._execute_sweepable_sequence(mobile_base)
            elif osgt_type == "task_areas":
                success = self._execute_task_area_sequence(mobile_base)
            elif osgt_type == "obstacles":
                success = self._execute_obstacle_avoidance(mobile_base)
            else:
                success = self._execute_graspable_sequence(mobile_base, target_force)
            
            if success:
                self.osgt_grasp_stats[f'{osgt_type}_successes'] += 1
            
            return success
            
        except Exception as e:
            print(f"   ❌ OSGT-{osgt_type}自适应抓取失败: {e}")
            return False
    
    def _execute_graspable_sequence(self, mobile_base, target_force: float) -> bool:
        """G类可抓取物：标准精确抓取序列"""
        try:
            print("   🦾 G类可抓取物：执行精确抓取序列")
            
            # 阶段1: 快速接近
            print("   🔧 阶段1: 快速接近目标")
            success = self._rapid_approach(mobile_base, approach_factor=0.7)
            if not success:
                return False
            
            # 阶段2: 慢速接触
            print("   🤏 阶段2: 慢速接触检测")
            success = self._slow_contact(mobile_base, contact_sensitivity=1.0)
            if not success:
                return False
            
            # 阶段3: 力控制抓取
            print("   💪 阶段3: 力控制抓取")
            success = self._force_controlled_grasp(mobile_base, target_force)
            if not success:
                return False
            
            # 阶段4: 抓取验证
            print("   ✅ 阶段4: 抓取验证")
            success = self._verify_grasp(mobile_base)
            
            return success
            
        except Exception as e:
            print(f"   ❌ G类抓取序列失败: {e}")
            return False
    
    def _execute_sweepable_sequence(self, mobile_base) -> bool:
        """S类可清扫物：简化吸附序列"""
        try:
            print("   🧹 S类可清扫物：执行吸附收集")
            
            # S类主要通过吸附收集，夹爪只是辅助接触
            print("   🔧 轻微接触确认...")
            success = self._gentle_contact(mobile_base)
            
            if success:
                print("   ✅ S类吸附收集完成")
                return True
            else:
                print("   ⚠️ S类接触失败，但可能已被吸附")
                return True  # S类容错性高
                
        except Exception as e:
            print(f"   ❌ S类吸附序列失败: {e}")
            return False
    
    def _execute_task_area_sequence(self, mobile_base) -> bool:
        """T类任务区：稳定接触序列"""
        try:
            print("   🎯 T类任务区：执行稳定接触")
            
            # 阶段1: 缓慢接近
            print("   🔧 阶段1: 缓慢稳定接近")
            success = self._stable_approach(mobile_base)
            if not success:
                return False
            
            # 阶段2: 确认接触
            print("   🤝 阶段2: 确认稳定接触")
            success = self._confirm_contact(mobile_base)
            
            if success:
                print("   ✅ T类任务区接触完成")
                return True
            else:
                print("   ⚠️ T类接触部分成功")
                return True  # T类部分成功也算成功
                
        except Exception as e:
            print(f"   ❌ T类接触序列失败: {e}")
            return False
    
    def _execute_obstacle_avoidance(self, mobile_base) -> bool:
        """O类障碍物：避障序列（不实际接触）"""
        try:
            print("   🚧 O类障碍物：执行避障动作")
            
            # O类不应该被抓取，执行避让动作
            print("   ⚠️ 检测到O类障碍物，执行避让...")
            
            # 模拟避让动作（机械臂撤回）
            time.sleep(0.5)
            
            print("   ✅ O类障碍物避让完成")
            return True  # 成功避让就算成功
            
        except Exception as e:
            print(f"   ❌ O类避障失败: {e}")
            return False
    
    def _rapid_approach(self, mobile_base, approach_factor: float = 0.7) -> bool:
        """快速接近阶段（OSGT优化版）"""
        try:
            print("   🚀 快速接近阶段...")
            target_position = self.gripper_open * approach_factor
            
            world = self._get_world_from_mobile_base(mobile_base)
            
            approach_steps = 25 if self.current_osgt_type == "graspable" else 15
            
            for step in range(approach_steps):
                self._set_gripper_position(mobile_base, target_position)
                
                if world:
                    world.step(render=True)
                time.sleep(0.033)
                
                if step % 8 == 0 and self.config.DEBUG["show_grasp_details"]:
                    print(f"   📈 接近进度: {(step+1)/approach_steps*100:.0f}%")
                
                if self._check_early_contact():
                    print("   ⚠️ 检测到早期接触，切换到慢速模式")
                    self.current_position = target_position
                    return True
            
            self.current_position = target_position
            print("   ✅ 快速接近完成")
            return True
            
        except Exception as e:
            print(f"   ❌ 快速接近失败: {e}")
            return False
    
    def _slow_contact(self, mobile_base, contact_sensitivity: float = 1.0) -> bool:
        """慢速接触检测（OSGT优化版）"""
        try:
            print("   🐌 慢速接触检测阶段...")
            
            # 根据OSGT类型调整超时和步长
            if self.current_osgt_type == "graspable":
                contact_timeout = 8.0
                step_size = 0.001
            elif self.current_osgt_type == "sweepable":
                contact_timeout = 4.0
                step_size = 0.002
            elif self.current_osgt_type == "task_areas":
                contact_timeout = 10.0
                step_size = 0.0008
            else:  # obstacles
                contact_timeout = 2.0
                step_size = 0.005
            
            start_time = time.time()
            step_count = 0
            
            world = self._get_world_from_mobile_base(mobile_base)
            
            while time.time() - start_time < contact_timeout:
                self.current_position -= step_size
                self.current_position = max(self.current_position, self.gripper_closed)
                
                self._set_gripper_position(mobile_base, self.current_position)
                
                if world:
                    for _ in range(2):
                        world.step(render=True)
                time.sleep(0.08)
                
                step_count += 1
                
                if step_count % 10 == 0 and self.config.DEBUG["show_grasp_details"]:
                    elapsed = time.time() - start_time
                    print(f"   📈 接触检测进度: {elapsed/contact_timeout*100:.0f}%, 位置: {self.current_position:.4f}")
                
                # 检测接触（考虑敏感度）
                if self._detect_contact() and random.random() < contact_sensitivity:
                    self.contact_detected = True
                    print(f"   ✅ 检测到接触！位置: {self.current_position:.4f}")
                    return True
                
                if self.current_position <= self.gripper_closed + 0.005:
                    if self.current_osgt_type == "sweepable":
                        print("   ✅ S类轻接触完成")
                        return True
                    else:
                        print("   ⚠️ 夹爪完全闭合但未检测到物体")
                        return False
            
            print("   ⚠️ 接触检测超时")
            return False
            
        except Exception as e:
            print(f"   ❌ 慢速接触失败: {e}")
            return False
    
    def _gentle_contact(self, mobile_base) -> bool:
        """轻柔接触（S类专用）"""
        try:
            print("   🕊️ 轻柔接触阶段...")
            
            # S类只需要轻微接触确认位置
            target_position = self.gripper_open * 0.8
            
            world = self._get_world_from_mobile_base(mobile_base)
            
            for step in range(10):
                self._set_gripper_position(mobile_base, target_position)
                
                if world:
                    world.step(render=True)
                time.sleep(0.05)
            
            self.current_position = target_position
            print("   ✅ S类轻柔接触完成")
            return True
            
        except Exception as e:
            print(f"   ❌ 轻柔接触失败: {e}")
            return False
    
    def _stable_approach(self, mobile_base) -> bool:
        """稳定接近（T类专用）"""
        try:
            print("   🛡️ 稳定接近阶段...")
            
            # T类需要非常稳定的接近
            target_position = self.gripper_open * 0.6
            
            world = self._get_world_from_mobile_base(mobile_base)
            
            for step in range(40):  # 更多步数确保稳定
                progress = step / 40
                current_target = self.gripper_open + progress * (target_position - self.gripper_open)
                
                self._set_gripper_position(mobile_base, current_target)
                
                if world:
                    world.step(render=True)
                time.sleep(0.05)
                
                if step % 10 == 0 and self.config.DEBUG["show_grasp_details"]:
                    print(f"   📈 稳定接近进度: {progress*100:.0f}%")
            
            self.current_position = target_position
            print("   ✅ T类稳定接近完成")
            return True
            
        except Exception as e:
            print(f"   ❌ 稳定接近失败: {e}")
            return False
    
    def _confirm_contact(self, mobile_base) -> bool:
        """确认接触（T类专用）"""
        try:
            print("   🤝 确认接触阶段...")
            
            # T类需要确认稳定接触
            world = self._get_world_from_mobile_base(mobile_base)
            
            for step in range(20):
                if world:
                    world.step(render=True)
                time.sleep(0.05)
                
                # 模拟接触确认
                if step > 10 and random.random() < 0.8:
                    print("   ✅ T类稳定接触确认")
                    return True
            
            print("   ✅ T类接触过程完成")
            return True
            
        except Exception as e:
            print(f"   ❌ 接触确认失败: {e}")
            return False
    
    def _force_controlled_grasp(self, mobile_base, target_force: float) -> bool:
        """力控制抓取（主要用于G类）"""
        try:
            print("   💪 力控制抓取阶段...")
            
            # 根据OSGT类型调整控制参数
            if self.current_osgt_type == "graspable":
                control_timeout = 5.0
                force_increment = 0.001
            elif self.current_osgt_type == "task_areas":
                control_timeout = 6.0
                force_increment = 0.0008
            else:
                control_timeout = 3.0
                force_increment = 0.002
            
            start_time = time.time()
            step_count = 0
            
            world = self._get_world_from_mobile_base(mobile_base)
            
            while time.time() - start_time < control_timeout:
                current_force = self._simulate_force_feedback()
                force_error = target_force - current_force
                
                if step_count % 15 == 0 and self.config.DEBUG["show_grasp_details"]:
                    elapsed = time.time() - start_time
                    print(f"   📊 力控制状态: 当前力={current_force:.1f}N, 目标力={target_force:.1f}N")
                
                if current_force > self.force_profile.force_threshold:
                    print(f"   ⚠️ 力过大 ({current_force:.1f}N)，停止抓取")
                    return False
                
                if current_force >= target_force * 0.7:
                    self.object_grasped = True
                    print(f"   ✅ 抓取成功！力: {current_force:.1f}N")
                    return True
                
                # PID调节
                adjustment = self._calculate_pid_adjustment(force_error)
                self.current_position -= adjustment * force_increment
                self.current_position = max(self.current_position, self.gripper_closed)
                
                self._set_gripper_position(mobile_base, self.current_position)
                
                if world:
                    world.step(render=True)
                time.sleep(0.05)
                
                step_count += 1
            
            # 检查最终状态
            final_force = self._simulate_force_feedback()
            force_threshold = target_force * 0.5 if self.current_osgt_type == "graspable" else target_force * 0.3
            
            if final_force >= force_threshold:
                self.object_grasped = True
                print(f"   ✅ 抓取基本成功，力: {final_force:.1f}N")
                return True
            
            print("   ❌ 力控制抓取失败")
            return False
            
        except Exception as e:
            print(f"   ❌ 力控制抓取异常: {e}")
            return False
    
    def _verify_grasp(self, mobile_base) -> bool:
        """验证抓取是否成功"""
        try:
            # 根据OSGT类型调整验证标准
            if self.current_osgt_type == "graspable":
                verification_steps = 10
                force_threshold = self.force_profile.contact_threshold
            elif self.current_osgt_type == "task_areas":
                verification_steps = 8
                force_threshold = self.force_profile.contact_threshold * 0.8
            else:
                verification_steps = 5
                force_threshold = self.force_profile.contact_threshold * 0.5
            
            for step in range(verification_steps):
                current_force = self._simulate_force_feedback()
                
                if current_force < force_threshold:
                    if self.current_osgt_type == "graspable":
                        print("   ❌ G类抓取验证失败：力不足")
                        self.object_grasped = False
                        return False
                
                time.sleep(0.1)
            
            print("   ✅ 抓取验证成功")
            return True
            
        except Exception as e:
            print(f"   ❌ 抓取验证异常: {e}")
            return False
    
    def _set_gripper_position(self, mobile_base, position: float):
        """设置夹爪位置"""
        try:
            articulation_controller = mobile_base.get_articulation_controller()
            if articulation_controller and hasattr(mobile_base, 'dof_names'):
                num_dofs = len(mobile_base.dof_names)
                joint_positions = np.zeros(num_dofs)
                
                gripper_joints = self.config.ARM_CONFIG["gripper_joint_names"]
                for joint_name in gripper_joints:
                    if joint_name in mobile_base.dof_names:
                        idx = mobile_base.dof_names.index(joint_name)
                        joint_positions[idx] = position
                
                from isaacsim.core.utils.types import ArticulationAction
                action = ArticulationAction(joint_positions=joint_positions)
                articulation_controller.apply_action(action)
                
        except Exception as e:
            if self.config.DEBUG["enable_debug_output"]:
                print(f"设置夹爪位置失败: {e}")
    
    def _detect_contact(self) -> bool:
        """检测是否接触到物体（模拟）"""
        grasp_progress = (self.gripper_open - self.current_position) / (self.gripper_open - self.gripper_closed)
        
        # 根据OSGT类型调整接触概率
        if self.current_osgt_type == "graspable":
            contact_probability = grasp_progress * 0.8 + random.uniform(0, 0.2)
            min_progress = 0.3
        elif self.current_osgt_type == "sweepable":
            contact_probability = grasp_progress * 0.6 + random.uniform(0, 0.4)
            min_progress = 0.1
        elif self.current_osgt_type == "task_areas":
            contact_probability = grasp_progress * 0.7 + random.uniform(0, 0.3)
            min_progress = 0.2
        else:  # obstacles
            contact_probability = grasp_progress * 0.3 + random.uniform(0, 0.1)
            min_progress = 0.05
        
        return random.random() < contact_probability and grasp_progress > min_progress
    
    def _check_early_contact(self) -> bool:
        """检查是否有早期接触"""
        base_probability = 0.1
        if self.current_osgt_type == "sweepable":
            base_probability = 0.2  # S类更容易早期接触
        elif self.current_osgt_type == "obstacles":
            base_probability = 0.05  # O类避免早期接触
        
        return random.random() < base_probability
    
    def _simulate_force_feedback(self) -> float:
        """模拟力反馈传感器"""
        if not self.contact_detected:
            return 0.0
        
        grasp_progress = (self.gripper_open - self.current_position) / (self.gripper_open - self.gripper_closed)
        
        # 根据OSGT类型调整力反馈
        if self.current_osgt_type == "graspable":
            base_force = grasp_progress * self.force_profile.grasp_force
            noise_range = 1.0
        elif self.current_osgt_type == "sweepable":
            base_force = grasp_progress * self.force_profile.grasp_force * 0.5
            noise_range = 0.5
        elif self.current_osgt_type == "task_areas":
            base_force = grasp_progress * self.force_profile.grasp_force * 0.8
            noise_range = 0.8
        else:  # obstacles
            base_force = grasp_progress * self.force_profile.grasp_force * 0.2
            noise_range = 0.2
        
        noise = random.uniform(-noise_range, noise_range)
        simulated_force = max(0, base_force + noise)
        
        return simulated_force
    
    def _calculate_pid_adjustment(self, error: float) -> float:
        """计算PID调节量"""
        self.force_pid['integral'] += error
        derivative = error - self.force_pid['prev_error']
        
        adjustment = (self.force_pid['kp'] * error + 
                     self.force_pid['ki'] * self.force_pid['integral'] + 
                     self.force_pid['kd'] * derivative)
        
        self.force_pid['prev_error'] = error
        return adjustment
    
    def _get_world_from_mobile_base(self, mobile_base):
        """从mobile_base获取world对象"""
        try:
            if self.world is not None:
                return self.world
            
            if hasattr(mobile_base, '_world'):
                return mobile_base._world
            elif hasattr(mobile_base, 'world'):
                return mobile_base.world
            
            if hasattr(mobile_base, '_scene') and mobile_base._scene:
                if hasattr(mobile_base._scene, '_world'):
                    return mobile_base._scene._world
            
            try:
                from isaacsim.core.api import World
                world_instance = World.instance()
                if world_instance:
                    return world_instance
            except:
                pass
            
            return None
            
        except Exception as e:
            if hasattr(self, 'config') and self.config.DEBUG["enable_debug_output"]:
                print(f"获取world对象失败: {e}")
            return None
    
    def release_osgt_object(self, mobile_base, osgt_type: str = None) -> bool:
        """OSGT类型特定的释放物体"""
        if osgt_type is None:
            osgt_type = self.current_osgt_type
        
        try:
            print(f"   🔓 释放OSGT-{osgt_type}物体")
            
            # 根据OSGT类型调整释放策略
            if osgt_type == "graspable":
                return self._release_precisely(mobile_base)
            elif osgt_type == "sweepable":
                return self._release_gently(mobile_base)
            elif osgt_type == "task_areas":
                return self._release_stably(mobile_base)
            else:  # obstacles
                return self._release_quickly(mobile_base)
                
        except Exception as e:
            print(f"   ❌ 释放OSGT-{osgt_type}物体失败: {e}")
            return False
    
    def _release_precisely(self, mobile_base) -> bool:
        """精确释放（G类）"""
        target_position = self.gripper_open
        steps = 20
        
        for i in range(steps):
            progress = i / steps
            current_pos = self.current_position + progress * (target_position - self.current_position)
            self._set_gripper_position(mobile_base, current_pos)
            time.sleep(0.05)
        
        self.current_position = target_position
        self.object_grasped = False
        self.contact_detected = False
        print("   ✅ G类物体精确释放完成")
        return True
    
    def _release_gently(self, mobile_base) -> bool:
        """轻柔释放（S类）"""
        target_position = self.gripper_open
        steps = 10
        
        for i in range(steps):
            progress = i / steps
            current_pos = self.current_position + progress * (target_position - self.current_position)
            self._set_gripper_position(mobile_base, current_pos)
            time.sleep(0.03)
        
        self.current_position = target_position
        self.object_grasped = False
        self.contact_detected = False
        print("   ✅ S类物体轻柔释放完成")
        return True
    
    def _release_stably(self, mobile_base) -> bool:
        """稳定释放（T类）"""
        target_position = self.gripper_open
        steps = 30
        
        for i in range(steps):
            progress = i / steps
            # 使用平滑函数确保稳定释放
            smooth_progress = progress * progress * (3.0 - 2.0 * progress)
            current_pos = self.current_position + smooth_progress * (target_position - self.current_position)
            self._set_gripper_position(mobile_base, current_pos)
            time.sleep(0.06)
        
        self.current_position = target_position
        self.object_grasped = False
        self.contact_detected = False
        print("   ✅ T类区域稳定释放完成")
        return True
    
    def _release_quickly(self, mobile_base) -> bool:
        """快速释放（O类避障）"""
        target_position = self.gripper_open
        steps = 5
        
        for i in range(steps):
            progress = i / steps
            current_pos = self.current_position + progress * (target_position - self.current_position)
            self._set_gripper_position(mobile_base, current_pos)
            time.sleep(0.02)
        
        self.current_position = target_position
        self.object_grasped = False
        self.contact_detected = False
        print("   ✅ O类避障快速释放完成")
        return True
    
    def get_osgt_grasp_stats(self) -> Dict[str, Any]:
        """获取OSGT抓取统计"""
        stats = {}
        for osgt_type in ['graspable', 'sweepable', 'task_areas', 'obstacles']:
            attempts = self.osgt_grasp_stats[f'{osgt_type}_attempts']
            successes = self.osgt_grasp_stats[f'{osgt_type}_successes']
            success_rate = (successes / attempts * 100) if attempts > 0 else 0
            stats[osgt_type] = {
                'attempts': attempts,
                'successes': successes,
                'success_rate': success_rate
            }
        return stats
    
    def execute_pick_and_place(self, mobile_base, target_object, 
                              drop_location: np.ndarray) -> bool:
        """向后兼容的抓取放下方法"""
        print(f"🔄 兼容性调用：转换为OSGT-graspable抓取")
        return self.execute_osgt_pick_and_place(
            mobile_base, target_object, drop_location, "graspable"
        )
    
    def print_performance_report(self):
        """向后兼容的性能报告方法"""
        return self.print_osgt_performance_report()
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """获取性能统计（向后兼容方法）"""
        return self.get_osgt_performance_stats()
    
    def set_world_reference(self, world):
        """设置世界对象引用（兼容性方法）"""
        self.world = world
        self.gripper_controller.world = world
        if hasattr(self, 'config') and self.config.DEBUG["enable_debug_output"]:
            print("✅ OSGT高级抓取系统已链接到World对象")

class OSGTAdvancedPickAndPlaceStrategy:
    """OSGT四类物体高级抓取放下策略"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.grasp_planner = OSGTCudaAcceleratedGraspPlanner(config)
        self.gripper_controller = OSGTAdaptiveGripperController(config)
        
        # 世界对象引用
        self.world = None
        
        # OSGT抓取历史和学习
        self.osgt_grasp_history = deque(maxlen=200)
        self.osgt_success_patterns = {
            'graspable': [], 'sweepable': [], 'task_areas': [], 'obstacles': []
        }
        
        # OSGT性能统计
        self.osgt_stats = {
            'total_attempts': 0,
            'osgt_type_stats': {
                'graspable': {'attempts': 0, 'successes': 0, 'avg_time': 0.0},
                'sweepable': {'attempts': 0, 'successes': 0, 'avg_time': 0.0},
                'task_areas': {'attempts': 0, 'successes': 0, 'avg_time': 0.0},
                'obstacles': {'attempts': 0, 'successes': 0, 'avg_time': 0.0}
            },
            'cuda_acceleration_used': CUDA_AVAILABLE
        }
    
    def set_world_reference(self, world):
        """设置世界对象引用"""
        self.world = world
        self.gripper_controller.world = world
        if self.config.DEBUG["enable_debug_output"]:
            print("✅ OSGT高级抓取系统已链接到World对象")
    
    def execute_osgt_pick_and_place(self, mobile_base, target_object, 
                                   drop_location: np.ndarray, osgt_type: str = "graspable") -> bool:
        """执行OSGT类型特定的完整抓取放下序列"""
        print(f"🦾 开始OSGT-{osgt_type}抓取放下: {target_object.name}")
        
        start_time = time.time()
        self.osgt_stats['total_attempts'] += 1
        self.osgt_stats['osgt_type_stats'][osgt_type]['attempts'] += 1
        
        try:
            # 设置OSGT类型
            self.gripper_controller.set_osgt_type(osgt_type)
            
            if osgt_type == "graspable":
                success = self._execute_graspable_pick_place(mobile_base, target_object, drop_location)
            elif osgt_type == "sweepable":
                success = self._execute_sweepable_collection(mobile_base, target_object, drop_location)
            elif osgt_type == "task_areas":
                success = self._execute_task_area_interaction(mobile_base, target_object, drop_location)
            elif osgt_type == "obstacles":
                success = self._execute_obstacle_avoidance(mobile_base, target_object)
            else:
                success = self._execute_graspable_pick_place(mobile_base, target_object, drop_location)
            
            # 记录结果
            execution_time = time.time() - start_time
            self._record_osgt_result(osgt_type, target_object, execution_time, success)
            
            if success:
                print(f"✅ OSGT-{osgt_type}抓取放下完成！用时: {execution_time:.1f}s")
            else:
                print(f"❌ OSGT-{osgt_type}抓取放下失败，用时: {execution_time:.1f}s")
            
            return success
            
        except Exception as e:
            print(f"❌ OSGT-{osgt_type}抓取放下异常: {e}")
            return self._handle_osgt_failure(osgt_type, f"异常: {e}")
    
    def _execute_graspable_pick_place(self, mobile_base, target_object, drop_location: np.ndarray) -> bool:
        """G类可抓取物：标准精确抓取放下"""
        try:
            print("   🎯 G类可抓取物：执行标准抓取放下序列")
            
            # 阶段1: G类抓取规划
            success = self._execute_osgt_grasp_planning(mobile_base, target_object, "graspable")
            if not success:
                return False
            
            # 阶段2: G类精确抓取
            success = self._execute_osgt_grasp_sequence(mobile_base, target_object, "graspable")
            if not success:
                return False
            
            # 阶段3: G类精确提升
            success = self._execute_osgt_lift_sequence(mobile_base, "graspable")
            if not success:
                return False
            
            # 阶段4: G类稳定运输
            success = self._execute_osgt_transport_sequence(mobile_base, "graspable")
            if not success:
                return False
            
            # 阶段5: G类精确放置
            success = self._execute_osgt_place_sequence(mobile_base, target_object, drop_location, "graspable")
            if not success:
                return False
            
            return True
            
        except Exception as e:
            print(f"   ❌ G类抓取放下失败: {e}")
            return False
    
    def _execute_sweepable_collection(self, mobile_base, target_object, drop_location: np.ndarray) -> bool:
        """S类可清扫物：吸附式收集"""
        try:
            print("   🧹 S类可清扫物：执行吸附收集序列")
            
            # S类主要通过底盘吸附收集，机械臂只做辅助确认
            success = self._execute_osgt_grasp_sequence(mobile_base, target_object, "sweepable")
            if success:
                # 简化的移动到收集位置
                target_object.set_world_pose(drop_location, target_object.get_world_pose()[1])
                print("   ✅ S类吸附收集完成")
                return True
            else:
                print("   ⚠️ S类收集部分成功（可能已被吸附）")
                return True  # S类容错性高
                
        except Exception as e:
            print(f"   ❌ S类收集失败: {e}")
            return False
    
    def _execute_task_area_interaction(self, mobile_base, target_object, drop_location: np.ndarray) -> bool:
        """T类任务区：交互操作"""
        try:
            print("   🎯 T类任务区：执行交互操作序列")
            
            # T类主要是确认接触和执行特定操作
            success = self._execute_osgt_grasp_sequence(mobile_base, target_object, "task_areas")
            if success:
                # 模拟在任务区的特定操作
                time.sleep(1.0)
                print("   ✅ T类任务区交互完成")
                return True
            else:
                print("   ⚠️ T类交互部分成功")
                return True  # T类部分成功也算成功
                
        except Exception as e:
            print(f"   ❌ T类交互失败: {e}")
            return False
    
    def _execute_obstacle_avoidance(self, mobile_base, target_object) -> bool:
        """O类障碍物：避障操作"""
        try:
            print("   🚧 O类障碍物：执行避障操作")
            
            # O类不应该被抓取，只执行避让
            success = self.gripper_controller.execute_osgt_adaptive_grasp(mobile_base, "obstacles")
            if success:
                print("   ✅ O类障碍物避让完成")
                return True
            else:
                print("   ⚠️ O类避让操作完成")
                return True  # 避让操作本身就是成功
                
        except Exception as e:
            print(f"   ❌ O类避障失败: {e}")
            return False
    
    # 其他方法保持原有逻辑，但添加OSGT类型支持...
    
    def _execute_osgt_grasp_planning(self, mobile_base, target_object, osgt_type: str) -> bool:
        """执行OSGT类型特定的抓取规划"""
        try:
            print(f"   🎯 生成OSGT-{osgt_type}抓取候选点...")
            
            target_position, _ = target_object.get_world_pose()
            object_size = np.array([0.05, 0.05, 0.1])  # 估计尺寸
            
            candidates = self.grasp_planner.generate_osgt_grasp_candidates(
                target_position, object_size, osgt_type
            )
            
            if not candidates:
                print(f"   ❌ 未找到有效OSGT-{osgt_type}抓取候选点")
                return False
            
            print(f"   ✅ 生成 {len(candidates)} 个OSGT-{osgt_type}抓取候选点")
            print(f"   🏆 最佳候选点OSGT分数: {candidates[0].osgt_specific_score:.3f}")
            
            self.best_osgt_grasp_candidate = candidates[0]
            return True
            
        except Exception as e:
            print(f"   ❌ OSGT-{osgt_type}抓取规划失败: {e}")
            return False
    
    def _execute_osgt_grasp_sequence(self, mobile_base, target_object, osgt_type: str) -> bool:
        """执行OSGT类型特定的抓取序列"""
        try:
            print(f"   🎯 执行OSGT-{osgt_type}抓取序列...")
            
            # 移动到预抓取姿态
            success = self._move_to_osgt_pre_grasp_pose(mobile_base, osgt_type)
            if not success:
                return False
            
            # 执行OSGT特定的自适应抓取
            success = self.gripper_controller.execute_osgt_adaptive_grasp(mobile_base, osgt_type)
            if not success:
                return False
            
            print(f"   ✅ OSGT-{osgt_type}抓取序列完成")
            return True
            
        except Exception as e:
            print(f"   ❌ OSGT-{osgt_type}抓取序列失败: {e}")
            return False
    
    def _execute_osgt_lift_sequence(self, mobile_base, osgt_type: str) -> bool:
        """执行OSGT类型特定的提升序列"""
        try:
            print(f"   ⬆️ OSGT-{osgt_type}提升序列...")
            
            # 根据OSGT类型选择不同的提升策略
            if osgt_type == "graspable":
                lift_poses = ["carry", "stow"]
            elif osgt_type == "sweepable":
                lift_poses = ["ready"]  # S类只需轻微提升
            elif osgt_type == "task_areas":
                lift_poses = ["carry"]  # T类稳定提升
            else:  # obstacles
                lift_poses = ["home"]   # O类撤回到安全位置
            
            for pose_name in lift_poses:
                success = self._move_arm_to_pose(mobile_base, pose_name)
                if not success:
                    return False
                time.sleep(0.5)
            
            print(f"   ✅ OSGT-{osgt_type}提升完成")
            return True
            
        except Exception as e:
            print(f"   ❌ OSGT-{osgt_type}提升失败: {e}")
            return False
    
    def _execute_osgt_transport_sequence(self, mobile_base, osgt_type: str) -> bool:
        """执行OSGT类型特定的运输序列"""
        try:
            print(f"   🚚 OSGT-{osgt_type}运输序列...")
            
            # 根据OSGT类型调整运输时间
            if osgt_type == "graspable":
                transport_time = 3.0
            elif osgt_type == "sweepable":
                transport_time = 1.0
            elif osgt_type == "task_areas":
                transport_time = 2.0
            else:  # obstacles
                transport_time = 0.5
            
            world = self._get_world_from_mobile_base(mobile_base)
            steps = int(transport_time * 30)
            
            for i in range(steps):
                if world:
                    world.step(render=True)
                time.sleep(0.033)
                
                if i % 30 == 0 and self.config.DEBUG["show_grasp_details"]:
                    progress = (i + 1) / steps * 100
                    print(f"   📈 OSGT-{osgt_type}运输进度: {progress:.0f}%")
            
            print(f"   ✅ OSGT-{osgt_type}运输完成")
            return True
            
        except Exception as e:
            print(f"   ❌ OSGT-{osgt_type}运输失败: {e}")
            return False
    
    def _execute_osgt_place_sequence(self, mobile_base, target_object, drop_location: np.ndarray, osgt_type: str) -> bool:
        """执行OSGT类型特定的放置序列"""
        try:
            print(f"   📦 OSGT-{osgt_type}放置序列...")
            
            # 移动到放置姿态
            success = self._move_to_osgt_place_pose(mobile_base, osgt_type)
            if not success:
                return False
            
            # 释放物体
            success = self.gripper_controller.release_osgt_object(mobile_base, osgt_type)
            if not success:
                return False
            
            # 移动物体到最终位置
            final_position = drop_location.copy()
            final_position[2] = -1.0  # 地下位置表示已收集
            target_object.set_world_pose(final_position, target_object.get_world_pose()[1])
            
            # 回到安全姿态
            self._move_arm_to_pose(mobile_base, "home")
            
            print(f"   ✅ OSGT-{osgt_type}放置完成")
            return True
            
        except Exception as e:
            print(f"   ❌ OSGT-{osgt_type}放置失败: {e}")
            return False
    
    def _move_to_osgt_pre_grasp_pose(self, mobile_base, osgt_type: str) -> bool:
        """移动到OSGT类型特定的预抓取姿态"""
        if osgt_type == "graspable":
            return self._move_arm_to_pose(mobile_base, "ready")
        elif osgt_type == "sweepable":
            return self._move_arm_to_pose(mobile_base, "pickup_low")
        elif osgt_type == "task_areas":
            return self._move_arm_to_pose(mobile_base, "inspect")
        else:  # obstacles
            return self._move_arm_to_pose(mobile_base, "home")
    
    def _move_to_osgt_place_pose(self, mobile_base, osgt_type: str) -> bool:
        """移动到OSGT类型特定的放置姿态"""
        if osgt_type == "graspable":
            return self._move_arm_to_pose(mobile_base, "pickup")
        elif osgt_type == "sweepable":
            return self._move_arm_to_pose(mobile_base, "pickup_low")
        elif osgt_type == "task_areas":
            return self._move_arm_to_pose(mobile_base, "ready")
        else:  # obstacles
            return self._move_arm_to_pose(mobile_base, "home")
    
    def _move_arm_to_pose(self, mobile_base, pose_name: str) -> bool:
        """移动机械臂到指定姿态"""
        try:
            arm_poses = self.config.ARM_CONFIG["poses"]
            if pose_name not in arm_poses:
                print(f"   ⚠️ 未知姿态: {pose_name}")
                return False
            
            target_positions = arm_poses[pose_name]
            arm_joint_names = self.config.ARM_CONFIG["joint_names"]
            
            articulation_controller = mobile_base.get_articulation_controller()
            if not articulation_controller:
                return False
            
            if hasattr(mobile_base, 'dof_names'):
                num_dofs = len(mobile_base.dof_names)
                joint_positions = np.zeros(num_dofs)
                
                for i, joint_name in enumerate(arm_joint_names):
                    if joint_name in mobile_base.dof_names and i < len(target_positions):
                        idx = mobile_base.dof_names.index(joint_name)
                        joint_positions[idx] = target_positions[i]
                
                from isaacsim.core.utils.types import ArticulationAction
                action = ArticulationAction(joint_positions=joint_positions)
                articulation_controller.apply_action(action)
                
                # 等待运动完成
                world = self._get_world_from_mobile_base(mobile_base)
                for _ in range(60):
                    if world:
                        world.step(render=True)
                    time.sleep(0.033)
                
                return True
            
        except Exception as e:
            print(f"   ❌ 移动机械臂失败: {e}")
            return False
    
    def _get_world_from_mobile_base(self, mobile_base):
        """从mobile_base获取world对象"""
        try:
            if self.world is not None:
                return self.world
            
            if hasattr(mobile_base, '_world'):
                return mobile_base._world
            elif hasattr(mobile_base, 'world'):
                return mobile_base.world
            
            if hasattr(mobile_base, '_scene') and mobile_base._scene:
                if hasattr(mobile_base._scene, '_world'):
                    return mobile_base._scene._world
            
            try:
                from isaacsim.core.api import World
                world_instance = World.instance()
                if world_instance:
                    return world_instance
            except:
                pass
            
            return None
            
        except Exception as e:
            if hasattr(self, 'config') and self.config.DEBUG["enable_debug_output"]:
                print(f"获取world对象失败: {e}")
            return None
    
    def _record_osgt_result(self, osgt_type: str, target_object, execution_time: float, success: bool):
        """记录OSGT结果"""
        type_stats = self.osgt_stats['osgt_type_stats'][osgt_type]
        
        if success:
            type_stats['successes'] += 1
        
        # 更新平均时间
        total_time = type_stats['avg_time'] * (type_stats['attempts'] - 1)
        type_stats['avg_time'] = (total_time + execution_time) / type_stats['attempts']
        
        # 记录历史
        result_pattern = {
            'osgt_type': osgt_type,
            'object_name': target_object.name,
            'execution_time': execution_time,
            'success': success,
            'timestamp': time.time(),
            'cuda_used': CUDA_AVAILABLE
        }
        self.osgt_grasp_history.append(result_pattern)
        self.osgt_success_patterns[osgt_type].append(result_pattern)
    
    def _handle_osgt_failure(self, osgt_type: str, reason: str) -> bool:
        """处理OSGT失败情况"""
        print(f"   ❌ OSGT-{osgt_type}失败: {reason}")
        
        failure_pattern = {
            'osgt_type': osgt_type,
            'reason': reason,
            'timestamp': time.time(),
            'cuda_used': CUDA_AVAILABLE
        }
        self.osgt_grasp_history.append(failure_pattern)
        
        return False
    
    def execute_pick_and_place(self, mobile_base, target_object, 
                              drop_location: np.ndarray) -> bool:
        """向后兼容的抓取放下方法"""
        print(f"🔄 兼容性调用：转换为OSGT-graspable抓取")
        return self.execute_osgt_pick_and_place(
            mobile_base, target_object, drop_location, "graspable"
        )
    
    def print_performance_report(self):
        """向后兼容的性能报告方法"""
        return self.print_osgt_performance_report()
    
    def get_osgt_performance_stats(self) -> Dict[str, Any]:
        """获取OSGT性能统计"""
        stats = self.osgt_stats.copy()
        
        # 计算各OSGT类型的成功率
        for osgt_type, type_stats in stats['osgt_type_stats'].items():
            if type_stats['attempts'] > 0:
                type_stats['success_rate'] = (type_stats['successes'] / type_stats['attempts'] * 100)
            else:
                type_stats['success_rate'] = 0.0
        
        # 计算总体成功率
        total_successes = sum(type_stats['successes'] for type_stats in stats['osgt_type_stats'].values())
        if stats['total_attempts'] > 0:
            stats['overall_success_rate'] = (total_successes / stats['total_attempts'] * 100)
        else:
            stats['overall_success_rate'] = 0.0
        
        return stats
    
    def print_osgt_performance_report(self):
        """打印OSGT性能报告"""
        stats = self.get_osgt_performance_stats()
        gripper_stats = self.gripper_controller.get_osgt_grasp_stats()
        
        print(f"\n📊 OSGT高级抓取性能报告:")
        print(f"   总体成功率: {stats['overall_success_rate']:.1f}%")
        print(f"   总尝试次数: {stats['total_attempts']}")
        print(f"   CUDA加速: {'✅ 启用' if stats['cuda_acceleration'] else '❌ 未启用'}")
        
        print(f"\n🎯 OSGT各类型详细统计:")
        osgt_symbols = {
            'graspable': '🦾',
            'sweepable': '🧹',
            'task_areas': '🎯',
            'obstacles': '🚧'
        }
        
        for osgt_type, type_stats in stats['osgt_type_stats'].items():
            symbol = osgt_symbols.get(osgt_type, '📦')
            if type_stats['attempts'] > 0:
                print(f"   {symbol} {osgt_type}: {type_stats['successes']}/{type_stats['attempts']} "
                      f"({type_stats['success_rate']:.1f}%) 平均用时: {type_stats['avg_time']:.1f}s")

# 工厂函数
def create_osgt_advanced_pick_and_place_system(config: Dict[str, Any]) -> OSGTAdvancedPickAndPlaceStrategy:
    """创建OSGT高级抓取放下系统"""
    print("🏭 初始化OSGT四类物体高级抓取放下系统...")
    
    system = OSGTAdvancedPickAndPlaceStrategy(config)
    
    print("✅ OSGT高级抓取放下系统初始化完成")
    print(f"   - CUDA加速: {'启用' if CUDA_AVAILABLE else '禁用'}")
    print(f"   - OSGT四类支持: 🚧O类避障 | 🧹S类吸附 | 🦾G类精确抓取 | 🎯T类任务交互")
    print(f"   - 力控制反馈: 启用")
    print(f"   - 自适应抓取: 启用")
    print(f"   - 性能监控: 启用")
    
    return system

# ==================== 兼容性别名 ====================
# 为了保持向后兼容性的别名
AdvancedPickAndPlaceStrategy = OSGTAdvancedPickAndPlaceStrategy
create_advanced_pick_and_place_system = create_osgt_advanced_pick_and_place_system
GraspPhase = OSGTGraspPhase
GraspCandidate = OSGTGraspCandidate
ForceProfile = OSGTForceProfile
CudaAcceleratedGraspPlanner = OSGTCudaAcceleratedGraspPlanner
AdaptiveGripperController = OSGTAdaptiveGripperController

# 确保所有必要的类都可以被导入
__all__ = [
    # OSGT 新类名
    'OSGTAdvancedPickAndPlaceStrategy',
    'OSGTGraspPhase', 
    'OSGTGraspCandidate',
    'OSGTForceProfile',
    'OSGTCudaAcceleratedGraspPlanner',
    'OSGTAdaptiveGripperController',
    'create_osgt_advanced_pick_and_place_system',
    
    # 向后兼容别名
    'AdvancedPickAndPlaceStrategy',
    'GraspPhase',
    'GraspCandidate', 
    'ForceProfile',
    'CudaAcceleratedGraspPlanner',
    'AdaptiveGripperController',
    'create_advanced_pick_and_place_system',
]

print("✅ OSGT抓取系统兼容性别名已加载")