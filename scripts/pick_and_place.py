#!/usr/bin/env python3
"""
高级机械臂抓取放下封装模块 - pick_and_place.py
基于Auromix auro_sim项目的抓取原理，集成CUDA加速和力控制反馈
用于Create-3+机械臂的大垃圾精确抓取和放置
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
    print("✅ CUDA加速可用于抓取规划")
except ImportError:
    CUDA_AVAILABLE = False
    cp = np
    print("⚠️ CUDA不可用，使用CPU抓取规划")

try:
    from scipy.spatial.transform import Rotation as R
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False
    print("⚠️ SciPy不可用，使用简化旋转计算")

class GraspPhase(Enum):
    """抓取阶段枚举"""
    IDLE = "idle"
    APPROACHING = "approaching"
    GRASPING = "grasping"
    LIFTING = "lifting"
    TRANSPORTING = "transporting"
    PLACING = "placing"
    RETRACTING = "retracting"
    COMPLETED = "completed"
    FAILED = "failed"

@dataclass
class GraspCandidate:
    """抓取候选点数据结构"""
    position: np.ndarray      # 抓取位置 [x, y, z]
    orientation: np.ndarray   # 抓取方向 [roll, pitch, yaw]
    approach_vector: np.ndarray  # 接近向量
    quality_score: float      # 抓取质量分数 (0-1)
    collision_free: bool      # 是否无碰撞
    reachable: bool          # 是否可达
    
class ForceProfile:
    """力控制配置文件"""
    def __init__(self):
        self.approach_force = 2.0     # 接近时的最大力 (N)
        self.grasp_force = 15.0       # 抓取时的目标力 (N)
        self.lift_force = 20.0        # 提升时的最大力 (N)
        self.transport_force = 10.0   # 运输时的保持力 (N)
        self.force_threshold = 25.0   # 力阈值，超过则停止 (N)
        self.contact_threshold = 3.0  # 接触检测阈值 (N)

class CudaAcceleratedGraspPlanner:
    """CUDA加速的抓取规划器"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.use_cuda = CUDA_AVAILABLE
        
        # 抓取参数
        self.gripper_width = 0.08  # 夹爪最大开口 (m)
        self.approach_distance = 0.15  # 接近距离 (m)
        self.lift_height = 0.2     # 提升高度 (m)
        
        # CUDA优化参数
        self.batch_size = 64 if self.use_cuda else 8
        self.max_candidates = 256
        
        # 预定义抓取模板
        self.grasp_templates = self._generate_grasp_templates()
        
        if config.DEBUG["enable_debug_output"]:
            print(f"🔧 抓取规划器初始化: CUDA={self.use_cuda}, 批处理={self.batch_size}")
    
    def _generate_grasp_templates(self) -> List[np.ndarray]:
        """生成预定义抓取模板"""
        templates = []
        
        # 顶部垂直抓取
        templates.append(np.array([0, 0, -1, 0, 0, 0]))  # [dx, dy, dz, roll, pitch, yaw]
        
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
        
        return templates
    
    def generate_grasp_candidates(self, target_position: np.ndarray, 
                                object_size: np.ndarray) -> List[GraspCandidate]:
        """生成抓取候选点"""
        candidates = []
        
        try:
            # 使用模板生成候选点
            for template in self.grasp_templates:
                # 计算抓取位置
                approach_vector = template[:3]
                orientation = template[3:]
                
                # 考虑物体大小调整接近距离
                object_radius = np.max(object_size) / 2
                adjusted_distance = self.approach_distance + object_radius
                
                grasp_position = target_position + approach_vector * adjusted_distance
                
                # 创建候选点
                candidate = GraspCandidate(
                    position=grasp_position,
                    orientation=orientation,
                    approach_vector=approach_vector,
                    quality_score=0.0,
                    collision_free=True,
                    reachable=True
                )
                
                candidates.append(candidate)
            
            # CUDA加速的质量评估
            if self.use_cuda and len(candidates) > 0:
                candidates = self._cuda_evaluate_candidates(candidates, target_position)
            else:
                candidates = self._cpu_evaluate_candidates(candidates, target_position)
            
            # 按质量分数排序
            candidates.sort(key=lambda x: x.quality_score, reverse=True)
            
            return candidates[:self.max_candidates]
            
        except Exception as e:
            print(f"❌ 生成抓取候选点失败: {e}")
            return []
    
    def _cuda_evaluate_candidates(self, candidates: List[GraspCandidate], 
                                target_pos: np.ndarray) -> List[GraspCandidate]:
        """CUDA加速的候选点评估"""
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
            
            # 方向分数（垂直向下抓取得分更高）
            approach_vectors = cp.array([c.approach_vector for c in candidates])
            vertical_preference = cp.abs(approach_vectors[:, 2])  # z分量
            
            # 综合分数
            quality_scores = 0.6 * distance_scores + 0.4 * vertical_preference
            
            # 添加随机扰动避免完全确定性
            noise = cp.random.normal(0, 0.05, len(quality_scores))
            quality_scores += noise
            
            # 限制到[0,1]范围
            quality_scores = cp.clip(quality_scores, 0, 1)
            
            # 更新候选点分数
            scores_cpu = cp.asnumpy(quality_scores)
            for i, candidate in enumerate(candidates):
                candidate.quality_score = float(scores_cpu[i])
            
            return candidates
            
        except Exception as e:
            print(f"⚠️ CUDA评估失败，回退到CPU: {e}")
            return self._cpu_evaluate_candidates(candidates, target_pos)
    
    def _cpu_evaluate_candidates(self, candidates: List[GraspCandidate], 
                               target_pos: np.ndarray) -> List[GraspCandidate]:
        """CPU版本的候选点评估"""
        for candidate in candidates:
            try:
                # 距离分数
                distance = np.linalg.norm(candidate.position - target_pos)
                distance_score = np.exp(-distance / 0.5)
                
                # 方向分数（垂直向下优先）
                vertical_score = abs(candidate.approach_vector[2])
                
                # 可达性检查（简化版）
                reachability_score = self._check_reachability(candidate.position)
                
                # 综合分数
                candidate.quality_score = (
                    0.5 * distance_score + 
                    0.3 * vertical_score + 
                    0.2 * reachability_score
                )
                
                # 添加随机扰动
                candidate.quality_score += random.uniform(-0.05, 0.05)
                candidate.quality_score = max(0, min(1, candidate.quality_score))
                
            except Exception as e:
                candidate.quality_score = 0.0
        
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

class AdaptiveGripperController:
    """自适应夹爪控制器"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.force_profile = ForceProfile()
        
        # 夹爪参数
        self.gripper_open = config.ARM_CONFIG["gripper_open"]
        self.gripper_closed = config.ARM_CONFIG["gripper_closed"]
        self.current_position = self.gripper_open
        
        # 力控制参数
        self.force_pid = {
            'kp': 0.001,
            'ki': 0.0001,
            'kd': 0.0005,
            'integral': 0.0,
            'prev_error': 0.0
        }
        
        # 状态监控
        self.contact_detected = False
        self.object_grasped = False
        self.grasp_start_time = 0
        
    def execute_adaptive_grasp(self, mobile_base, target_force: float = None) -> bool:
        """执行自适应抓取"""
        if target_force is None:
            target_force = self.force_profile.grasp_force
        
        try:
            self.grasp_start_time = time.time()
            
            # 阶段1: 快速接近
            print("   🔧 阶段1: 快速接近目标")
            success = self._rapid_approach(mobile_base)
            if not success:
                return False
            
            # 阶段2: 慢速接触
            print("   🤏 阶段2: 慢速接触检测")
            success = self._slow_contact(mobile_base)
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
            print(f"   ❌ 自适应抓取失败: {e}")
            return False
    
    def _rapid_approach(self, mobile_base) -> bool:
        """快速接近阶段"""
        try:
            # 快速移动到接近位置
            target_position = self.gripper_open * 0.7  # 70%开口
            
            for step in range(20):
                self._set_gripper_position(mobile_base, target_position)
                time.sleep(0.05)
                
                # 检查是否有意外接触
                if self._check_early_contact():
                    print("   ⚠️ 检测到早期接触，切换到慢速模式")
                    return True
            
            self.current_position = target_position
            return True
            
        except Exception as e:
            print(f"   ❌ 快速接近失败: {e}")
            return False
    
    def _slow_contact(self, mobile_base) -> bool:
        """慢速接触检测"""
        try:
            contact_timeout = 5.0  # 5秒超时
            start_time = time.time()
            
            while time.time() - start_time < contact_timeout:
                # 缓慢闭合夹爪
                self.current_position -= 0.002  # 每步2mm
                self.current_position = max(self.current_position, self.gripper_closed)
                
                self._set_gripper_position(mobile_base, self.current_position)
                time.sleep(0.1)
                
                # 检测接触
                if self._detect_contact():
                    self.contact_detected = True
                    print(f"   ✅ 检测到接触，位置: {self.current_position:.4f}")
                    return True
                
                # 检查是否已完全闭合
                if self.current_position <= self.gripper_closed + 0.005:
                    print("   ⚠️ 夹爪完全闭合但未检测到物体")
                    return False
            
            print("   ⚠️ 接触检测超时")
            return False
            
        except Exception as e:
            print(f"   ❌ 慢速接触失败: {e}")
            return False
    
    def _force_controlled_grasp(self, mobile_base, target_force: float) -> bool:
        """力控制抓取"""
        try:
            control_timeout = 3.0
            start_time = time.time()
            
            while time.time() - start_time < control_timeout:
                # 模拟力传感器读数
                current_force = self._simulate_force_feedback()
                
                # PID控制
                force_error = target_force - current_force
                
                # 如果力过大，停止
                if current_force > self.force_profile.force_threshold:
                    print(f"   ⚠️ 力过大 ({current_force:.1f}N)，停止抓取")
                    return False
                
                # 如果力足够，认为抓取成功
                if current_force >= target_force * 0.8:
                    self.object_grasped = True
                    print(f"   ✅ 抓取成功，力: {current_force:.1f}N")
                    return True
                
                # PID调节
                adjustment = self._calculate_pid_adjustment(force_error)
                self.current_position -= adjustment
                self.current_position = max(self.current_position, self.gripper_closed)
                
                self._set_gripper_position(mobile_base, self.current_position)
                time.sleep(0.05)
            
            # 检查最终状态
            final_force = self._simulate_force_feedback()
            if final_force >= target_force * 0.6:
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
            # 持续监控抓取力
            for _ in range(10):
                current_force = self._simulate_force_feedback()
                
                if current_force < self.force_profile.contact_threshold:
                    print("   ❌ 抓取验证失败：力不足")
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
                
                # 设置夹爪关节
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
        # 简化的接触检测：基于夹爪位置和随机性
        grasp_progress = (self.gripper_open - self.current_position) / (self.gripper_open - self.gripper_closed)
        contact_probability = grasp_progress * 0.8 + random.uniform(0, 0.2)
        
        return random.random() < contact_probability and grasp_progress > 0.3
    
    def _check_early_contact(self) -> bool:
        """检查是否有早期接触"""
        return random.random() < 0.1  # 10%概率的早期接触
    
    def _simulate_force_feedback(self) -> float:
        """模拟力反馈传感器"""
        if not self.contact_detected:
            return 0.0
        
        # 基于夹爪位置计算模拟力
        grasp_progress = (self.gripper_open - self.current_position) / (self.gripper_open - self.gripper_closed)
        
        # 力随着闭合程度增加
        base_force = grasp_progress * self.force_profile.grasp_force
        
        # 添加噪声
        noise = random.uniform(-1, 1)
        simulated_force = max(0, base_force + noise)
        
        return simulated_force
    
    def _calculate_pid_adjustment(self, error: float) -> float:
        """计算PID调节量"""
        # PID控制
        self.force_pid['integral'] += error
        derivative = error - self.force_pid['prev_error']
        
        adjustment = (
            self.force_pid['kp'] * error + 
            self.force_pid['ki'] * self.force_pid['integral'] + 
            self.force_pid['kd'] * derivative
        )
        
        self.force_pid['prev_error'] = error
        
        # 限制调节量
        return np.clip(adjustment, -0.005, 0.005)
    
    def release_object(self, mobile_base) -> bool:
        """释放物体"""
        try:
            print("   🔓 释放物体")
            
            # 逐步打开夹爪
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
            
            print("   ✅ 物体释放完成")
            return True
            
        except Exception as e:
            print(f"   ❌ 释放物体失败: {e}")
            return False

class AdvancedPickAndPlaceStrategy:
    """高级抓取放下策略"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.grasp_planner = CudaAcceleratedGraspPlanner(config)
        self.gripper_controller = AdaptiveGripperController(config)
        
        # 抓取历史和学习
        self.grasp_history = deque(maxlen=100)
        self.success_patterns = {}
        
        # 性能统计
        self.stats = {
            'total_attempts': 0,
            'successful_grasps': 0,
            'failed_grasps': 0,
            'avg_grasp_time': 0.0,
            'cuda_acceleration_used': CUDA_AVAILABLE
        }
        
    def execute_pick_and_place(self, mobile_base, target_object, 
                             drop_location: np.ndarray) -> bool:
        """执行完整的抓取放下序列"""
        print(f"🦾 开始高级抓取放下: {target_object.name}")
        
        start_time = time.time()
        self.stats['total_attempts'] += 1
        
        try:
            # 阶段1: 抓取规划
            grasp_phase = GraspPhase.APPROACHING
            success = self._execute_grasp_planning(mobile_base, target_object)
            if not success:
                return self._handle_failure("抓取规划失败")
            
            # 阶段2: 接近和抓取
            grasp_phase = GraspPhase.GRASPING
            success = self._execute_grasp_sequence(mobile_base, target_object)
            if not success:
                return self._handle_failure("抓取执行失败")
            
            # 阶段3: 提升
            grasp_phase = GraspPhase.LIFTING
            success = self._execute_lift_sequence(mobile_base)
            if not success:
                return self._handle_failure("提升失败")
            
            # 阶段4: 运输
            grasp_phase = GraspPhase.TRANSPORTING
            success = self._execute_transport_sequence(mobile_base, drop_location)
            if not success:
                return self._handle_failure("运输失败")
            
            # 阶段5: 放置
            grasp_phase = GraspPhase.PLACING
            success = self._execute_place_sequence(mobile_base, target_object, drop_location)
            if not success:
                return self._handle_failure("放置失败")
            
            # 成功完成
            execution_time = time.time() - start_time
            self._record_success(target_object, execution_time)
            
            print(f"✅ 抓取放下完成！用时: {execution_time:.1f}s")
            return True
            
        except Exception as e:
            print(f"❌ 抓取放下异常: {e}")
            return self._handle_failure(f"异常: {e}")
    
    def _execute_grasp_planning(self, mobile_base, target_object) -> bool:
        """执行抓取规划"""
        try:
            print("   🎯 生成抓取候选点...")
            
            # 获取目标位置和尺寸
            target_position, _ = target_object.get_world_pose()
            object_size = np.array([0.05, 0.05, 0.1])  # 估计尺寸
            
            # 生成抓取候选点
            candidates = self.grasp_planner.generate_grasp_candidates(
                target_position, object_size
            )
            
            if not candidates:
                print("   ❌ 未找到有效抓取候选点")
                return False
            
            print(f"   ✅ 生成 {len(candidates)} 个抓取候选点")
            print(f"   🏆 最佳候选点质量分数: {candidates[0].quality_score:.3f}")
            
            # 存储最佳候选点
            self.best_grasp_candidate = candidates[0]
            return True
            
        except Exception as e:
            print(f"   ❌ 抓取规划失败: {e}")
            return False
    
    def _execute_grasp_sequence(self, mobile_base, target_object) -> bool:
        """执行抓取序列"""
        try:
            print("   🎯 执行精确抓取序列...")
            
            # 移动到预抓取姿态
            success = self._move_to_pre_grasp_pose(mobile_base)
            if not success:
                return False
            
            # 执行自适应抓取
            success = self.gripper_controller.execute_adaptive_grasp(mobile_base)
            if not success:
                return False
            
            print("   ✅ 抓取序列完成")
            return True
            
        except Exception as e:
            print(f"   ❌ 抓取序列失败: {e}")
            return False
    
    def _execute_lift_sequence(self, mobile_base) -> bool:
        """执行提升序列"""
        try:
            print("   ⬆️ 提升物体...")
            
            # 缓慢提升机械臂
            lift_poses = ["carry", "stow"]
            
            for pose_name in lift_poses:
                success = self._move_arm_to_pose(mobile_base, pose_name)
                if not success:
                    return False
                
                # 检查是否仍然抓着物体
                if not self.gripper_controller.object_grasped:
                    print("   ❌ 提升过程中物体掉落")
                    return False
                
                time.sleep(0.5)  # 稳定时间
            
            print("   ✅ 提升完成")
            return True
            
        except Exception as e:
            print(f"   ❌ 提升失败: {e}")
            return False
    
    def _execute_transport_sequence(self, mobile_base, drop_location: np.ndarray) -> bool:
        """执行运输序列"""
        try:
            print("   🚚 运输到目标位置...")
            
            # 这里可以集成导航系统
            # 简化版本：直接认为已经到达目标位置
            transport_time = 2.0
            
            for i in range(int(transport_time * 10)):
                # 监控抓取状态
                if not self.gripper_controller.object_grasped:
                    print("   ❌ 运输过程中物体掉落")
                    return False
                
                time.sleep(0.1)
            
            print("   ✅ 运输完成")
            return True
            
        except Exception as e:
            print(f"   ❌ 运输失败: {e}")
            return False
    
    def _execute_place_sequence(self, mobile_base, target_object, drop_location: np.ndarray) -> bool:
        """执行放置序列"""
        try:
            print("   📦 放置物体...")
            
            # 移动到放置姿态
            success = self._move_to_place_pose(mobile_base, drop_location)
            if not success:
                return False
            
            # 释放物体
            success = self.gripper_controller.release_object(mobile_base)
            if not success:
                return False
            
            # 移动物体到最终位置（模拟）
            final_position = drop_location.copy()
            final_position[2] = -1.0  # 地下位置表示已收集
            target_object.set_world_pose(final_position, target_object.get_world_pose()[1])
            
            # 回到安全姿态
            self._move_arm_to_pose(mobile_base, "home")
            
            print("   ✅ 放置完成")
            return True
            
        except Exception as e:
            print(f"   ❌ 放置失败: {e}")
            return False
    
    def _move_to_pre_grasp_pose(self, mobile_base) -> bool:
        """移动到预抓取姿态"""
        return self._move_arm_to_pose(mobile_base, "ready")
    
    def _move_to_place_pose(self, mobile_base, drop_location: np.ndarray) -> bool:
        """移动到放置姿态"""
        return self._move_arm_to_pose(mobile_base, "pickup")
    
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
                for _ in range(30):
                    mobile_base._world.step(render=True) if hasattr(mobile_base, '_world') else None
                    time.sleep(0.016)
                
                return True
            
        except Exception as e:
            print(f"   ❌ 移动机械臂失败: {e}")
            return False
    
    def _handle_failure(self, reason: str) -> bool:
        """处理失败情况"""
        print(f"   ❌ {reason}")
        self.stats['failed_grasps'] += 1
        
        # 记录失败模式
        failure_pattern = {
            'reason': reason,
            'timestamp': time.time(),
            'cuda_used': CUDA_AVAILABLE
        }
        self.grasp_history.append(failure_pattern)
        
        return False
    
    def _record_success(self, target_object, execution_time: float):
        """记录成功抓取"""
        self.stats['successful_grasps'] += 1
        
        # 更新平均时间
        total_time = self.stats['avg_grasp_time'] * (self.stats['successful_grasps'] - 1)
        self.stats['avg_grasp_time'] = (total_time + execution_time) / self.stats['successful_grasps']
        
        # 记录成功模式
        success_pattern = {
            'object_name': target_object.name,
            'execution_time': execution_time,
            'timestamp': time.time(),
            'cuda_used': CUDA_AVAILABLE
        }
        self.grasp_history.append(success_pattern)
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """获取性能统计"""
        success_rate = 0
        if self.stats['total_attempts'] > 0:
            success_rate = self.stats['successful_grasps'] / self.stats['total_attempts'] * 100
        
        return {
            'success_rate': success_rate,
            'total_attempts': self.stats['total_attempts'],
            'successful_grasps': self.stats['successful_grasps'],
            'failed_grasps': self.stats['failed_grasps'],
            'avg_grasp_time': self.stats['avg_grasp_time'],
            'cuda_acceleration': self.stats['cuda_acceleration_used']
        }
    
    def print_performance_report(self):
        """打印性能报告"""
        stats = self.get_performance_stats()
        
        print(f"\n📊 高级抓取性能报告:")
        print(f"   成功率: {stats['success_rate']:.1f}%")
        print(f"   总尝试: {stats['total_attempts']}")
        print(f"   成功抓取: {stats['successful_grasps']}")
        print(f"   失败次数: {stats['failed_grasps']}")
        print(f"   平均用时: {stats['avg_grasp_time']:.1f}s")
        print(f"   CUDA加速: {'✅ 启用' if stats['cuda_acceleration'] else '❌ 未启用'}")

# 工厂函数
def create_advanced_pick_and_place_system(config: Dict[str, Any]) -> AdvancedPickAndPlaceStrategy:
    """创建高级抓取放下系统"""
    print("🏭 初始化高级抓取放下系统...")
    
    system = AdvancedPickAndPlaceStrategy(config)
    
    print("✅ 高级抓取放下系统初始化完成")
    print(f"   - CUDA加速: {'启用' if CUDA_AVAILABLE else '禁用'}")
    print(f"   - 力控制反馈: 启用")
    print(f"   - 自适应抓取: 启用")
    print(f"   - 性能监控: 启用")
    
    return system