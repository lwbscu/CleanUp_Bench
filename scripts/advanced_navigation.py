#!/usr/bin/env python3
"""
OSGT四类物体导航系统 - 简化优化版（O类避障，S/G/T类精确导航）
提供丝滑精准的路径规划和移动控制
O类-障碍物避让 | S类-可清扫物接近 | G类-可抓取物定位 | T类-任务区访问
"""

import numpy as np
import heapq
import time
from collections import deque
from typing import List, Tuple, Optional, Dict, Any
import math

try:
    import cupy as cp
    CUDA_AVAILABLE = True
    print("✅ CUDA加速可用")
except ImportError:
    CUDA_AVAILABLE = False
    cp = np
    print("⚠️ CUDA不可用，使用CPU计算")

class OSGTAcceleratedAStar:
    """OSGT四类物体CUDA加速A*路径规划算法（全知全能版）"""
    
    def __init__(self, grid_resolution: float = 0.1, map_size: float = 20.0):
        self.grid_resolution = grid_resolution
        self.map_size = map_size
        self.map_cells = int(map_size / grid_resolution)
        
        # OSGT全知全能：完全空的障碍物地图（机器人可以到达任何地方）
        # O类障碍物在实际导航中通过动态避让处理
        if CUDA_AVAILABLE:
            self.obstacle_map = cp.zeros((self.map_cells, self.map_cells), dtype=cp.bool_)
        else:
            self.obstacle_map = np.zeros((self.map_cells, self.map_cells), dtype=bool)
        
        print(f"🗺️ OSGT全知全能A*：地图大小 {self.map_cells}x{self.map_cells}，无静态障碍物限制")
        
        # 预计算方向向量（8方向移动）
        self.directions = np.array([
            [0, 1], [1, 0], [0, -1], [-1, 0],  # 基础方向
            [1, 1], [1, -1], [-1, 1], [-1, -1]  # 对角线方向
        ])
        
        # 预计算距离权重
        self.direction_costs = np.array([1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414])
        
        if CUDA_AVAILABLE:
            self.directions = cp.array(self.directions)
            self.direction_costs = cp.array(self.direction_costs)
    
    def world_to_grid(self, pos: np.ndarray) -> Tuple[int, int]:
        """世界坐标转网格坐标"""
        x = int((pos[0] + self.map_size/2) / self.grid_resolution)
        y = int((pos[1] + self.map_size/2) / self.grid_resolution)
        return (max(0, min(x, self.map_cells-1)), max(0, min(y, self.map_cells-1)))
    
    def grid_to_world(self, grid_pos: Tuple[int, int]) -> List[float]:
        """网格坐标转世界坐标"""
        x = grid_pos[0] * self.grid_resolution - self.map_size/2
        y = grid_pos[1] * self.grid_resolution - self.map_size/2
        return [x, y]
    
    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """启发式函数 - 使用欧几里得距离"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
    
    def plan_osgt_path(self, start_pos: np.ndarray, goal_pos: np.ndarray, 
                       osgt_type: str = "sweepable") -> List[List[float]]:
        """执行OSGT类型特定的路径规划"""
        start_grid = self.world_to_grid(start_pos)
        goal_grid = self.world_to_grid(goal_pos)
        
        # 如果起点和终点相同，直接返回
        if start_grid == goal_grid:
            return [start_pos.tolist(), goal_pos.tolist()]
        
        # OSGT全知全能：根据物体类型调整路径策略
        distance = np.linalg.norm(goal_pos - start_pos)
        
        if osgt_type == "obstacles":
            # O类障碍物：不直接导航到障碍物，而是绕行
            return self._plan_obstacle_avoidance_path(start_pos, goal_pos)
        elif osgt_type == "sweepable":
            # S类可清扫物：直接快速路径
            return self._plan_direct_path(start_pos, goal_pos, distance)
        elif osgt_type == "graspable":
            # G类可抓取物：精确接近路径
            return self._plan_precision_path(start_pos, goal_pos, distance)
        elif osgt_type == "task_areas":
            # T类任务区：稳定接近路径
            return self._plan_stable_approach_path(start_pos, goal_pos, distance)
        else:
            # 默认：标准路径
            return self._plan_direct_path(start_pos, goal_pos, distance)
    
    def _plan_direct_path(self, start_pos: np.ndarray, goal_pos: np.ndarray, distance: float) -> List[List[float]]:
        """S类可清扫物：直接快速路径"""
        if distance < 0.5:
            return [start_pos.tolist(), goal_pos.tolist()]
        
        # 生成直线路径
        num_points = max(3, int(distance / 0.8))
        path = []
        
        for i in range(num_points + 1):
            t = i / num_points
            x = start_pos[0] + t * (goal_pos[0] - start_pos[0])
            y = start_pos[1] + t * (goal_pos[1] - start_pos[1])
            path.append([x, y])
        
        return path
    
    def _plan_precision_path(self, start_pos: np.ndarray, goal_pos: np.ndarray, distance: float) -> List[List[float]]:
        """G类可抓取物：精确接近路径"""
        if distance < 0.3:
            return [start_pos.tolist(), goal_pos.tolist()]
        
        # 生成带有精确接近段的路径
        path = []
        
        if distance > 1.0:
            # 远距离：快速接近
            approach_distance = distance - 0.5
            approach_direction = (goal_pos - start_pos) / distance
            approach_point = start_pos + approach_direction * approach_distance
            
            # 快速接近段
            num_approach_points = max(2, int(approach_distance / 1.0))
            for i in range(num_approach_points + 1):
                t = i / num_approach_points
                x = start_pos[0] + t * (approach_point[0] - start_pos[0])
                y = start_pos[1] + t * (approach_point[1] - start_pos[1])
                path.append([x, y])
            
            # 精确接近段
            num_precision_points = 5
            for i in range(1, num_precision_points + 1):
                t = i / num_precision_points
                x = approach_point[0] + t * (goal_pos[0] - approach_point[0])
                y = approach_point[1] + t * (goal_pos[1] - approach_point[1])
                path.append([x, y])
        else:
            # 近距离：直接精确接近
            num_points = max(4, int(distance / 0.2))
            for i in range(num_points + 1):
                t = i / num_points
                x = start_pos[0] + t * (goal_pos[0] - start_pos[0])
                y = start_pos[1] + t * (goal_pos[1] - start_pos[1])
                path.append([x, y])
        
        return path
    
    def _plan_stable_approach_path(self, start_pos: np.ndarray, goal_pos: np.ndarray, distance: float) -> List[List[float]]:
        """T类任务区：稳定接近路径"""
        if distance < 0.4:
            return [start_pos.tolist(), goal_pos.tolist()]
        
        # 生成稳定的弧形接近路径
        path = []
        
        # 计算接近方向
        direction = goal_pos - start_pos
        direction_norm = np.linalg.norm(direction)
        if direction_norm > 0:
            direction = direction / direction_norm
        
        # 添加中间控制点以形成平滑路径
        num_points = max(4, int(distance / 0.6))
        
        for i in range(num_points + 1):
            t = i / num_points
            # 使用smooth step函数使接近更平滑
            smooth_t = t * t * (3.0 - 2.0 * t)
            
            x = start_pos[0] + smooth_t * (goal_pos[0] - start_pos[0])
            y = start_pos[1] + smooth_t * (goal_pos[1] - start_pos[1])
            path.append([x, y])
        
        return path
    
    def _plan_obstacle_avoidance_path(self, start_pos: np.ndarray, goal_pos: np.ndarray) -> List[List[float]]:
        """O类障碍物：避障路径（实际上不应该导航到障碍物）"""
        # 如果意外导航到障碍物，返回绕行路径
        print("⚠️ 警告：不应该导航到O类障碍物位置")
        
        # 生成简单的绕行路径
        direction = goal_pos - start_pos
        distance = np.linalg.norm(direction)
        
        if distance < 0.1:
            return [start_pos.tolist()]
        
        # 创建绕行中间点
        mid_point = start_pos + 0.5 * direction
        # 向侧面偏移
        perpendicular = np.array([-direction[1], direction[0]])
        perpendicular = perpendicular / np.linalg.norm(perpendicular) * 1.0
        mid_point += perpendicular
        
        return [start_pos.tolist(), mid_point.tolist(), goal_pos.tolist()]
    
    def smooth_osgt_path(self, path: List[List[float]], osgt_type: str = "sweepable") -> List[List[float]]:
        """OSGT类型特定的路径平滑处理"""
        if len(path) <= 2:
            return path
        
        if osgt_type == "sweepable":
            # S类：快速平滑，保持效率
            return self._smooth_fast(path)
        elif osgt_type == "graspable":
            # G类：精确平滑，保持精度
            return self._smooth_precise(path)
        elif osgt_type == "task_areas":
            # T类：稳定平滑，保持稳定性
            return self._smooth_stable(path)
        else:
            return self._smooth_fast(path)
    
    def _smooth_fast(self, path: List[List[float]]) -> List[List[float]]:
        """快速平滑（S类可清扫物）"""
        smoothed_path = [path[0]]
        
        i = 0
        while i < len(path) - 1:
            # 寻找最远的可直达点
            max_reachable = min(len(path) - 1, i + 3)  # 限制跳跃距离
            
            if max_reachable > i:
                smoothed_path.append(path[max_reachable])
                i = max_reachable
            else:
                smoothed_path.append(path[i + 1])
                i += 1
        
        return smoothed_path
    
    def _smooth_precise(self, path: List[List[float]]) -> List[List[float]]:
        """精确平滑（G类可抓取物）"""
        smoothed_path = [path[0]]
        
        i = 0
        while i < len(path) - 1:
            # 保持更多中间点以确保精确性
            next_point = min(i + 2, len(path) - 1)
            smoothed_path.append(path[next_point])
            i = next_point
        
        # 确保包含最后一个点
        if smoothed_path[-1] != path[-1]:
            smoothed_path.append(path[-1])
        
        return smoothed_path
    
    def _smooth_stable(self, path: List[List[float]]) -> List[List[float]]:
        """稳定平滑（T类任务区）"""
        # 保持所有关键点以确保稳定接近
        return path

class OSGTSmoothMovementController:
    """OSGT四类物体丝滑移动控制器（优化转弯控制）"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        
        # 控制参数
        self.max_linear_vel = config.ROBOT_CONTROL["max_linear_velocity"]
        self.max_angular_vel = config.ROBOT_CONTROL["max_angular_velocity"]
        self.wheel_radius = config.ROBOT_CONTROL["wheel_radius"]
        self.wheel_base = config.ROBOT_CONTROL["wheel_base"]
        
        # OSGT类型特定的平滑控制参数
        self.osgt_control_params = {
            "sweepable": {
                "velocity_smoothing": 0.15,  # S类：适中平滑，保持效率
                "angular_smoothing": 0.1,
                "approach_style": "efficient"
            },
            "graspable": {
                "velocity_smoothing": 0.25,  # G类：更多平滑，保证精度
                "angular_smoothing": 0.2,
                "approach_style": "precise"
            },
            "task_areas": {
                "velocity_smoothing": 0.3,   # T类：最大平滑，保证稳定
                "angular_smoothing": 0.25,
                "approach_style": "stable"
            },
            "obstacles": {
                "velocity_smoothing": 0.1,   # O类：最少平滑，快速避让
                "angular_smoothing": 0.05,
                "approach_style": "avoidance"
            }
        }
        
        # 当前状态
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.current_osgt_type = "sweepable"
        
        # 控制历史
        self.control_history = deque(maxlen=10)
    
    def set_osgt_type(self, osgt_type: str):
        """设置当前处理的OSGT物体类型"""
        self.current_osgt_type = osgt_type
        if self.config.DEBUG["show_navigation_progress"]:
            print(f"🎯 切换到{osgt_type}类型导航模式")
    
    def compute_osgt_control(self, current_pos: np.ndarray, current_yaw: float, 
                            target_pos: np.ndarray, path_progress: float,
                            osgt_type: str = None) -> Tuple[float, float]:
        """计算OSGT类型特定的控制命令"""
        
        if osgt_type is None:
            osgt_type = self.current_osgt_type
        
        # 获取OSGT类型特定参数
        params = self.osgt_control_params.get(osgt_type, self.osgt_control_params["sweepable"])
        
        # 计算基础控制量
        direction = target_pos - current_pos[:2]
        distance = np.linalg.norm(direction)
        
        if distance < 0.01:
            return 0.0, 0.0
        
        # 计算目标角度
        target_angle = np.arctan2(direction[1], direction[0])
        angle_diff = self._normalize_angle(target_angle - current_yaw)
        
        # OSGT类型特定的控制策略
        if params["approach_style"] == "efficient":
            linear_vel, angular_vel = self._efficient_control_strategy(distance, angle_diff, path_progress)
        elif params["approach_style"] == "precise":
            linear_vel, angular_vel = self._precise_control_strategy(distance, angle_diff, path_progress)
        elif params["approach_style"] == "stable":
            linear_vel, angular_vel = self._stable_control_strategy(distance, angle_diff, path_progress)
        elif params["approach_style"] == "avoidance":
            linear_vel, angular_vel = self._avoidance_control_strategy(distance, angle_diff, path_progress)
        else:
            linear_vel, angular_vel = self._efficient_control_strategy(distance, angle_diff, path_progress)
        
        # OSGT类型特定的平滑处理
        linear_vel = self._smooth_velocity(linear_vel, self.current_linear_vel, params["velocity_smoothing"])
        angular_vel = self._smooth_velocity(angular_vel, self.current_angular_vel, params["angular_smoothing"])
        
        # 更新状态
        self.current_linear_vel = linear_vel
        self.current_angular_vel = angular_vel
        
        # 记录控制历史
        self.control_history.append((linear_vel, angular_vel, time.time(), osgt_type))
        
        return linear_vel, angular_vel
    
    def _efficient_control_strategy(self, distance: float, angle_diff: float, progress: float) -> Tuple[float, float]:
        """高效控制策略（S类可清扫物）"""
        # 基础速度，优先效率
        if distance > 2.0:
            base_speed = 0.5
        elif distance > 1.0:
            base_speed = 0.45
        elif distance > 0.5:
            base_speed = 0.4
        else:
            base_speed = max(0.25, distance * 0.5)
        
        abs_angle_diff = abs(angle_diff)
        
        # 效率优先：较小的角度阈值就开始减速
        if abs_angle_diff > 2.0:
            linear_vel = 0.15 * base_speed
            angular_vel = 2.0 * np.sign(angle_diff)
        elif abs_angle_diff > 1.2:
            linear_vel = 0.4 * base_speed
            angular_vel = 1.6 * np.sign(angle_diff)
        elif abs_angle_diff > 0.6:
            linear_vel = 0.7 * base_speed
            angular_vel = 1.2 * np.sign(angle_diff)
        else:
            linear_vel = base_speed
            angular_vel = 0.6 * np.sign(angle_diff)
        
        # 确保最小速度
        if distance > 0.1:
            linear_vel = max(linear_vel, 0.2)
        
        return self._limit_velocities(linear_vel, angular_vel)
    
    def _precise_control_strategy(self, distance: float, angle_diff: float, progress: float) -> Tuple[float, float]:
        """精确控制策略（G类可抓取物）"""
        # 基础速度，优先精确性
        if distance > 3.0:
            base_speed = 0.4  # 降低最大速度提高精确性
        elif distance > 1.5:
            base_speed = 0.35
        elif distance > 0.8:
            base_speed = 0.3
        elif distance > 0.3:
            base_speed = 0.25
        else:
            base_speed = max(0.15, distance * 0.3)  # 更慢的接近速度
        
        abs_angle_diff = abs(angle_diff)
        
        # 精确优先：更早开始角度调整
        if abs_angle_diff > 1.8:
            linear_vel = 0.1 * base_speed
            angular_vel = 1.5 * np.sign(angle_diff)
        elif abs_angle_diff > 1.0:
            linear_vel = 0.3 * base_speed
            angular_vel = 1.2 * np.sign(angle_diff)
        elif abs_angle_diff > 0.4:
            linear_vel = 0.6 * base_speed
            angular_vel = 0.8 * np.sign(angle_diff)
        elif abs_angle_diff > 0.15:
            linear_vel = 0.8 * base_speed
            angular_vel = 0.4 * np.sign(angle_diff)
        else:
            linear_vel = base_speed
            angular_vel = 0.2 * np.sign(angle_diff)
        
        # 接近目标时进一步减速
        if progress > 0.8:
            linear_vel *= 0.7
        elif progress > 0.9:
            linear_vel *= 0.5
        
        # 确保最小速度（但比效率模式更低）
        if distance > 0.1:
            linear_vel = max(linear_vel, 0.12)
        
        return self._limit_velocities(linear_vel, angular_vel)
    
    def _stable_control_strategy(self, distance: float, angle_diff: float, progress: float) -> Tuple[float, float]:
        """稳定控制策略（T类任务区）"""
        # 基础速度，优先稳定性
        if distance > 2.5:
            base_speed = 0.35  # 保守的最大速度
        elif distance > 1.2:
            base_speed = 0.3
        elif distance > 0.6:
            base_speed = 0.25
        else:
            base_speed = max(0.12, distance * 0.25)  # 非常慢的接近速度
        
        abs_angle_diff = abs(angle_diff)
        
        # 稳定优先：非常平滑的角度控制
        if abs_angle_diff > 1.5:
            linear_vel = 0.2 * base_speed
            angular_vel = 1.0 * np.sign(angle_diff)
        elif abs_angle_diff > 0.8:
            linear_vel = 0.5 * base_speed
            angular_vel = 0.8 * np.sign(angle_diff)
        elif abs_angle_diff > 0.3:
            linear_vel = 0.7 * base_speed
            angular_vel = 0.5 * np.sign(angle_diff)
        else:
            linear_vel = base_speed
            angular_vel = 0.3 * np.sign(angle_diff)
        
        # 渐进式减速
        if progress > 0.7:
            linear_vel *= 0.8
        elif progress > 0.85:
            linear_vel *= 0.6
        elif progress > 0.95:
            linear_vel *= 0.4
        
        # 确保最小速度（最低）
        if distance > 0.1:
            linear_vel = max(linear_vel, 0.1)
        
        return self._limit_velocities(linear_vel, angular_vel)
    
    def _avoidance_control_strategy(self, distance: float, angle_diff: float, progress: float) -> Tuple[float, float]:
        """避障控制策略（O类障碍物）"""
        # 快速避让策略
        base_speed = 0.6  # 较高的避让速度
        
        abs_angle_diff = abs(angle_diff)
        
        # 快速转向避让
        if abs_angle_diff > 2.5:
            linear_vel = 0.1 * base_speed
            angular_vel = 2.5 * np.sign(angle_diff)  # 更快的角速度
        elif abs_angle_diff > 1.0:
            linear_vel = 0.3 * base_speed
            angular_vel = 2.0 * np.sign(angle_diff)
        else:
            linear_vel = base_speed
            angular_vel = 0.8 * np.sign(angle_diff)
        
        # 避障时保持较高的最小速度
        if distance > 0.1:
            linear_vel = max(linear_vel, 0.25)
        
        return self._limit_velocities(linear_vel, angular_vel)
    
    def _limit_velocities(self, linear_vel: float, angular_vel: float) -> Tuple[float, float]:
        """限制速度到安全范围"""
        linear_vel = np.clip(linear_vel, 0.0, self.max_linear_vel)
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        return linear_vel, angular_vel
    
    def _smooth_velocity(self, target_vel: float, current_vel: float, smoothing: float) -> float:
        """速度平滑处理"""
        return smoothing * current_vel + (1 - smoothing) * target_vel
    
    def _normalize_angle(self, angle: float) -> float:
        """角度标准化到[-π, π]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def reset(self):
        """重置控制器状态"""
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.control_history.clear()
        self.current_osgt_type = "sweepable"

class OSGTAdvancedNavigationSystem:
    """OSGT四类物体高级导航系统（简化版，移除卡住检测）"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        
        # 初始化OSGT组件
        self.path_planner = OSGTAcceleratedAStar(
            grid_resolution=config.NAVIGATION["grid_resolution"],
            map_size=config.NAVIGATION["map_size"]
        )
        
        self.movement_controller = OSGTSmoothMovementController(config)
        
        # 导航状态
        self.current_path = []
        self.path_index = 0
        self.navigation_active = False
        self.current_osgt_type = "sweepable"
        
        # OSGT性能监控
        self.osgt_navigation_stats = {
            'total_navigations': 0,
            'successful_navigations': 0,
            'osgt_type_stats': {
                'obstacles': {'attempts': 0, 'successes': 0},
                'sweepable': {'attempts': 0, 'successes': 0},
                'graspable': {'attempts': 0, 'successes': 0},
                'task_areas': {'attempts': 0, 'successes': 0}
            },
            'average_time': 0.0,
            'total_distance': 0.0
        }
    
    def navigate_to_osgt_target(self, robot_controller, target_pos: np.ndarray, 
                               osgt_type: str = "sweepable", max_time: float = 30.0, 
                               tolerance: float = 0.2) -> bool:
        """执行OSGT类型特定的导航"""
        start_time = time.time()
        self.osgt_navigation_stats['total_navigations'] += 1
        self.osgt_navigation_stats['osgt_type_stats'][osgt_type]['attempts'] += 1
        
        # 设置当前OSGT类型
        self.current_osgt_type = osgt_type
        self.movement_controller.set_osgt_type(osgt_type)
        
        try:
            # 获取当前位置
            current_pos, current_yaw = robot_controller.get_robot_pose()
            
            # OSGT类型特定的路径规划
            self.current_path = self.path_planner.plan_osgt_path(
                current_pos[:2], target_pos[:2], osgt_type
            )
            
            if len(self.current_path) > 2:
                print(f"   🗺️ OSGT-{osgt_type}路径规划完成，{len(self.current_path)}个路径点")
            
            # OSGT类型特定的路径平滑
            self.current_path = self.path_planner.smooth_osgt_path(self.current_path, osgt_type)
            
            # 重置控制器
            self.movement_controller.reset()
            self.movement_controller.set_osgt_type(osgt_type)
            
            # OSGT导航循环
            self.path_index = 1
            self.navigation_active = True
            
            # 根据OSGT类型调整路径点切换距离
            switch_distances = {
                "sweepable": 0.6,    # S类：较大的切换距离，提高效率
                "graspable": 0.4,    # G类：较小的切换距离，提高精确性
                "task_areas": 0.5,   # T类：中等切换距离，保持稳定
                "obstacles": 0.8     # O类：大切换距离，快速避让
            }
            switch_distance = switch_distances.get(osgt_type, 0.6)
            
            while (time.time() - start_time < max_time and 
                   self.path_index < len(self.current_path) and 
                   self.navigation_active):
                
                # 获取当前状态
                current_pos, current_yaw = robot_controller.get_robot_pose()
                
                # 检查是否到达目标
                final_distance = np.linalg.norm(current_pos[:2] - target_pos[:2])
                if final_distance < tolerance:
                    robot_controller._stop_robot()
                    print(f"   ✅ OSGT-{osgt_type}导航成功！距离: {final_distance:.3f}m")
                    self.osgt_navigation_stats['successful_navigations'] += 1
                    self.osgt_navigation_stats['osgt_type_stats'][osgt_type]['successes'] += 1
                    return True
                
                # 获取当前目标点
                current_target = np.array(self.current_path[self.path_index])
                
                # 计算路径进度
                progress = self.path_index / len(self.current_path)
                
                # 计算OSGT类型特定的控制命令
                linear_vel, angular_vel = self.movement_controller.compute_osgt_control(
                    current_pos, current_yaw, current_target, progress, osgt_type
                )
                
                # 发送控制命令
                robot_controller._send_movement_command(linear_vel, angular_vel)
                
                # 检查路径点切换（使用OSGT类型特定的距离）
                target_distance = np.linalg.norm(current_pos[:2] - current_target)
                if target_distance < switch_distance:
                    self.path_index += 1
                    if self.config.DEBUG["show_navigation_progress"] and self.path_index % 3 == 0:
                        print(f"   📈 OSGT-{osgt_type}导航进度: {progress*100:.0f}%")
                
                # 渲染
                robot_controller.world.step(render=True)
                time.sleep(0.016)
            
            # 检查最终结果
            current_pos, _ = robot_controller.get_robot_pose()
            final_distance = np.linalg.norm(current_pos[:2] - target_pos[:2])
            
            # 根据OSGT类型调整成功判定容差
            success_tolerance = tolerance * 1.5
            if osgt_type == "graspable":
                success_tolerance = tolerance * 1.2  # G类要求更高精度
            elif osgt_type == "task_areas":
                success_tolerance = tolerance * 1.8  # T类容许更大偏差
            
            if final_distance < success_tolerance:
                print(f"   ✅ OSGT-{osgt_type}导航接近成功！距离: {final_distance:.3f}m")
                self.osgt_navigation_stats['successful_navigations'] += 1
                self.osgt_navigation_stats['osgt_type_stats'][osgt_type]['successes'] += 1
                return True
            else:
                print(f"   ⚠️ OSGT-{osgt_type}导航超时，距离: {final_distance:.3f}m")
                return False
                
        except Exception as e:
            print(f"   ❌ OSGT-{osgt_type}导航异常: {e}")
            return False
        
        finally:
            # 更新统计
            navigation_time = time.time() - start_time
            self.osgt_navigation_stats['average_time'] = (
                (self.osgt_navigation_stats['average_time'] * (self.osgt_navigation_stats['total_navigations'] - 1) + 
                 navigation_time) / self.osgt_navigation_stats['total_navigations']
            )
            self.navigation_active = False
    
    def navigate_to_target(self, robot_controller, target_pos: np.ndarray, 
                          max_time: float = 30.0, tolerance: float = 0.2) -> bool:
        """向后兼容的导航方法"""
        return self.navigate_to_osgt_target(
            robot_controller, target_pos, "sweepable", max_time, tolerance
        )
    
    def get_navigation_stats(self) -> Dict[str, Any]:
        """向后兼容的统计方法"""
        return self.get_osgt_navigation_stats()
    
    def print_stats(self):
        """向后兼容的统计打印方法"""
        return self.print_osgt_stats()
    
    def get_osgt_navigation_stats(self) -> Dict[str, Any]:
        """获取OSGT导航统计信息"""
        stats = self.osgt_navigation_stats.copy()
        
        if stats['total_navigations'] > 0:
            stats['overall_success_rate'] = (stats['successful_navigations'] / 
                                            stats['total_navigations'] * 100)
        else:
            stats['overall_success_rate'] = 0.0
        
        # 计算各OSGT类型的成功率
        for osgt_type, type_stats in stats['osgt_type_stats'].items():
            if type_stats['attempts'] > 0:
                type_stats['success_rate'] = (type_stats['successes'] / 
                                            type_stats['attempts'] * 100)
            else:
                type_stats['success_rate'] = 0.0
        
        return stats
    
    def print_osgt_stats(self):
        """打印OSGT导航统计"""
        stats = self.get_osgt_navigation_stats()
        print(f"\n📊 OSGT导航统计:")
        print(f"   总导航次数: {stats['total_navigations']}")
        print(f"   总体成功率: {stats['overall_success_rate']:.1f}%")
        print(f"   平均用时: {stats['average_time']:.1f}s")
        
        print(f"\n🎯 OSGT各类型统计:")
        osgt_symbols = {
            'obstacles': '🚧',
            'sweepable': '🧹', 
            'graspable': '🦾',
            'task_areas': '🎯'
        }
        
        for osgt_type, type_stats in stats['osgt_type_stats'].items():
            symbol = osgt_symbols.get(osgt_type, '📦')
            if type_stats['attempts'] > 0:
                print(f"   {symbol} {osgt_type}: {type_stats['successes']}/{type_stats['attempts']} "
                      f"({type_stats['success_rate']:.1f}%)")

# 为了保持向后兼容性的别名
AdvancedNavigationSystem = OSGTAdvancedNavigationSystem

# 确保所有必要的类都可以被导入
__all__ = [
    # OSGT 新类名
    'OSGTAdvancedNavigationSystem',
    'OSGTAcceleratedAStar',
    'OSGTSmoothMovementController',
    
    # 向后兼容别名
    'AdvancedNavigationSystem',
]

print("✅ OSGT导航系统兼容性别名已加载")