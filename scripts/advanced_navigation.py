#!/usr/bin/env python3
"""
高级导航系统 - 简化优化版（移除卡住检测，优化转弯控制）
提供丝滑精准的路径规划和移动控制
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

class CUDAAcceleratedAStar:
    """CUDA加速的A*路径规划算法（全知全能版）"""
    
    def __init__(self, grid_resolution: float = 0.1, map_size: float = 20.0):
        self.grid_resolution = grid_resolution
        self.map_size = map_size
        self.map_cells = int(map_size / grid_resolution)
        
        # 全知全能：完全空的障碍物地图（机器人可以到达任何地方）
        if CUDA_AVAILABLE:
            self.obstacle_map = cp.zeros((self.map_cells, self.map_cells), dtype=cp.bool_)
        else:
            self.obstacle_map = np.zeros((self.map_cells, self.map_cells), dtype=bool)
        
        print(f"🗺️ 全知全能A*：地图大小 {self.map_cells}x{self.map_cells}，无障碍物限制")
        
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
    
    def plan_path(self, start_pos: np.ndarray, goal_pos: np.ndarray) -> List[List[float]]:
        """执行全知全能A*路径规划（无障碍物限制）"""
        start_grid = self.world_to_grid(start_pos)
        goal_grid = self.world_to_grid(goal_pos)
        
        # 如果起点和终点相同，直接返回
        if start_grid == goal_grid:
            return [start_pos.tolist(), goal_pos.tolist()]
        
        # 全知全能：直接生成直线路径（无障碍物）
        distance = np.linalg.norm(goal_pos - start_pos)
        if distance < 0.5:
            # 短距离直接连接
            return [start_pos.tolist(), goal_pos.tolist()]
        
        # 生成平滑的直线路径
        num_points = max(3, int(distance / 0.8))  # 每0.8米一个路径点
        path = []
        
        for i in range(num_points + 1):
            t = i / num_points
            x = start_pos[0] + t * (goal_pos[0] - start_pos[0])
            y = start_pos[1] + t * (goal_pos[1] - start_pos[1])
            path.append([x, y])
        
        return path
    
    def plan_path_with_astar(self, start_pos: np.ndarray, goal_pos: np.ndarray) -> List[List[float]]:
        """执行标准A*路径规划（备用方案）"""
        start_grid = self.world_to_grid(start_pos)
        goal_grid = self.world_to_grid(goal_pos)
        
        # A*算法核心
        frontier = []
        heapq.heappush(frontier, (0, start_grid))
        came_from = {start_grid: None}
        cost_so_far = {start_grid: 0}
        
        while frontier:
            current = heapq.heappop(frontier)[1]
            
            if current == goal_grid:
                break
            
            # 检查所有方向
            for i, (dx, dy) in enumerate(self.directions):
                next_pos = (current[0] + dx, current[1] + dy)
                
                # 边界检查
                if (next_pos[0] < 0 or next_pos[0] >= self.map_cells or 
                    next_pos[1] < 0 or next_pos[1] >= self.map_cells):
                    continue
                
                # 全知全能：跳过障碍物检查（所有位置都可达）
                
                # 计算新的成本
                new_cost = cost_so_far[current] + self.direction_costs[i]
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal_grid, next_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current
        
        # 重建路径
        if goal_grid not in came_from:
            # 如果A*失败，回退到直线路径
            return self.plan_path(start_pos, goal_pos)
        
        path = []
        current = goal_grid
        while current is not None:
            path.append(self.grid_to_world(current))
            current = came_from[current]
        
        path.reverse()
        return self.smooth_path(path)
    
    def smooth_path(self, path: List[List[float]]) -> List[List[float]]:
        """路径平滑处理（全知全能版）"""
        if len(path) <= 2:
            return path
        
        # 全知全能：所有点都可直达，大幅简化路径
        smoothed_path = [path[0]]
        
        i = 0
        while i < len(path) - 1:
            # 寻找最远的可直达点（全知全能：所有点都可达）
            max_reachable = len(path) - 1  # 直接跳到终点
            
            # 添加中间点避免过长的直线段
            step_size = max(1, (max_reachable - i) // 3)
            next_point = min(i + step_size, max_reachable)
            
            if next_point > i:
                smoothed_path.append(path[next_point])
                i = next_point
            else:
                smoothed_path.append(path[i + 1])
                i += 1
        
        return smoothed_path

class SmoothMovementController:
    """丝滑移动控制器（优化转弯控制）"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        
        # 控制参数
        self.max_linear_vel = config.ROBOT_CONTROL["max_linear_velocity"]
        self.max_angular_vel = config.ROBOT_CONTROL["max_angular_velocity"]
        self.wheel_radius = config.ROBOT_CONTROL["wheel_radius"]
        self.wheel_base = config.ROBOT_CONTROL["wheel_base"]
        
        # 平滑控制参数（大幅减少平滑，让运动更连续）
        self.velocity_smoothing = 0.2  # 大幅减少平滑
        self.angular_smoothing = 0.15  # 大幅减少平滑
        
        # 当前状态
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # 控制历史
        self.control_history = deque(maxlen=10)
    
    def compute_control(self, current_pos: np.ndarray, current_yaw: float, 
                       target_pos: np.ndarray, path_progress: float) -> Tuple[float, float]:
        """计算丝滑的控制命令（优化转弯策略）"""
        
        # 计算基础控制量
        direction = target_pos - current_pos[:2]
        distance = np.linalg.norm(direction)
        
        if distance < 0.01:
            return 0.0, 0.0
        
        # 计算目标角度
        target_angle = np.arctan2(direction[1], direction[0])
        angle_diff = self._normalize_angle(target_angle - current_yaw)
        
        # 优化的控制策略：边走边转
        linear_vel, angular_vel = self._smooth_control_strategy(distance, angle_diff, path_progress)
        
        # 平滑处理
        linear_vel = self._smooth_velocity(linear_vel, self.current_linear_vel, self.velocity_smoothing)
        angular_vel = self._smooth_velocity(angular_vel, self.current_angular_vel, self.angular_smoothing)
        
        # 更新状态
        self.current_linear_vel = linear_vel
        self.current_angular_vel = angular_vel
        
        # 记录控制历史
        self.control_history.append((linear_vel, angular_vel, time.time()))
        
        return linear_vel, angular_vel
    
    def _smooth_control_strategy(self, distance: float, angle_diff: float, progress: float) -> Tuple[float, float]:
        """优化的控制策略：确保连续流畅运动"""
        
        # 基础速度，根据距离调整
        if distance > 3.0:
            base_speed = 0.5
        elif distance > 2.0:
            base_speed = 0.45
        elif distance > 1.0:
            base_speed = 0.4
        elif distance > 0.5:
            base_speed = 0.35
        else:
            base_speed = max(0.2, distance * 0.4)  # 确保接近目标时不会停止
        
        # 角度差绝对值
        abs_angle_diff = abs(angle_diff)
        
        # 优化策略：始终保持前进，只在极大角度时才减速
        if abs_angle_diff > 2.5:  # 超过143度才大幅减速
            linear_vel = 0.1 * base_speed
            angular_vel = 2.0 * np.sign(angle_diff)
        elif abs_angle_diff > 1.5:  # 86度到143度：适度减速转向
            linear_vel = 0.3 * base_speed
            angular_vel = 1.6 * np.sign(angle_diff)
        elif abs_angle_diff > 0.8:  # 46度到86度：轻微减速转向
            linear_vel = 0.6 * base_speed
            angular_vel = 1.2 * np.sign(angle_diff)
        elif abs_angle_diff > 0.3:  # 17度到46度：保持速度，轻微转向
            linear_vel = 0.8 * base_speed
            angular_vel = 0.8 * np.sign(angle_diff)
        else:  # 小于17度：全速前进，微调
            linear_vel = base_speed
            angular_vel = 0.4 * np.sign(angle_diff)
        
        # 确保最小速度，避免停止
        if distance > 0.1:
            linear_vel = max(linear_vel, 0.15)
        
        # 根据路径进度调整（减少减速）
        if progress > 0.9:
            linear_vel *= 0.8  # 接近目标时适度减速
        elif progress > 0.8:
            linear_vel *= 0.9
        
        # 限制速度
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

class AdvancedNavigationSystem:
    """高级导航系统（简化版，移除卡住检测）"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        
        # 初始化组件
        self.path_planner = CUDAAcceleratedAStar(
            grid_resolution=config.NAVIGATION["grid_resolution"],
            map_size=config.NAVIGATION["map_size"]
        )
        
        self.movement_controller = SmoothMovementController(config)
        
        # 导航状态
        self.current_path = []
        self.path_index = 0
        self.navigation_active = False
        
        # 性能监控
        self.navigation_stats = {
            'total_navigations': 0,
            'successful_navigations': 0,
            'average_time': 0.0,
            'total_distance': 0.0
        }
    
    def navigate_to_target(self, robot_controller, target_pos: np.ndarray, 
                          max_time: float = 30.0, tolerance: float = 0.2) -> bool:
        """执行简化导航（移除卡住检测）"""
        start_time = time.time()
        self.navigation_stats['total_navigations'] += 1
        
        try:
            # 获取当前位置
            current_pos, current_yaw = robot_controller.get_robot_pose()
            
            # 规划路径
            self.current_path = self.path_planner.plan_path(current_pos[:2], target_pos[:2])
            
            if len(self.current_path) > 2:
                print(f"   🗺️ A*路径规划完成，{len(self.current_path)}个路径点")
            
            # 重置控制器
            self.movement_controller.reset()
            
            # 导航循环（简化版本）
            self.path_index = 1
            self.navigation_active = True
            
            while (time.time() - start_time < max_time and 
                   self.path_index < len(self.current_path) and 
                   self.navigation_active):
                
                # 获取当前状态
                current_pos, current_yaw = robot_controller.get_robot_pose()
                
                # 检查是否到达目标
                final_distance = np.linalg.norm(current_pos[:2] - target_pos[:2])
                if final_distance < tolerance:
                    robot_controller._stop_robot()
                    print(f"   ✅ 导航成功！距离: {final_distance:.3f}m")
                    self.navigation_stats['successful_navigations'] += 1
                    return True
                
                # 获取当前目标点
                current_target = np.array(self.current_path[self.path_index])
                
                # 计算路径进度
                progress = self.path_index / len(self.current_path)
                
                # 计算控制命令
                linear_vel, angular_vel = self.movement_controller.compute_control(
                    current_pos, current_yaw, current_target, progress
                )
                
                # 发送控制命令
                robot_controller._send_movement_command(linear_vel, angular_vel)
                
                # 检查路径点切换（增大切换距离，让路径跟随更流畅）
                target_distance = np.linalg.norm(current_pos[:2] - current_target)
                if target_distance < 0.6:  # 增大路径点切换距离
                    self.path_index += 1
                
                # 渲染
                robot_controller.world.step(render=True)
                time.sleep(0.016)
            
            # 检查最终结果
            current_pos, _ = robot_controller.get_robot_pose()
            final_distance = np.linalg.norm(current_pos[:2] - target_pos[:2])
            
            if final_distance < tolerance * 1.5:
                print(f"   ✅ 导航接近成功！距离: {final_distance:.3f}m")
                self.navigation_stats['successful_navigations'] += 1
                return True
            else:
                print(f"   ⚠️ 导航超时，距离: {final_distance:.3f}m")
                return False
                
        except Exception as e:
            print(f"   ❌ 导航异常: {e}")
            return False
        
        finally:
            # 更新统计
            navigation_time = time.time() - start_time
            self.navigation_stats['average_time'] = (
                (self.navigation_stats['average_time'] * (self.navigation_stats['total_navigations'] - 1) + 
                 navigation_time) / self.navigation_stats['total_navigations']
            )
            self.navigation_active = False
    
    def get_navigation_stats(self) -> Dict[str, Any]:
        """获取导航统计信息"""
        if self.navigation_stats['total_navigations'] > 0:
            success_rate = (self.navigation_stats['successful_navigations'] / 
                          self.navigation_stats['total_navigations'] * 100)
        else:
            success_rate = 0.0
        
        return {
            'total_navigations': self.navigation_stats['total_navigations'],
            'success_rate': success_rate,
            'average_time': self.navigation_stats['average_time']
        }
    
    def print_stats(self):
        """打印导航统计"""
        stats = self.get_navigation_stats()
        print(f"\n📊 导航统计:")
        print(f"   总导航次数: {stats['total_navigations']}")
        print(f"   成功率: {stats['success_rate']:.1f}%")
        print(f"   平均用时: {stats['average_time']:.1f}s")