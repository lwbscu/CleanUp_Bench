#!/usr/bin/env python3
"""
高级导航系统 - CUDA加速优化版
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
    """CUDA加速的A*路径规划算法"""
    
    def __init__(self, grid_resolution: float = 0.1, map_size: float = 20.0):
        self.grid_resolution = grid_resolution
        self.map_size = map_size
        self.map_cells = int(map_size / grid_resolution)
        
        # 初始化障碍物地图
        if CUDA_AVAILABLE:
            self.obstacle_map = cp.zeros((self.map_cells, self.map_cells), dtype=cp.bool_)
        else:
            self.obstacle_map = np.zeros((self.map_cells, self.map_cells), dtype=bool)
        
        # 预计算方向向量
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
        """执行A*路径规划"""
        start_grid = self.world_to_grid(start_pos)
        goal_grid = self.world_to_grid(goal_pos)
        
        # 如果起点和终点相同，直接返回
        if start_grid == goal_grid:
            return [start_pos.tolist(), goal_pos.tolist()]
        
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
                
                # 障碍物检查
                if CUDA_AVAILABLE:
                    if self.obstacle_map[next_pos[0], next_pos[1]]:
                        continue
                else:
                    if self.obstacle_map[next_pos[0], next_pos[1]]:
                        continue
                
                # 计算新的成本
                new_cost = cost_so_far[current] + self.direction_costs[i]
                
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.heuristic(goal_grid, next_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current
        
        # 重建路径
        if goal_grid not in came_from:
            return [start_pos.tolist(), goal_pos.tolist()]
        
        path = []
        current = goal_grid
        while current is not None:
            path.append(self.grid_to_world(current))
            current = came_from[current]
        
        path.reverse()
        return self.smooth_path(path)
    
    def smooth_path(self, path: List[List[float]]) -> List[List[float]]:
        """路径平滑处理"""
        if len(path) <= 2:
            return path
        
        smoothed_path = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # 寻找最远的可直达点
            max_reachable = i + 1
            for j in range(i + 2, len(path)):
                if self._is_line_clear(path[i], path[j]):
                    max_reachable = j
                else:
                    break
            
            # 添加路径点
            if max_reachable > i + 1:
                smoothed_path.append(path[max_reachable])
                i = max_reachable
            else:
                smoothed_path.append(path[i + 1])
                i += 1
        
        return smoothed_path
    
    def _is_line_clear(self, start: List[float], end: List[float]) -> bool:
        """检查两点间的直线是否无障碍"""
        start_grid = self.world_to_grid(np.array(start))
        end_grid = self.world_to_grid(np.array(end))
        
        # 使用Bresenham算法检查直线路径
        x0, y0 = start_grid
        x1, y1 = end_grid
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            # 检查当前点是否为障碍物
            if (x < 0 or x >= self.map_cells or y < 0 or y >= self.map_cells or 
                self.obstacle_map[x, y]):
                return False
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return True

class SmoothMovementController:
    """丝滑移动控制器"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        
        # 控制参数
        self.max_linear_vel = config.ROBOT_CONTROL["max_linear_velocity"]
        self.max_angular_vel = config.ROBOT_CONTROL["max_angular_velocity"]
        self.wheel_radius = config.ROBOT_CONTROL["wheel_radius"]
        self.wheel_base = config.ROBOT_CONTROL["wheel_base"]
        
        # 平滑控制参数
        self.velocity_smoothing = 0.85  # 更积极的平滑
        self.angular_smoothing = 0.8
        
        # 当前状态
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # 预测控制参数
        self.prediction_steps = 3
        self.control_history = deque(maxlen=10)
        
        # 自适应参数
        self.adaptive_gains = {
            'distance_factor': 0.8,
            'angle_factor': 1.2,
            'speed_factor': 1.0
        }
    
    def compute_control(self, current_pos: np.ndarray, current_yaw: float, 
                       target_pos: np.ndarray, path_progress: float) -> Tuple[float, float]:
        """计算丝滑的控制命令"""
        
        # 计算基础控制量
        direction = target_pos - current_pos[:2]
        distance = np.linalg.norm(direction)
        
        if distance < 0.01:
            return 0.0, 0.0
        
        # 计算目标角度
        target_angle = np.arctan2(direction[1], direction[0])
        angle_diff = self._normalize_angle(target_angle - current_yaw)
        
        # 自适应控制策略
        linear_vel, angular_vel = self._adaptive_control(distance, angle_diff, path_progress)
        
        # 平滑处理
        linear_vel = self._smooth_velocity(linear_vel, self.current_linear_vel, self.velocity_smoothing)
        angular_vel = self._smooth_velocity(angular_vel, self.current_angular_vel, self.angular_smoothing)
        
        # 更新状态
        self.current_linear_vel = linear_vel
        self.current_angular_vel = angular_vel
        
        # 记录控制历史
        self.control_history.append((linear_vel, angular_vel, time.time()))
        
        return linear_vel, angular_vel
    
    def _adaptive_control(self, distance: float, angle_diff: float, progress: float) -> Tuple[float, float]:
        """自适应控制策略"""
        
        # 距离相关的速度调整
        if distance > 2.0:
            distance_factor = 1.0
        elif distance > 1.0:
            distance_factor = 0.8
        elif distance > 0.5:
            distance_factor = 0.6
        else:
            distance_factor = 0.4
        
        # 角度相关的控制
        abs_angle_diff = abs(angle_diff)
        
        if abs_angle_diff > np.pi/2:
            # 大角度转向：先转向再前进
            linear_vel = 0.1 * distance_factor
            angular_vel = 2.0 * np.sign(angle_diff)
        elif abs_angle_diff > np.pi/4:
            # 中等角度：转向为主，适当前进
            linear_vel = 0.3 * distance_factor
            angular_vel = 1.5 * np.sign(angle_diff)
        elif abs_angle_diff > np.pi/8:
            # 小角度：前进为主，微调转向
            linear_vel = 0.6 * distance_factor
            angular_vel = 1.0 * angle_diff
        else:
            # 直行：全速前进
            linear_vel = min(0.8 * distance_factor, distance * 0.5)
            angular_vel = 0.5 * angle_diff
        
        # 根据路径进度调整
        if progress > 0.8:
            linear_vel *= 0.7  # 接近目标时减速
        
        # 限制速度
        linear_vel = np.clip(linear_vel, -self.max_linear_vel, self.max_linear_vel)
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

class IntelligentStuckDetector:
    """智能卡住检测器"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        
        # 检测参数
        self.stuck_threshold = 0.02  # 更严格的卡住阈值
        self.stuck_time_threshold = 3.0  # 卡住时间阈值
        self.detection_window = 60  # 检测窗口
        
        # 状态跟踪
        self.position_history = deque(maxlen=self.detection_window)
        self.last_significant_move = time.time()
        self.stuck_count = 0
        
        # 多层检测
        self.velocity_history = deque(maxlen=30)
        self.angular_history = deque(maxlen=30)
    
    def update(self, position: np.ndarray, velocity: float, angular_velocity: float) -> bool:
        """更新检测状态，返回是否卡住"""
        current_time = time.time()
        
        # 记录历史
        self.position_history.append((position.copy(), current_time))
        self.velocity_history.append(velocity)
        self.angular_history.append(angular_velocity)
        
        # 多层检测
        stuck_detected = False
        
        # 位置检测
        if len(self.position_history) >= 30:
            recent_movement = self._calculate_recent_movement()
            if recent_movement < self.stuck_threshold:
                stuck_detected = True
        
        # 速度检测
        if len(self.velocity_history) >= 20:
            avg_velocity = np.mean(list(self.velocity_history)[-20:])
            if abs(avg_velocity) < 0.05:
                stuck_detected = True
        
        # 振荡检测
        if self._detect_oscillation():
            stuck_detected = True
        
        # 更新状态
        if not stuck_detected:
            self.last_significant_move = current_time
            self.stuck_count = 0
        else:
            time_since_move = current_time - self.last_significant_move
            if time_since_move > self.stuck_time_threshold:
                self.stuck_count += 1
                return True
        
        return False
    
    def _calculate_recent_movement(self) -> float:
        """计算最近的移动距离"""
        if len(self.position_history) < 2:
            return 0.0
        
        positions = [pos for pos, _ in self.position_history]
        recent_positions = positions[-20:]
        
        if len(recent_positions) < 2:
            return 0.0
        
        # 计算移动距离
        total_distance = 0.0
        for i in range(1, len(recent_positions)):
            distance = np.linalg.norm(recent_positions[i][:2] - recent_positions[i-1][:2])
            total_distance += distance
        
        return total_distance
    
    def _detect_oscillation(self) -> bool:
        """检测振荡行为"""
        if len(self.angular_history) < 20:
            return False
        
        recent_angular = list(self.angular_history)[-20:]
        
        # 检查角速度是否频繁变号
        sign_changes = 0
        for i in range(1, len(recent_angular)):
            if recent_angular[i] * recent_angular[i-1] < 0:
                sign_changes += 1
        
        return sign_changes > 8  # 频繁变号表示振荡
    
    def reset(self):
        """重置检测器"""
        self.position_history.clear()
        self.velocity_history.clear()
        self.angular_history.clear()
        self.last_significant_move = time.time()
        self.stuck_count = 0

class AdvancedBreakthroughStrategy:
    """高级突破策略"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.breakthrough_count = 0
        self.last_breakthrough_time = 0
        
        # 策略参数
        self.strategies = [
            self._strategy_reverse_and_turn,
            self._strategy_multiple_turns,
            self._strategy_random_walk,
            self._strategy_spiral_escape
        ]
    
    def execute_breakthrough(self, robot_controller, stuck_count: int) -> bool:
        """执行突破策略"""
        current_time = time.time()
        
        # 防止过于频繁的突破
        if current_time - self.last_breakthrough_time < 5.0:
            return False
        
        self.last_breakthrough_time = current_time
        self.breakthrough_count += 1
        
        # 选择策略
        strategy_index = min(stuck_count - 1, len(self.strategies) - 1)
        strategy = self.strategies[strategy_index]
        
        print(f"   💥 执行突破策略 {strategy_index + 1}: {strategy.__name__}")
        
        try:
            return strategy(robot_controller)
        except Exception as e:
            print(f"   ❌ 突破策略失败: {e}")
            return False
    
    def _strategy_reverse_and_turn(self, robot_controller) -> bool:
        """策略1：后退并转向"""
        # 后退
        for _ in range(30):
            robot_controller._send_movement_command(-0.3, 0.0)
            robot_controller.world.step(render=True)
            time.sleep(0.016)
        
        # 随机转向
        turn_direction = 1 if np.random.random() > 0.5 else -1
        for _ in range(40):
            robot_controller._send_movement_command(0.0, 2.0 * turn_direction)
            robot_controller.world.step(render=True)
            time.sleep(0.016)
        
        # 前进
        for _ in range(30):
            robot_controller._send_movement_command(0.4, 0.0)
            robot_controller.world.step(render=True)
            time.sleep(0.016)
        
        return True
    
    def _strategy_multiple_turns(self, robot_controller) -> bool:
        """策略2：多次转向"""
        directions = [1, -1, 1, -1]
        
        for direction in directions:
            for _ in range(25):
                robot_controller._send_movement_command(0.1, 1.8 * direction)
                robot_controller.world.step(render=True)
                time.sleep(0.016)
            
            # 短暂前进
            for _ in range(15):
                robot_controller._send_movement_command(0.3, 0.0)
                robot_controller.world.step(render=True)
                time.sleep(0.016)
        
        return True
    
    def _strategy_random_walk(self, robot_controller) -> bool:
        """策略3：随机游走"""
        for _ in range(100):
            linear_vel = np.random.uniform(-0.2, 0.4)
            angular_vel = np.random.uniform(-1.5, 1.5)
            
            robot_controller._send_movement_command(linear_vel, angular_vel)
            robot_controller.world.step(render=True)
            time.sleep(0.016)
        
        return True
    
    def _strategy_spiral_escape(self, robot_controller) -> bool:
        """策略4：螺旋逃逸"""
        for i in range(60):
            # 螺旋运动
            linear_vel = 0.2 + i * 0.005
            angular_vel = 1.0 + i * 0.02
            
            robot_controller._send_movement_command(linear_vel, angular_vel)
            robot_controller.world.step(render=True)
            time.sleep(0.016)
        
        return True

class AdvancedNavigationSystem:
    """高级导航系统集成"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        
        # 初始化组件
        self.path_planner = CUDAAcceleratedAStar(
            grid_resolution=config.NAVIGATION["grid_resolution"],
            map_size=config.NAVIGATION["map_size"]
        )
        
        self.movement_controller = SmoothMovementController(config)
        self.stuck_detector = IntelligentStuckDetector(config)
        self.breakthrough_strategy = AdvancedBreakthroughStrategy(config)
        
        # 导航状态
        self.current_path = []
        self.path_index = 0
        self.navigation_active = False
        
        # 性能监控
        self.navigation_stats = {
            'total_navigations': 0,
            'successful_navigations': 0,
            'average_time': 0.0,
            'stuck_events': 0
        }
    
    def navigate_to_target(self, robot_controller, target_pos: np.ndarray, 
                          max_time: float = 30.0, tolerance: float = 0.2) -> bool:
        """执行高级导航"""
        start_time = time.time()
        self.navigation_stats['total_navigations'] += 1
        
        try:
            # 获取当前位置
            current_pos, current_yaw = robot_controller.get_robot_pose()
            
            # 规划路径
            self.current_path = self.path_planner.plan_path(current_pos[:2], target_pos[:2])
            
            if len(self.current_path) > 2:
                print(f"   🗺️ 高级A*路径规划完成，{len(self.current_path)}个优化路径点")
            
            # 重置控制器
            self.movement_controller.reset()
            self.stuck_detector.reset()
            
            # 导航循环
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
                    print(f"   ✅ 高级导航成功！距离: {final_distance:.3f}m")
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
                
                # 检查路径点切换
                target_distance = np.linalg.norm(current_pos[:2] - current_target)
                if target_distance < 0.3:
                    self.path_index += 1
                
                # 卡住检测
                if self.stuck_detector.update(current_pos, linear_vel, angular_vel):
                    print(f"   🚨 检测到卡住，执行突破策略...")
                    self.navigation_stats['stuck_events'] += 1
                    
                    success = self.breakthrough_strategy.execute_breakthrough(
                        robot_controller, self.stuck_detector.stuck_count
                    )
                    
                    if success:
                        # 重新规划路径
                        current_pos, _ = robot_controller.get_robot_pose()
                        self.current_path = self.path_planner.plan_path(
                            current_pos[:2], target_pos[:2]
                        )
                        self.path_index = 1
                        self.stuck_detector.reset()
                        print(f"   ✅ 突破成功，重新规划路径")
                    else:
                        print(f"   ❌ 突破失败")
                        break
                
                # 渲染
                robot_controller.world.step(render=True)
                time.sleep(0.016)
            
            # 检查最终结果
            current_pos, _ = robot_controller.get_robot_pose()
            final_distance = np.linalg.norm(current_pos[:2] - target_pos[:2])
            
            if final_distance < tolerance * 1.5:
                print(f"   ✅ 高级导航接近成功！距离: {final_distance:.3f}m")
                self.navigation_stats['successful_navigations'] += 1
                return True
            else:
                print(f"   ⚠️ 高级导航失败，距离: {final_distance:.3f}m")
                return False
                
        except Exception as e:
            print(f"   ❌ 高级导航异常: {e}")
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
            'average_time': self.navigation_stats['average_time'],
            'stuck_events': self.navigation_stats['stuck_events']
        }
    
    def print_stats(self):
        """打印导航统计"""
        stats = self.get_navigation_stats()
        print(f"\n📊 高级导航统计:")
        print(f"   总导航次数: {stats['total_navigations']}")
        print(f"   成功率: {stats['success_rate']:.1f}%")
        print(f"   平均用时: {stats['average_time']:.1f}s")
        print(f"   卡住事件: {stats['stuck_events']}")