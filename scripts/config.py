#!/usr/bin/env python3
"""
Create-3+机械臂室内清洁系统配置文件
所有超参数和配置都在这里，方便调试和修改
"""

import numpy as np
import os

class CleanupSystemConfig:
    """清洁系统配置类"""

    def __init__(self, username='lwb'):
        # ==================== 用户配置 ====================
        # 获取当前用户名，支持多种方式
        if username is None:
            # 自动检测用户名的多种方式
            username = (
                os.environ.get('USER') or           # Linux/macOS
                os.environ.get('USERNAME') or       # Windows
                os.environ.get('LOGNAME') or        # 备用
                'user'                              # 默认值
            )
        
        self.USERNAME = username
        print(f"🔧 配置用户: {self.USERNAME}")
        
        # ==================== 文件路径配置 ====================
        # 用户相关路径配置 - 用户需要根据自己的安装情况修改这些路径
        self.USER_PATHS = {
            # Isaac Sim资产库根目录 - 请根据您的实际安装路径修改
            "isaac_assets_base": f"/home/{self.USERNAME}/isaacsim_assets/Assets/Isaac/4.5",
            
            # Isaac Sim安装目录 - 请根据您的实际安装路径修改
            "isaac_sim_install": f"/home/{self.USERNAME}/isaacsim",
            
            # 其他可能的资产路径（按优先级排序）
            "alternative_asset_paths": [
                f"/home/{self.USERNAME}/isaacsim_assets/Assets/Isaac/4.5",
                f"/home/{self.USERNAME}/.local/share/ov/pkg/isaac_sim-*/assets/Isaac/4.5",
                f"/opt/isaac_sim/assets/Isaac/4.5",
                f"/usr/local/isaac_sim/assets/Isaac/4.5",
            ],
            
            # 其他可能的Isaac Sim安装路径（按优先级排序）
            "alternative_isaac_paths": [
                f"/home/{self.USERNAME}/isaacsim",
                f"/home/{self.USERNAME}/.local/share/ov/pkg/isaac_sim-*",
                f"/opt/isaac_sim",
                f"/usr/local/isaac_sim",
            ]
        }
        
        # 自动检测资产路径
        self._detect_asset_paths()
        
        # 构建最终路径
        self.PATHS = {
            # 住宅资产库路径
            "residential_assets_root": os.path.join(
                self.USER_PATHS["isaac_assets_base"], 
                "NVIDIA/Assets/ArchVis/Residential"
            ),
            
            # 机器人模型路径
            "robot_usd_path": os.path.join(
                self.USER_PATHS["isaac_assets_base"], 
                "Isaac/Robots/iRobot/create_3_with_arm.usd"
            ),
            
            # 机器人在场景中的路径
            "robot_prim_path": "/World/create3_robot",
        }
        
        # 验证路径有效性
        self._validate_paths()
        
        # ==================== 缩放比例配置 ====================
        self.SCALE_CONFIG = {
            # 家具缩放 - 如果家具太大，减小这个值
            "furniture": 0.02,        # 2% (推荐范围: 0.02-0.05)
            
            # 垃圾物品缩放
            "small_trash": 0.02,       # 2% 略微缩小
            "large_trash": 0.02,       # 2% 略微缩小

            # 装饰物品缩放
            "books": 0.02,             # 2% 书籍大小

            # 如果所有物品都太大，可以添加全局缩放
            "global_scale": 1.0,      # 全局缩放倍数
        }
        
        # ==================== 家具位置配置 ====================
        self.FURNITURE_POSITIONS = {
            # 格式: "家具名": [x, y, z, rotation_z_degrees]
            "desk": [150.0, 80.0, 0.0, 0.0],
            "chair": [140.0, 60.0, 0.0, 0.0],
            "coffee_table": [-200.0, 180.0, 0.0, 0.0],
            "side_table": [350.0, -280.0, 0.0, 45.0],
            "console_table": [-450.0, -150.0, 0.0, 90.0],
            "bookshelf": [-380.0, -420.0, 0.0, 0.0],
        }
        
        # ==================== 书籍位置配置 ====================
        self.BOOK_POSITIONS = {
            # 格式: "书名": [x, y, z]
            "book1": [-370.0, -400.0, 0.8],
            "book2": [-350.0, -390.0, 0.8],
            "book3": [-330.0, -410.0, 0.8],
        }
        
        # ==================== 小垃圾位置配置 ====================
        self.SMALL_TRASH_POSITIONS = {
            # 格式: "物品名": [x, y, z]
            "orange1": [280.0, 150.0, 0.03],
            "lemon1": [520.0, -320.0, 0.03],
            "coaster": [-180.0, 450.0, 0.01],
            "eraser": [-680.0, 120.0, 0.015],
            "marble": [750.0, 80.0, 0.015],
            "orange2": [-420.0, 650.0, 0.03],
            "lemon2": [320.0, -580.0, 0.03],
        }
        
        # ==================== 大垃圾位置配置 ====================
        self.LARGE_TRASH_POSITIONS = {
            # 格式: "物品名": [x, y, z]
            "tin_can": [240.0, 360.0, 0.05],
            "mason_jar": [-325.0, -240.0, 0.05],
            "pencil": [190.0, -375.0, 0.05],
            "dice_d6": [425.0, 190.0, 0.05],
            "dice_d20": [-110.0, 440.0, 0.05],
        }
        
        # ==================== 机器人控制参数 ====================
        self.ROBOT_CONTROL = {
            # 移动参数
            "max_linear_velocity": 0.5,      # 最大线速度 (m/s)
            "max_angular_velocity": 1.8,     # 最大角速度 (rad/s)
            "movement_threshold": 0.6,       # 到达目标的距离阈值 (m)
            "angular_threshold": 0.3,        # 角度对齐阈值 (rad)
            
            # 速度平滑参数
            "velocity_smoothing": 0.2,       # 速度平滑系数 (0-1)
            
            # 轮子配置
            "wheel_joint_names": ["left_wheel_joint", "right_wheel_joint"],
            "wheel_radius": 0.036,           # 轮子半径 (m)
            "wheel_base": 0.235,             # 轮距 (m)
        }
        
        # ==================== 机械臂配置 ====================
        self.ARM_CONFIG = {
            # 关节名称
            "joint_names": [
                "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                "panda_joint5", "panda_joint6", "panda_joint7"
            ],
            
            # 夹爪关节名称
            "gripper_joint_names": ["panda_finger_joint1", "panda_finger_joint2"],
            
            # 夹爪状态
            "gripper_open": 0.04,            # 张开位置 (m)
            "gripper_closed": 0.0,           # 闭合位置 (m)
            
            # 机械臂预设姿态
            "poses": {
                "home": [0.0, -0.569, 0.0, -2.810, 0.0, 2.0, 0.741],
                "ready": [0.0, -0.3, 0.0, -1.8, 0.0, 1.6, 0.785],
                "pickup": [0.0, 0.5, 0.0, -1.6, 0.0, 2.4, 0.785],
                "pickup_low": [0.0, 0.7, 0.0, -1.4, 0.0, 2.6, 0.785],
                "stow": [0.0, -1.2, 0.0, -2.8, 0.0, 1.5, 0.0],
                "carry": [0.0, -0.5, 0.0, -2.0, 0.0, 1.6, 0.785],
                "inspect": [0.0, 0.2, 0.0, -1.8, 0.0, 2.0, 0.0],
            }
        }
        
        # ==================== 导航参数配置 ====================
        self.NAVIGATION = {
            # A*路径规划参数
            "grid_resolution": 0.2,          # 网格分辨率 (m)
            "map_size": 20,                  # 地图大小 (m)
            
            # 导航容差
            "tolerance_small_trash": 0.6,    # 小垃圾导航容差 (m)
            "tolerance_large_trash": 0.7,    # 大垃圾导航容差 (m)
            "tolerance_home": 0.5,           # 返回家位置容差 (m)
            
            # 卡住检测参数
            "stuck_threshold": 0.08,         # 卡住检测阈值 (m)
            "stuck_detection_window": 200,   # 检测窗口大小 (步数)
            "stuck_timeout": 8.0,            # 卡住超时时间 (s)
            
            # 导航超时
            "nav_timeout_small": 25,         # 小垃圾导航超时 (s)
            "nav_timeout_large": 30,         # 大垃圾导航超时 (s)
            "nav_timeout_home": 15,          # 返回家超时 (s)
            
            # 控制策略参数
            "angle_threshold_large": 0.8,    # 大角度阈值 (rad)
            "angle_threshold_medium": 0.3,   # 中等角度阈值 (rad)
            "linear_velocity_factors": {     # 线速度计算因子
                "min": 0.2,
                "max": 0.5,
                "distance_factor": 0.8
            }
        }
        
        # ==================== 物理参数配置 ====================
        self.PHYSICS = {
            # 仿真频率
            "physics_dt": 1.0/120.0,         # 物理时间步 (120Hz)
            "rendering_dt": 1.0/60.0,        # 渲染时间步 (60Hz)
            
            # GPU加速参数
            "gpu_max_rigid_contact_count": 1024*1024,
            "gpu_max_rigid_patch_count": 80*1024,
            "gpu_heap_capacity": 64*1024*1024,
            "gpu_temp_buffer_capacity": 16*1024*1024,
            "gpu_max_num_partitions": 8,
            
            # 求解器参数
            "solver_position_iterations": 8,  # 位置求解器迭代次数
            "solver_velocity_iterations": 4,  # 速度求解器迭代次数
            
            # 地面摩擦参数
            "ground_static_friction": 1.2,
            "ground_dynamic_friction": 1.0,
            "ground_restitution": 0.05,
            
            # 机器人物理参数
            "robot_mass": 4.0,               # 机器人质量 (kg)
            "robot_com_offset": [0.0, 0.0, -0.05],  # 质心偏移
            "robot_inertia": [0.12, 0.12, 0.06],    # 惯性张量
        }
        
        # ==================== 关节控制参数 ====================
        self.JOINT_CONTROL = {
            # 轮子关节参数
            "wheel_kp": 0.0,                 # 轮子位置增益
            "wheel_kd": 800.0,               # 轮子速度增益
            
            # 机械臂关节参数
            "arm_kp": 1000.0,                # 机械臂位置增益
            "arm_kd": 50.0,                  # 机械臂速度增益
            
            # 夹爪关节参数
            "gripper_kp": 2e5,               # 夹爪位置增益
            "gripper_kd": 2e3,               # 夹爪速度增益
            
            # 其他关节参数
            "default_kp": 8000.0,            # 默认位置增益
            "default_kd": 1500.0,            # 默认速度增益
        }
        
        # ==================== 成功率配置 ====================
        self.SUCCESS_RATES = {
            "grasp_success_probability": 0.85,  # 抓取成功概率
            "collection_retry_attempts": 1,      # 收集重试次数
        }
        
        # ==================== 资产文件映射 ====================
        self.ASSET_PATHS = {
            # 家具配置 (选择小尺寸文件)
            "furniture": {
                "desk": "Furniture/Desks/Desk_01.usd",
                "chair": "Furniture/Chairs/Chair_Desk.usd",
                "coffee_table": "Furniture/CoffeeTables/Midtown.usd",
                "side_table": "Furniture/EndTables/Festus01.usd",
                "console_table": "Furniture/SofaTables/Ellisville.usd",
                "bookshelf": "Furniture/Bookshelves/Fenton.usd",
            },
            
            # 小垃圾物品 (吸附收集)
            "small_trash": {
                "orange1": "Decor/Tchotchkes/Orange_01.usd",
                "orange2": "Decor/Tchotchkes/Orange_02.usd",
                "lemon1": "Decor/Tchotchkes/Lemon_01.usd",
                "lemon2": "Decor/Tchotchkes/Lemon_02.usd",
                "coaster": "Decor/Coasters/Coaster_Hexagon.usd",
                "eraser": "Misc/Supplies/Eraser.usd",
                "marble": "Entertainment/Games/Solid_Marble.usd",
            },
            
            # 大垃圾物品 (机械臂抓取)
            "large_trash": {
                "tin_can": "Food/Containers/TinCan.usd",
                "mason_jar": "Food/Containers/MasonJar.usd",
                "pencil": "Misc/Supplies/MechanicalPencil.usd",
                "dice_d6": "Entertainment/Games/DiceSet/D6.usd",
                "dice_d20": "Entertainment/Games/DiceSet/D20.usd",
            },
            
            # 书籍装饰
            "books": {
                "book1": "Decor/Books/Book_01.usd",
                "book2": "Decor/Books/Book_02.usd",
                "book3": "Decor/Books/Book_11.usd",
            }
        }
        
        # ==================== 照明配置 ====================
        self.LIGHTING = {
            "distant_light_intensity": 5000,   # 远距离光照强度
            "distant_light_color": (1.0, 1.0, 0.9),  # 暖白光颜色
        }
        
        # ==================== 调试配置 ====================
        self.DEBUG = {
            "enable_debug_output": True,       # 启用调试输出
            "show_robot_state": True,          # 显示机器人状态
            "show_navigation_progress": True,   # 显示导航进度
            "show_grasp_details": True,        # 显示抓取详情
            "progress_report_interval": 2.5,   # 进度报告间隔 (s)
        }
        
        # ==================== 实验配置 ====================
        self.EXPERIMENT = {
            "run_arm_pose_demo": True,         # 运行机械臂姿态演示
            "demo_poses": ["home", "ready", "inspect", "pickup", "pickup_low", "carry", "stow"],
            "stabilization_time": 2.0,         # 稳定时间 (s)
            "collection_delay": 0.5,           # 收集间隔 (s)
        }
    
    # ==================== 路径检测和验证方法 ====================
    
    def _detect_asset_paths(self):
        """自动检测Isaac Sim资产路径"""
        print("🔍 自动检测Isaac Sim安装路径...")
        
        # 检测Isaac Sim资产路径
        for path in self.USER_PATHS["alternative_asset_paths"]:
            # 处理通配符路径
            if '*' in path:
                import glob
                matches = glob.glob(path)
                if matches:
                    path = matches[0]  # 使用第一个匹配的路径
            
            if os.path.exists(path):
                self.USER_PATHS["isaac_assets_base"] = path
                print(f"✅ 找到Isaac资产路径: {path}")
                break
        else:
            print(f"⚠️ 使用默认Isaac资产路径: {self.USER_PATHS['isaac_assets_base']}")
        
        # 检测Isaac Sim安装路径
        for path in self.USER_PATHS["alternative_isaac_paths"]:
            # 处理通配符路径
            if '*' in path:
                import glob
                matches = glob.glob(path)
                if matches:
                    path = matches[0]
            
            if os.path.exists(path):
                self.USER_PATHS["isaac_sim_install"] = path
                print(f"✅ 找到Isaac Sim安装路径: {path}")
                break
        else:
            print(f"⚠️ 使用默认Isaac Sim安装路径: {self.USER_PATHS['isaac_sim_install']}")
    
    def _validate_paths(self):
        """验证关键路径的有效性"""
        print("🔍 验证路径有效性...")
        
        validation_results = {}
        
        # 验证住宅资产库
        residential_path = self.PATHS["residential_assets_root"]
        if os.path.exists(residential_path):
            validation_results["residential_assets"] = "✅ 有效"
        else:
            validation_results["residential_assets"] = "❌ 缺失"
            print(f"⚠️ 住宅资产库路径不存在: {residential_path}")
        
        # 验证机器人模型
        robot_path = self.PATHS["robot_usd_path"]
        if os.path.exists(robot_path):
            validation_results["robot_model"] = "✅ 有效"
        else:
            validation_results["robot_model"] = "❌ 缺失"
            print(f"⚠️ 机器人模型路径不存在: {robot_path}")
        
        # 验证Isaac Sim安装
        isaac_path = self.USER_PATHS["isaac_sim_install"]
        if os.path.exists(isaac_path):
            validation_results["isaac_sim"] = "✅ 有效"
        else:
            validation_results["isaac_sim"] = "❌ 缺失"
            print(f"⚠️ Isaac Sim安装路径不存在: {isaac_path}")
        
        # 存储验证结果
        self._path_validation_results = validation_results
        
        # 如果有缺失的路径，给出配置建议
        if "❌ 缺失" in validation_results.values():
            print("\n" + "="*60)
            print("📋 路径配置建议:")
            print("请在config.py中更新以下路径，或设置正确的用户名:")
            print("1. 确认您的用户名是否正确")
            print("2. 检查Isaac Sim是否正确安装")
            print("3. 检查住宅资产包是否已下载")
            print("="*60)
    
    def set_user_paths(self, isaac_assets_base=None, isaac_sim_install=None):
        """手动设置用户路径"""
        if isaac_assets_base:
            self.USER_PATHS["isaac_assets_base"] = isaac_assets_base
            print(f"🔧 手动设置Isaac资产路径: {isaac_assets_base}")
        
        if isaac_sim_install:
            self.USER_PATHS["isaac_sim_install"] = isaac_sim_install
            print(f"🔧 手动设置Isaac Sim路径: {isaac_sim_install}")
        
        # 重新构建路径
        self.PATHS.update({
            "residential_assets_root": os.path.join(
                self.USER_PATHS["isaac_assets_base"], 
                "NVIDIA/Assets/ArchVis/Residential"
            ),
            "robot_usd_path": os.path.join(
                self.USER_PATHS["isaac_assets_base"], 
                "Isaac/Robots/iRobot/create_3_with_arm.usd"
            ),
        })
        
        # 重新验证
        self._validate_paths()
    
    # ==================== 便捷方法 ====================
    
    def get_full_asset_path(self, category, item_name):
        """获取资产的完整路径"""
        if category in self.ASSET_PATHS and item_name in self.ASSET_PATHS[category]:
            relative_path = self.ASSET_PATHS[category][item_name]
            return os.path.join(self.PATHS["residential_assets_root"], relative_path)
        return None
    
    def update_scale(self, **kwargs):
        """更新缩放配置"""
        for key, value in kwargs.items():
            if key in self.SCALE_CONFIG:
                self.SCALE_CONFIG[key] = value
                print(f"🔧 {key} 缩放更新为: {value}")
    
    def add_furniture_position(self, name, x, y, z, rotation=0.0):
        """添加家具位置"""
        self.FURNITURE_POSITIONS[name] = [x, y, z, rotation]
        print(f"🪑 添加家具位置: {name} -> ({x}, {y}, {z}, {rotation}°)")
    
    def add_trash_position(self, category, name, x, y, z):
        """添加垃圾位置"""
        if category == "small":
            self.SMALL_TRASH_POSITIONS[name] = [x, y, z]
            print(f"🔸 添加小垃圾位置: {name} -> ({x}, {y}, {z})")
        elif category == "large":
            self.LARGE_TRASH_POSITIONS[name] = [x, y, z]
            print(f"🔹 添加大垃圾位置: {name} -> ({x}, {y}, {z})")
    
    def update_robot_control(self, **kwargs):
        """更新机器人控制参数"""
        for key, value in kwargs.items():
            if key in self.ROBOT_CONTROL:
                self.ROBOT_CONTROL[key] = value
                print(f"🤖 机器人参数更新: {key} = {value}")
    
    def update_navigation(self, **kwargs):
        """更新导航参数"""
        for key, value in kwargs.items():
            if key in self.NAVIGATION:
                self.NAVIGATION[key] = value
                print(f"🧭 导航参数更新: {key} = {value}")
    
    def print_summary(self):
        """打印配置摘要"""
        print("\n" + "="*60)
        print("📋 清洁系统配置摘要")
        print("="*60)
        print(f"👤 用户: {self.USERNAME}")
        print(f"🏠 住宅资产库: {self.PATHS['residential_assets_root']}")
        print(f"🤖 机器人模型: {self.PATHS['robot_usd_path']}")
        print(f"🔧 Isaac Sim: {self.USER_PATHS['isaac_sim_install']}")
        
        # 显示路径验证结果
        if hasattr(self, '_path_validation_results'):
            print("📂 路径验证结果:")
            for key, status in self._path_validation_results.items():
                print(f"   - {key}: {status}")
        
        print(f"📏 缩放配置:")
        for key, value in self.SCALE_CONFIG.items():
            print(f"   - {key}: {value}")
        print(f"🪑 家具数量: {len(self.FURNITURE_POSITIONS)}")
        print(f"🔸 小垃圾数量: {len(self.SMALL_TRASH_POSITIONS)}")
        print(f"🔹 大垃圾数量: {len(self.LARGE_TRASH_POSITIONS)}")
        print(f"📚 书籍数量: {len(self.BOOK_POSITIONS)}")
        print(f"🚀 最大线速度: {self.ROBOT_CONTROL['max_linear_velocity']} m/s")
        print(f"🌀 最大角速度: {self.ROBOT_CONTROL['max_angular_velocity']} rad/s")
        print("="*60)

# ==================== 快速配置预设 ====================

class QuickConfigs:
    """快速配置预设"""
    
    @staticmethod
    def small_scene(username=None):
        """小场景配置 - 减少物品数量，提高性能"""
        config = CleanupSystemConfig(username)
        
        # 只保留核心家具
        config.FURNITURE_POSITIONS = {
            "desk": [2.0, 1.5, 0.0, 0.0],
            "chair": [2.0, 0.8, 0.0, 0.0],
            "coffee_table": [-1.5, 1.0, 0.0, 0.0],
        }
        
        # 减少垃圾数量
        config.SMALL_TRASH_POSITIONS = {
            "orange1": [1.0, 0.5, 0.03],
            "lemon1": [1.5, -0.5, 0.03],
            "coaster": [-1.0, 0.8, 0.01],
        }
        
        config.LARGE_TRASH_POSITIONS = {
            "tin_can": [1.8, 1.2, 0.05],
            "mason_jar": [-1.2, -1.0, 0.05],
        }
        
        return config
    
    @staticmethod
    def tiny_furniture(username=None):
        """超小家具配置 - 如果默认缩放还是太大"""
        config = CleanupSystemConfig(username)
        config.update_scale(
            furniture=0.01,      # 1% - 非常小的家具
            books=0.2,           # 20% - 很小的书籍
            large_trash=0.5,     # 50% - 更小的大垃圾
        )
        return config
    
    @staticmethod
    def performance_optimized(username=None):
        """性能优化配置 - 提高仿真性能"""
        config = CleanupSystemConfig(username)
        
        # 降低物理频率
        config.PHYSICS["physics_dt"] = 1.0/60.0  # 60Hz instead of 120Hz
        
        # 减少GPU资源使用
        config.PHYSICS["gpu_max_rigid_contact_count"] = 512*1024
        config.PHYSICS["gpu_heap_capacity"] = 32*1024*1024
        
        # 降低求解器精度换取性能
        config.PHYSICS["solver_position_iterations"] = 4
        config.PHYSICS["solver_velocity_iterations"] = 2
        
        return config
    
    @staticmethod
    def debug_mode(username=None):
        """调试模式配置 - 更多调试信息"""
        config = CleanupSystemConfig(username)
        
        # 启用所有调试输出
        config.DEBUG.update({
            "enable_debug_output": True,
            "show_robot_state": True,
            "show_navigation_progress": True,
            "show_grasp_details": True,
            "progress_report_interval": 1.0,  # 更频繁的报告
        })
        
        # 降低速度便于观察
        config.ROBOT_CONTROL.update({
            "max_linear_velocity": 0.3,
            "max_angular_velocity": 1.0,
        })
        
        return config

# ==================== 使用示例 ====================

def example_usage():
    """配置文件使用示例"""
    
    # 1. 使用默认配置（自动检测用户名）
    config = CleanupSystemConfig()
    
    # 2. 指定用户名
    # config = CleanupSystemConfig(username="your_username")
    
    # 3. 手动设置路径（如果自动检测失败）
    # config.set_user_paths(
    #     isaac_assets_base="/path/to/your/isaac/assets",
    #     isaac_sim_install="/path/to/your/isaac/sim"
    # )
    
    # 4. 修改缩放比例
    config.update_scale(furniture=0.02, books=0.3)
    
    # 5. 添加新的家具位置
    config.add_furniture_position("sofa", 0.0, 3.0, 0.0, 180.0)
    
    # 6. 添加新的垃圾位置
    config.add_trash_position("small", "pen", 0.5, 1.8, 0.02)
    config.add_trash_position("large", "bottle", 1.5, 2.5, 0.05)
    
    # 7. 调整机器人参数
    config.update_robot_control(max_linear_velocity=0.4, max_angular_velocity=1.5)
    
    # 8. 调整导航参数
    config.update_navigation(stuck_threshold=0.1, nav_timeout_small=20)
    
    # 9. 使用快速预设
    # small_config = QuickConfigs.small_scene("your_username")
    # tiny_config = QuickConfigs.tiny_furniture("your_username")
    # perf_config = QuickConfigs.performance_optimized("your_username")
    # debug_config = QuickConfigs.debug_mode("your_username")
    
    # 10. 打印配置摘要
    config.print_summary()
    
    return config

if __name__ == "__main__":
    # 测试配置
    config = example_usage()