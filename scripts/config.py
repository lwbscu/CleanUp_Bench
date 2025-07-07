#!/usr/bin/env python3
"""
OSGT四类物体标准室内清洁系统配置文件（通用版）
O类-障碍物 | S类-可清扫物 | G类-可抓取物 | T类-任务区
适配场景：家庭住宅、学校、医院、工厂等
"""

import numpy as np
import os

class OSGTCleanupSystemConfig:
    """OSGT标准清洁系统配置类（四类物体通用版）"""

    def __init__(self, username=None, scenario_type="residential"):
        # ==================== 用户配置 ====================
        if username is None:
            username = (
                os.environ.get('USER') or           # Linux/macOS
                os.environ.get('USERNAME') or       # Windows
                os.environ.get('LOGNAME') or        # 备用
                'user'                              # 默认值
            )
        
        self.USERNAME = username
        self.SCENARIO_TYPE = scenario_type  # residential, school, hospital, factory
        print(f"🔧 配置用户: {self.USERNAME}")
        print(f"🏢 场景类型: {self.SCENARIO_TYPE}")
        
        # ==================== 文件路径配置 ====================
        self.USER_PATHS = {
            "isaac_assets_base": f"/home/{self.USERNAME}/isaacsim_assets/Assets/Isaac/4.5",
            "isaac_sim_install": f"/home/{self.USERNAME}/isaacsim",
            "alternative_asset_paths": [
                f"/home/{self.USERNAME}/isaacsim_assets/Assets/Isaac/4.5",
                f"/home/{self.USERNAME}/.local/share/ov/pkg/isaac_sim-*/assets/Isaac/4.5",
                f"/opt/isaac_sim/assets/Isaac/4.5",
                f"/usr/local/isaac_sim/assets/Isaac/4.5",
            ],
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
            "residential_assets_root": os.path.join(
                self.USER_PATHS["isaac_assets_base"], 
                "NVIDIA/Assets/ArchVis/Residential"
            ),
            "robot_usd_path": os.path.join(
                self.USER_PATHS["isaac_assets_base"], 
                "Isaac/Robots/iRobot/create_3_with_arm.usd"
            ),
            "robot_prim_path": "/World/create3_robot",
        }
        
        # 验证路径有效性
        self._validate_paths()
        
        # ==================== OSGT四类物体缩放配置 ====================
        self.SCALE_CONFIG = {
            # O类 - 障碍物缩放
            "obstacles": 0.02,           # 2% (环境障碍物：家具、设备等)
            
            # S类 - 可清扫物缩放  
            "sweepable_items": 0.02,     # 2% (小颗粒物：纸屑、碎渣等)
            
            # G类 - 可抓取物缩放
            "graspable_items": 0.02,     # 2% (可抓取物：工具、容器等)
            
            # T类 - 任务区缩放
            "task_areas": 0.02,          # 2% (回收区、存放区等)

            # 全局缩放
            "global_scale": 1.0,         # 全局缩放倍数
        }
        
        # ==================== OSGT四类物体位置配置 ====================
        
        # O类 - 障碍物位置配置 (Obstacles)
        self.OBSTACLES_POSITIONS = {
            # 格式: "障碍物名": [x, y, z, rotation_z_degrees]
            # 适配多场景：家庭(桌椅)、学校(课桌)、医院(病床)、工厂(设备)
            "obstacle_1": [150.0, 80.0, 0.0, 0.0],      # 主要工作台/桌面
            "obstacle_2": [140.0, 60.0, 0.0, 0.0],      # 座椅/推车
            "obstacle_3": [-200.0, 180.0, 0.0, 0.0],    # 中央设施
            "obstacle_4": [350.0, -280.0, 0.0, 45.0],   # 边角设备
            "obstacle_5": [-450.0, -150.0, 0.0, 90.0],  # 存储设施
            "obstacle_6": [-380.0, -420.0, 0.0, 0.0],   # 大型设备/书架
        }
        
        # S类 - 可清扫物位置配置 (Sweepable Items)
        self.SWEEPABLE_POSITIONS = {
            # 格式: "可清扫物名": [x, y, z]
            # 小颗粒物质：纸屑、食物碎渣、灰尘、金属屑等
            "sweepable_1": [280.0, 150.0, 0.03],        # 工作区域碎渣
            "sweepable_2": [520.0, -320.0, 0.03],       # 角落积尘
            "sweepable_3": [-180.0, 450.0, 0.01],       # 地面碎片
            "sweepable_4": [-680.0, 120.0, 0.015],      # 清洁盲区
            "sweepable_5": [750.0, 80.0, 0.015],        # 设备下方
            "sweepable_6": [-420.0, 650.0, 0.03],       # 通道区域
            "sweepable_7": [320.0, -580.0, 0.03],       # 边缘区域
        }
        
        # G类 - 可抓取物位置配置 (Graspable Items)
        self.GRASPABLE_POSITIONS = {
            # 格式: "可抓取物名": [x, y, z]
            # 工具、容器、书籍、零件等需要机械臂抓取的物体
            "graspable_1": [240.0, 360.0, 0.05],        # 容器类
            "graspable_2": [-325.0, -240.0, 0.05],      # 工具类
            "graspable_3": [190.0, -375.0, 0.05],       # 文具类
            "graspable_4": [425.0, 190.0, 0.05],        # 零件类
            "graspable_5": [-110.0, 440.0, 0.05],       # 设备类
            # 书籍等特殊可抓取物
            "graspable_book_1": [-370.0, -400.0, 0.8],  # 桌面书籍
            "graspable_book_2": [-350.0, -390.0, 0.8],  # 散落书本
            "graspable_book_3": [-330.0, -410.0, 0.8],  # 文档资料
        }
        
        # T类 - 任务区位置配置 (Task Areas)
        self.TASK_AREAS_POSITIONS = {
            # 格式: "任务区名": [x, y, z, rotation_z_degrees]
            # 回收区、分拣区、存放区等
            "collection_zone_s": [800.0, 800.0, 0.0, 0.0],     # S类回收区
            "collection_zone_g": [-800.0, 800.0, 0.0, 0.0],    # G类存放区
            "sorting_area": [0.0, 900.0, 0.0, 0.0],             # 分拣中心
            "maintenance_station": [0.0, -900.0, 0.0, 0.0],     # 维护站点
        }
        
        # ==================== 机器人控制参数 ====================
        self.ROBOT_CONTROL = {
            "max_linear_velocity": 0.5,      # 最大线速度 (m/s)
            "max_angular_velocity": 2.0,     # 最大角速度 (rad/s)
            "movement_threshold": 0.4,       # 到达目标的距离阈值 (m)
            "angular_threshold": 0.15,       # 角度对齐阈值 (rad)
            "velocity_smoothing": 0.05,      # 速度平滑系数 (0-1)
            "wheel_joint_names": ["left_wheel_joint", "right_wheel_joint"],
            "wheel_radius": 0.036,           # 轮子半径 (m)
            "wheel_base": 0.235,             # 轮距 (m)
        }
        
        # ==================== 机械臂配置 ====================
        self.ARM_CONFIG = {
            "joint_names": [
                "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                "panda_joint5", "panda_joint6", "panda_joint7"
            ],
            "gripper_joint_names": ["panda_finger_joint1", "panda_finger_joint2"],
            "gripper_open": 0.04,            # 张开位置 (m)
            "gripper_closed": 0.008,         # 闭合位置 (m)
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
        
        # ==================== OSGT导航参数配置 ====================
        self.NAVIGATION = {
            "grid_resolution": 0.2,          # 网格分辨率 (m)
            "map_size": 20,                  # 地图大小 (m)
            
            # OSGT四类导航容差
            "tolerance_obstacles": 0.5,          # O类障碍物避让距离 (m)
            "tolerance_sweepable": 1.2,         # S类可清扫物导航容差 (m)
            "tolerance_graspable": 1.3,         # G类可抓取物导航容差 (m)
            "tolerance_task_areas": 0.8,        # T类任务区容差 (m)
            
            # 导航超时配置
            "nav_timeout_sweepable": 45,         # S类导航超时 (s)
            "nav_timeout_graspable": 50,         # G类导航超时 (s)
            "nav_timeout_task_areas": 25,        # T类导航超时 (s)
            
            # 控制策略参数
            "angle_threshold_large": 2.5,       # 大角度阈值 (rad)
            "angle_threshold_medium": 1.5,      # 中等角度阈值 (rad)
            "angle_threshold_small": 0.8,       # 小角度阈值 (rad)
            "linear_velocity_factors": {
                "min": 0.2,
                "max": 0.5,
                "distance_factor": 0.8
            }
        }
        
        # ==================== 物理参数配置 ====================
        self.PHYSICS = {
            "physics_dt": 1.0/120.0,         # 物理时间步 (120Hz)
            "rendering_dt": 1.0/60.0,        # 渲染时间步 (60Hz)
            "gpu_max_rigid_contact_count": 1024*1024,
            "gpu_max_rigid_patch_count": 80*1024,
            "gpu_heap_capacity": 64*1024*1024,
            "gpu_temp_buffer_capacity": 16*1024*1024,
            "gpu_max_num_partitions": 8,
            "solver_position_iterations": 6,
            "solver_velocity_iterations": 3,
            "ground_static_friction": 1.0,
            "ground_dynamic_friction": 0.8,
            "ground_restitution": 0.02,
            "robot_mass": 4.0,
            "robot_com_offset": [0.0, 0.0, -0.05],
            "robot_inertia": [0.12, 0.12, 0.06],
        }
        
        # ==================== 关节控制参数 ====================
        self.JOINT_CONTROL = {
            "wheel_kp": 0.0,                 # 轮子位置增益
            "wheel_kd": 1200.0,              # 轮子速度增益
            "arm_kp": 1000.0,                # 机械臂位置增益
            "arm_kd": 50.0,                  # 机械臂速度增益
            "gripper_kp": 2e5,               # 夹爪位置增益
            "gripper_kd": 2e3,               # 夹爪速度增益
            "default_kp": 8000.0,            # 默认位置增益
            "default_kd": 1500.0,            # 默认速度增益
        }
        
        # ==================== 成功率配置 ====================
        self.SUCCESS_RATES = {
            "grasp_success_probability": 0.85,  # 抓取成功概率
            "collection_retry_attempts": 1,      # 收集重试次数
        }
        
        # ==================== OSGT资产文件映射 ====================
        self.ASSET_PATHS = {
            # O类 - 障碍物配置 (通用环境障碍)
            "obstacles": {
                "obstacle_1": "Furniture/Desks/Desk_01.usd",          # 桌面/工作台
                "obstacle_2": "Furniture/Chairs/Chair_Desk.usd",      # 座椅/推车
                "obstacle_3": "Furniture/CoffeeTables/Midtown.usd",   # 中央设施
                "obstacle_4": "Furniture/EndTables/Festus01.usd",     # 边角设备
                "obstacle_5": "Furniture/SofaTables/Ellisville.usd",  # 存储设施
                "obstacle_6": "Furniture/Bookshelves/Fenton.usd",     # 大型设备
            },
            
            # S类 - 可清扫物配置 (小颗粒吸附收集)
            "sweepable_items": {
                "sweepable_1": "Decor/Tchotchkes/Orange_01.usd",      # 有机碎渣
                "sweepable_2": "Decor/Tchotchkes/Orange_02.usd",      # 食物残渣
                "sweepable_3": "Decor/Tchotchkes/Lemon_01.usd",       # 小型碎片
                "sweepable_4": "Decor/Tchotchkes/Lemon_02.usd",       # 细小颗粒
                "sweepable_5": "Decor/Coasters/Coaster_Hexagon.usd",  # 薄片物
                "sweepable_6": "Misc/Supplies/Eraser.usd",            # 橡胶碎片
                "sweepable_7": "Entertainment/Games/Solid_Marble.usd", # 滚珠颗粒
            },
            
            # G类 - 可抓取物配置 (机械臂精确抓取)
            "graspable_items": {
                "graspable_1": "Food/Containers/TinCan.usd",          # 容器类
                "graspable_2": "Food/Containers/MasonJar.usd",        # 瓶罐类
                "graspable_3": "Misc/Supplies/MechanicalPencil.usd",  # 工具类
                "graspable_4": "Entertainment/Games/DiceSet/D6.usd",   # 小型零件
                "graspable_5": "Entertainment/Games/DiceSet/D20.usd",  # 精密器件
                # 书籍文档类
                "graspable_book_1": "Decor/Books/Book_01.usd",
                "graspable_book_2": "Decor/Books/Book_02.usd", 
                "graspable_book_3": "Decor/Books/Book_11.usd",
            },
            
            # T类 - 任务区配置 (基础形状表示功能区)
            "task_areas": {
                "collection_zone_s": "Furniture/Desks/Desk_01.usd",   # S类回收台
                "collection_zone_g": "Furniture/Desks/Desk_01.usd",   # G类存放台
                "sorting_area": "Furniture/CoffeeTables/Midtown.usd",  # 分拣中心
                "maintenance_station": "Furniture/EndTables/Festus01.usd", # 维护站点
            }
        }
        
        # ==================== 照明配置 ====================
        self.LIGHTING = {
            "distant_light_intensity": 5000,
            "distant_light_color": (1.0, 1.0, 0.9),
        }
        
        # ==================== 调试配置 ====================
        self.DEBUG = {
            "enable_debug_output": True,
            "show_robot_state": True,
            "show_navigation_progress": True,
            "show_grasp_details": True,
            "progress_report_interval": 2.5,
        }
        
        # ==================== 实验配置 ====================
        self.EXPERIMENT = {
            "run_arm_pose_demo": True,
            "demo_poses": ["home", "ready", "inspect", "pickup", "pickup_low", "carry", "stow"],
            "stabilization_time": 2.0,
            "collection_delay": 0.3,
        }
    
    # ==================== 路径检测和验证方法 ====================
    
    def _detect_asset_paths(self):
        """自动检测Isaac Sim资产路径"""
        print("🔍 自动检测Isaac Sim安装路径...")
        
        for path in self.USER_PATHS["alternative_asset_paths"]:
            if '*' in path:
                import glob
                matches = glob.glob(path)
                if matches:
                    path = matches[0]
            
            if os.path.exists(path):
                self.USER_PATHS["isaac_assets_base"] = path
                print(f"✅ 找到Isaac资产路径: {path}")
                break
        else:
            print(f"⚠️ 使用默认Isaac资产路径: {self.USER_PATHS['isaac_assets_base']}")
        
        for path in self.USER_PATHS["alternative_isaac_paths"]:
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
        
        residential_path = self.PATHS["residential_assets_root"]
        if os.path.exists(residential_path):
            validation_results["residential_assets"] = "✅ 有效"
        else:
            validation_results["residential_assets"] = "❌ 缺失"
            print(f"⚠️ 住宅资产库路径不存在: {residential_path}")
        
        robot_path = self.PATHS["robot_usd_path"]
        if os.path.exists(robot_path):
            validation_results["robot_model"] = "✅ 有效"
        else:
            validation_results["robot_model"] = "❌ 缺失"
            print(f"⚠️ 机器人模型路径不存在: {robot_path}")
        
        isaac_path = self.USER_PATHS["isaac_sim_install"]
        if os.path.exists(isaac_path):
            validation_results["isaac_sim"] = "✅ 有效"
        else:
            validation_results["isaac_sim"] = "❌ 缺失"
            print(f"⚠️ Isaac Sim安装路径不存在: {isaac_path}")
        
        self._path_validation_results = validation_results
        
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
        
        self._validate_paths()
    
    # ==================== OSGT便捷方法 ====================
    
    def get_full_asset_path(self, osgt_category, item_name):
        """获取OSGT分类资产的完整路径"""
        if osgt_category in self.ASSET_PATHS and item_name in self.ASSET_PATHS[osgt_category]:
            relative_path = self.ASSET_PATHS[osgt_category][item_name]
            return os.path.join(self.PATHS["residential_assets_root"], relative_path)
        return None
    
    def update_scale(self, **kwargs):
        """更新OSGT缩放配置"""
        for key, value in kwargs.items():
            if key in self.SCALE_CONFIG:
                self.SCALE_CONFIG[key] = value
                print(f"🔧 OSGT-{key} 缩放更新为: {value}")
    
    def add_obstacle_position(self, name, x, y, z, rotation=0.0):
        """添加O类障碍物位置"""
        self.OBSTACLES_POSITIONS[name] = [x, y, z, rotation]
        print(f"🚧 添加O类障碍物: {name} -> ({x}, {y}, {z}, {rotation}°)")
    
    def add_sweepable_position(self, name, x, y, z):
        """添加S类可清扫物位置"""
        self.SWEEPABLE_POSITIONS[name] = [x, y, z]
        print(f"🧹 添加S类可清扫物: {name} -> ({x}, {y}, {z})")
    
    def add_graspable_position(self, name, x, y, z):
        """添加G类可抓取物位置"""
        self.GRASPABLE_POSITIONS[name] = [x, y, z]
        print(f"🦾 添加G类可抓取物: {name} -> ({x}, {y}, {z})")
    
    def add_task_area_position(self, name, x, y, z, rotation=0.0):
        """添加T类任务区位置"""
        self.TASK_AREAS_POSITIONS[name] = [x, y, z, rotation]
        print(f"🎯 添加T类任务区: {name} -> ({x}, {y}, {z}, {rotation}°)")
    
    def update_robot_control(self, **kwargs):
        """更新机器人控制参数"""
        for key, value in kwargs.items():
            if key in self.ROBOT_CONTROL:
                self.ROBOT_CONTROL[key] = value
                print(f"🤖 机器人参数更新: {key} = {value}")
    
    def update_navigation(self, **kwargs):
        """更新OSGT导航参数"""
        for key, value in kwargs.items():
            if key in self.NAVIGATION:
                self.NAVIGATION[key] = value
                print(f"🧭 OSGT导航参数更新: {key} = {value}")
    
    def set_scenario_type(self, scenario_type):
        """设置场景类型并调整参数"""
        self.SCENARIO_TYPE = scenario_type
        print(f"🏢 切换场景类型: {scenario_type}")
        
        # 根据场景类型调整参数
        if scenario_type == "residential":
            self.update_navigation(
                tolerance_sweepable=1.2,
                tolerance_graspable=1.3,
                nav_timeout_sweepable=45
            )
        elif scenario_type == "school":
            self.update_navigation(
                tolerance_sweepable=1.0,
                tolerance_graspable=1.1,
                nav_timeout_sweepable=40
            )
        elif scenario_type == "hospital":
            self.update_navigation(
                tolerance_sweepable=0.8,
                tolerance_graspable=0.9,
                nav_timeout_sweepable=50
            )
        elif scenario_type == "factory":
            self.update_navigation(
                tolerance_sweepable=1.5,
                tolerance_graspable=1.6,
                nav_timeout_sweepable=60
            )
    
    def print_summary(self):
        """打印OSGT配置摘要"""
        print("\n" + "="*70)
        print("📋 OSGT四类物体清洁系统配置摘要")
        print("="*70)
        print(f"👤 用户: {self.USERNAME}")
        print(f"🏢 场景类型: {self.SCENARIO_TYPE}")
        print(f"🏠 住宅资产库: {self.PATHS['residential_assets_root']}")
        print(f"🤖 机器人模型: {self.PATHS['robot_usd_path']}")
        print(f"🔧 Isaac Sim: {self.USER_PATHS['isaac_sim_install']}")
        
        if hasattr(self, '_path_validation_results'):
            print("📂 路径验证结果:")
            for key, status in self._path_validation_results.items():
                print(f"   - {key}: {status}")
        
        print(f"📏 OSGT缩放配置:")
        for key, value in self.SCALE_CONFIG.items():
            print(f"   - {key}: {value}")
        
        print(f"🚧 O类障碍物数量: {len(self.OBSTACLES_POSITIONS)}")
        print(f"🧹 S类可清扫物数量: {len(self.SWEEPABLE_POSITIONS)}")
        print(f"🦾 G类可抓取物数量: {len(self.GRASPABLE_POSITIONS)}")
        print(f"🎯 T类任务区数量: {len(self.TASK_AREAS_POSITIONS)}")
        
        print(f"🚀 最大线速度: {self.ROBOT_CONTROL['max_linear_velocity']} m/s")
        print(f"🌀 最大角速度: {self.ROBOT_CONTROL['max_angular_velocity']} rad/s")
        print(f"🎯 OSGT导航容差: S类 {self.NAVIGATION['tolerance_sweepable']}m, G类 {self.NAVIGATION['tolerance_graspable']}m")
        print(f"⏱️ OSGT导航超时: S类 {self.NAVIGATION['nav_timeout_sweepable']}s, G类 {self.NAVIGATION['nav_timeout_graspable']}s")
        print("="*70)

# ==================== OSGT快速配置预设 ====================

class OSGTQuickConfigs:
    """OSGT快速配置预设（场景适配版）"""
    
    @staticmethod
    def residential_scene(username=None):
        """家庭住宅场景配置"""
        config = OSGTCleanupSystemConfig(username, "residential")
        
        # 家庭场景：减少障碍物密度，增加舒适度
        config.OBSTACLES_POSITIONS = {
            "living_table": [300.0, 150.0, 0.0, 0.0],      # 客厅茶几
            "dining_chair": [280.0, 80.0, 0.0, 0.0],       # 餐椅
            "sofa": [-300.0, 200.0, 0.0, 0.0],             # 沙发
        }
        
        config.SWEEPABLE_POSITIONS = {
            "food_crumb": [200.0, 100.0, 0.03],            # 食物碎渣
            "dust_ball": [300.0, -100.0, 0.03],            # 灰尘团
            "paper_scrap": [-200.0, 160.0, 0.01],          # 纸屑
        }
        
        config.GRASPABLE_POSITIONS = {
            "remote_control": [360.0, 240.0, 0.05],        # 遥控器
            "toy": [-240.0, -200.0, 0.05],                 # 玩具
        }
        
        return config
    
    @staticmethod
    def school_scene(username=None):
        """学校场景配置"""
        config = OSGTCleanupSystemConfig(username, "school")
        
        # 学校场景：课桌椅密度高，教具分散
        config.OBSTACLES_POSITIONS = {
            "desk_1": [200.0, 100.0, 0.0, 0.0],            # 课桌1
            "desk_2": [400.0, 100.0, 0.0, 0.0],            # 课桌2
            "chair_1": [180.0, 80.0, 0.0, 0.0],            # 椅子1
            "chair_2": [380.0, 80.0, 0.0, 0.0],            # 椅子2
            "blackboard": [-400.0, 0.0, 0.0, 0.0],         # 黑板
        }
        
        config.SWEEPABLE_POSITIONS = {
            "chalk_dust": [150.0, 50.0, 0.03],             # 粉笔灰
            "paper_ball": [350.0, 50.0, 0.03],             # 纸团
            "eraser_bit": [-200.0, 80.0, 0.01],            # 橡皮屑
        }
        
        config.GRASPABLE_POSITIONS = {
            "textbook": [190.0, 120.0, 0.05],              # 教科书
            "pencil_case": [390.0, 120.0, 0.05],           # 文具盒
            "lab_equipment": [-180.0, -100.0, 0.05],       # 实验器材
        }
        
        return config
    
    @staticmethod
    def hospital_scene(username=None):
        """医院场景配置"""
        config = OSGTCleanupSystemConfig(username, "hospital")
        
        # 医院场景：洁污分区，无菌要求
        config.OBSTACLES_POSITIONS = {
            "hospital_bed": [300.0, 200.0, 0.0, 0.0],      # 病床
            "medical_cart": [100.0, 100.0, 0.0, 0.0],      # 医疗推车
            "monitor": [320.0, 180.0, 0.0, 0.0],           # 监护仪
        }
        
        config.SWEEPABLE_POSITIONS = {
            "medical_waste": [250.0, 150.0, 0.03],         # 医疗废料
            "cotton_ball": [350.0, 150.0, 0.03],           # 棉球
            "packaging": [-200.0, 100.0, 0.01],            # 包装废料
        }
        
        config.GRASPABLE_POSITIONS = {
            "medicine_bottle": [280.0, 220.0, 0.05],       # 药瓶
            "medical_chart": [120.0, 120.0, 0.05],         # 病历夹
            "syringe": [-150.0, -80.0, 0.05],              # 注射器
        }
        
        # 医院场景需要更严格的容差
        config.update_navigation(
            tolerance_sweepable=0.8,
            tolerance_graspable=0.9
        )
        
        return config
    
    @staticmethod
    def factory_scene(username=None):
        """工厂场景配置"""
        config = OSGTCleanupSystemConfig(username, "factory")
        
        # 工厂场景：设备密度极高，重型物品
        config.OBSTACLES_POSITIONS = {
            "machine_1": [400.0, 300.0, 0.0, 0.0],         # 生产设备1
            "machine_2": [400.0, -300.0, 0.0, 0.0],        # 生产设备2
            "conveyor": [0.0, 200.0, 0.0, 90.0],           # 传送带
            "storage_rack": [-400.0, 0.0, 0.0, 0.0],       # 货架
            "agv_station": [200.0, -200.0, 0.0, 45.0],     # AGV站点
        }
        
        config.SWEEPABLE_POSITIONS = {
            "metal_chip": [350.0, 250.0, 0.03],            # 金属碎屑
            "plastic_bead": [450.0, 250.0, 0.03],          # 塑料颗粒
            "oil_spot": [50.0, 180.0, 0.01],               # 油污
            "dust": [-350.0, 50.0, 0.03],                  # 工业粉尘
        }
        
        config.GRASPABLE_POSITIONS = {
            "component": [380.0, 280.0, 0.05],             # 零部件
            "tool": [420.0, 280.0, 0.05],                  # 工具
            "packaging_box": [180.0, -180.0, 0.05],        # 包装箱
            "spare_part": [-180.0, -50.0, 0.05],           # 备件
        }
        
        # 工厂场景需要更大的容差和更长的超时
        config.update_navigation(
            tolerance_sweepable=1.5,
            tolerance_graspable=1.6,
            nav_timeout_sweepable=60,
            nav_timeout_graspable=70
        )
        
        return config
    
    @staticmethod
    def debug_mode(username=None, scenario_type="residential"):
        """OSGT调试模式配置"""
        config = OSGTCleanupSystemConfig(username, scenario_type)
        
        config.DEBUG.update({
            "enable_debug_output": True,
            "show_robot_state": True,
            "show_navigation_progress": True,
            "show_grasp_details": True,
            "progress_report_interval": 1.0,
        })
        
        config.ROBOT_CONTROL.update({
            "max_linear_velocity": 0.4,
            "max_angular_velocity": 1.5,
        })
        
        return config

# ==================== 使用示例 ====================

def example_usage():
    """OSGT配置文件使用示例"""
    
    # 1. 使用默认配置（家庭住宅场景）
    config = OSGTCleanupSystemConfig()
    
    # 2. 指定场景类型
    # config = OSGTCleanupSystemConfig(username="your_username", scenario_type="hospital")
    
    # 3. 使用快速预设
    # config = OSGTQuickConfigs.residential_scene("your_username")
    # config = OSGTQuickConfigs.school_scene("your_username")
    # config = OSGTQuickConfigs.hospital_scene("your_username")
    # config = OSGTQuickConfigs.factory_scene("your_username")
    
    # 4. 修改OSGT缩放比例
    config.update_scale(obstacles=0.02, sweepable_items=0.02)
    
    # 5. 添加新的OSGT物体位置
    config.add_obstacle_position("new_machine", 500.0, 500.0, 0.0, 45.0)
    config.add_sweepable_position("new_debris", 100.0, 360.0, 0.02)
    config.add_graspable_position("new_tool", 300.0, 500.0, 0.05)
    config.add_task_area_position("new_station", 600.0, 600.0, 0.0, 0.0)
    
    # 6. 切换场景类型
    # config.set_scenario_type("factory")
    
    # 7. 调整机器人参数
    config.update_robot_control(max_linear_velocity=0.6, max_angular_velocity=2.2)
    
    # 8. 调整OSGT导航参数
    config.update_navigation(tolerance_sweepable=0.8, nav_timeout_sweepable=35)
    
    # 9. 打印配置摘要
    config.print_summary()
    
    return config

if __name__ == "__main__":
    # 测试OSGT配置
    config = example_usage()