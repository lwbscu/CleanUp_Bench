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

    def __init__(self, username=None, scenario_type="restaurant"):
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
        
        self.BACKGROUND_ENVIRONMENT = {
            # 场景usd文件路径（相对Restaurant库）
            "usd_path": "My_asset/background/Restaurant.usd",
            # 缩放比例
            "scale": 0.02,
            # 位置 [x, y, z]
            "position": [0.0, 0.0, 0.0],
            # 旋转（绕z轴，单位度）
            "rotation_z": 0.0
        }
        # 自动检测资产路径
        self._detect_asset_paths()
        
        # 构建最终路径
        self.PATHS = {
            "residential_assets_root": os.path.join(
                self.USER_PATHS["isaac_assets_base"], 
                "NVIDIA/Assets/ArchVis/Restaurant"
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
            # "obstacle_1": [150.0, 80.0, 0.0, 0.0],      # 主要工作台/桌面
            # "obstacle_2": [140.0, 60.0, 0.0, 0.0],      # 座椅/推车
            # "obstacle_3": [-200.0, 180.0, 0.0, 0.0],    # 中央设施
            # "obstacle_4": [350.0, -280.0, 0.0, 45.0],   # 边角设备
            # "obstacle_5": [-450.0, -150.0, 0.0, 90.0],  # 存储设施
            # "obstacle_6": [-380.0, -420.0, 0.0, 0.0],   # 大型设备/书架
        }
        
        # S类 - 可清扫物位置配置 (Sweepable Items)
        self.SWEEPABLE_POSITIONS = {
            # 格式: "可清扫物名": [x, y, z]
            # 小颗粒物质：纸屑、食物碎渣、灰尘、金属屑等
            # "sweepable_1": [280.0, 150.0, 0.03],        # 工作区域碎渣
            # "sweepable_2": [520.0, -320.0, 0.03],       # 角落积尘
            # "sweepable_3": [-180.0, 450.0, 0.01],       # 地面碎片
            
            # "sweepable_5": [667.0, 80.0, 0.015],     
            
            # "sweepable_7": [-424, -580.0, 0.03],     
        }
        
        # G类 - 可抓取物位置配置 (Graspable Items)
        self.GRASPABLE_POSITIONS = {
            # 格式: "可抓取物名": [x, y, z]
            # 工具、容器、书籍、零件等需要机械臂抓取的物体
            # "orange2": [680.0, 165.0, 0.1],
            # "lemon2": [505.0, -266.0, 0.1],
            # "tin_can": [240.0, -75.0, 0.05],
            # "bottle": [650, -44, 0.1],
            # "cup": [332, 33, 0.1],
            # "fork": [-316, -391, 0.1],
        }
        
        # T类 - 任务区位置配置 (Task Areas)
        self.TASK_AREAS_POSITIONS = {
            # 格式: "任务区名": [x, y, z, rotation_z_degrees]
            # 回收区、分拣区、存放区等
            # "collection_zone_s": [800.0, 800.0, 0.0, 0.0],     # S类回收区
            # "collection_zone_g": [-800.0, 800.0, 0.0, 0.0],    # G类存放区
            # "sorting_area": [0.0, 900.0, 0.0, 0.0],             # 分拣中心
            # "maintenance_station": [0.0, -900.0, 0.0, 0.0],     # 维护站点
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
            "tolerance_sweepable": 0.5,         # S类可清扫物导航容差 (m)
            "tolerance_graspable": 0.5,         # G类可抓取物导航容差 (m)
            "tolerance_task_areas": 0.8,        # T类任务区容差 (m)
            
            # 导航超时配置
            "nav_timeout_sweepable": 45,         # S类导航超时 (s)
            "nav_timeout_graspable": 50,         # G类导航超时 (s)
            "nav_timeout_task_areas": 60,        # T类导航超时 (s)
            
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
                "chair_b1": "My_asset/O/ChairB_1.usd",
                "chair_b2": "My_asset/O/ChairB_2.usd",
                "dining_table": "My_asset/O/DiningTable_grp.usd",
                "folding_table": "My_asset/O/FoldingTable_grp.usd",
                "fridge_area": "My_asset/O/FridgeArea_grp.usd",
                "iron_board": "My_asset/O/IronBoard_1.usd",
                "kitchen_table": "My_asset/O/KitchenTable_1.usd",
                "stool_metal": "My_asset/O/StoolMetalWire_1.usd",
                "stool_wooden": "My_asset/O/StoolWooden_1.usd",
                "stove_area": "My_asset/O/StoveArea_grp.usd",
            },
            
            # S类 - 可清扫物配置 (小颗粒吸附收集)
            "sweepable_items": {
                "paper_crumpled": "My_asset/O/PaperBagCrumpled_1.usd",  # 临时使用O类中的纸质物品
                "cheerio_small": "My_asset/G/assets/Cheerio/Cheerio.usd",  # 小颗粒物
                "crayon_piece": "My_asset/G/assets/Crayon/Crayon.usd",    # 蜡笔碎片
                "paper_small": "My_asset/G/assets/PaperSmall/PaperSmall.usd",  # 纸屑
                "nail_small": "My_asset/G/assets/Nail/Nail.usd",          # 小钉子
            },
            
            # G类 - 可抓取物配置 (机械臂精确抓取)
            "graspable_items": {
                "book": "My_asset/G/assets/Book/Book.usd",
                "bottle": "My_asset/G/assets/Bottle/Bottle.usd",
                "bowl": "My_asset/G/assets/Bowl/Bowl.usd",
                "cup": "My_asset/G/assets/Cup/Cup.usd",
                "fork": "My_asset/G/assets/Fork/Fork.usd",
                "knife": "My_asset/G/assets/Knife/Knife.usd",
                "spoon": "My_asset/G/assets/Spoon/Spoon.usd",
                "plate": "My_asset/G/assets/Plate/Plate.usd",
                "tin_can": "My_asset/G/Containers/TinCan.usd",
                "mason_jar": "My_asset/G/Containers/MasonJar.usd",
                "mechanical_pencil": "My_asset/G/Supplies/MechanicalPencil.usd",
                "eraser": "My_asset/G/Supplies/Eraser.usd",
                "sketchbook": "My_asset/G/Supplies/Sketchbook.usd",
                "chess_pawn": "My_asset/G/Games/ChessSet/Chess_Pawn.usd",
                "d20_die": "My_asset/G/Games/D20.usd",
                "rubix_cube": "My_asset/G/Games/RubixCube.usd",
                "book_stack": "My_asset/G/Books/BookStack_01.usd",
                "encyclopedia": "My_asset/G/Books/Encyclopedia01.usd",
            },
            
            # T类 - 任务区配置 (基础形状表示功能区)
            "task_areas": {
                "collection_zone_s": "My_asset/G/assets/FoldingTable/FoldingTable.usd",  # S类回收桌
                "collection_zone_g": "My_asset/G/assets/KitchenTable/KitchenTable.usd", # G类存放桌
                "sorting_station": "My_asset/G/assets/Chair/Chair.usd",                 # 分拣区域
                "maintenance_area": "My_asset/G/assets/StoolWooden/StoolWooden.usd",    # 维护站点
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
    
    # ==================== 背景场景配置方法 ====================
    
    def set_background_scene(self, usd_path, scale=1.0, position=None, rotation_z=0.0):
        """设置背景场景"""
        if position is None:
            position = [0.0, 0.0, 0.0]
        
        self.BACKGROUND_ENVIRONMENT.update({
            "usd_path": usd_path,
            "scale": scale,
            "position": position,
            "rotation_z": rotation_z,
        })
        
        print(f"🏠 设置背景场景: {usd_path}")
        print(f"   缩放: {scale}")
        print(f"   位置: {position}")
        print(f"   旋转: {rotation_z}°")
    
    def disable_background_scene(self):
        """禁用背景场景"""
        self.BACKGROUND_ENVIRONMENT["usd_path"] = ""
        print("🏠 背景场景已禁用")
    
    def enable_background_scene(self, usd_path=None):
        """启用背景场景"""
        if usd_path:
            self.BACKGROUND_ENVIRONMENT["usd_path"] = usd_path
        print(f"🏠 背景场景已启用: {self.BACKGROUND_ENVIRONMENT['usd_path']}")

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

