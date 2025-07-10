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

    def __init__(self,username=None, scenario_type ="lobby"):
        # ==================== 用户配置 ====================
        if username is None:
            username = (
                os.environ.get('USER') or           # Linux/macOS
                os.environ.get('USERNAME') or       # Windows
                os.environ.get('LOGNAME') or        # 备用
                'user'                              # 默认值
            )
        
        self.USERNAME = username
        self.SCENARIO_TYPE = scenario_type  # residential, school, hospital
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
        
        # 根据场景类型配置完整的背景场景参数
        background_scene_mapping = {
            "lobby": {
                "usd_path": "My_asset/background/Lobby.usd",
                "scale": 0.02,
                "position": [-40, -50, 0.0],
                "rotation_z": 0.0
            },
            "lobby_collision": {
                "usd_path": "My_asset/background/Lobby_collision.usd",
                "scale": 0.02,
                "position": [-40, -50, 0.0],
                "rotation_z": 0.0
            },
            "office": {
                "usd_path": "My_asset/background/Office.usd",
                "scale": 1,
                "position": [0.0, 0.0, 0.0],
                "rotation_z": 0.0
            },
            "office_collision": {
                "usd_path": "My_asset/background/Office_collision.usd",
                "scale": 1,
                "position": [0.0, 0.0, 0.0],
                "rotation_z": 0.0
            },
            "hospital": {
                "usd_path": "My_asset/background/Hospital.usd",
                "scale": 1,
                "position": [0.0, 0.0, 0.0],
                "rotation_z": 0.0
            },
            "hospital_collision": {
                "usd_path": "My_asset/background/Hospital_collision.usd",
                "scale": 1,
                "position": [0.0, 0.0, 0.0],
                "rotation_z": 0.0
            },
            "kitchen": {
                "usd_path": "My_asset/background/Kitchen.usd",
                "scale": 0.02,
                "position": [0.0, 0.0, 0.0],
                "rotation_z": 0.0
            },
            "kitchen_collision": {
                "usd_path": "My_asset/background/Kitchen_collision.usd",
                "scale": 0.02,
                "position": [0.0, 0.0, 0.0],
                "rotation_z": 0.0
            },
            "restaurant": {
                "usd_path": "My_asset/background/Restaurant.usd",
                "scale": 0.01,
                "position": [-4.26926, 11.01489, 0.0],
                "rotation_z": 270.0
            },
            "restaurant_collision": {
                "usd_path": "My_asset/background/Restaurant_collision.usd",
                "scale": 0.01,
                "position": [-4.26926, 11.01489, 0.0],
                "rotation_z": 270.0
            },
            "isaacWarehouse": {
                "usd_path": "My_asset/background/IsaacWarehouse.usd",
                'scale': 0.01,
                "position": [0.0, 0.0, 0.0],
                "rotation_z": 0.0
            },
            # 其他场景类型默认使用Lobby配置
            "residential": {
                "usd_path": "My_asset/background/Lobby.usd",
                "scale": 0.02,
                "position": [-40, -50, 0.0],
                "rotation_z": 0.0
            },
            "school": {
                "usd_path": "My_asset/background/Office.usd",
                "scale": 0.025,
                "position": [-35, -45, 0.0],
                "rotation_z": 45.0
            },
            "factory": {
                "usd_path": "My_asset/background/Office.usd",
                "scale": 0.03,
                "position": [-60, -70, 0.0],
                "rotation_z": 0.0
            }
        }
        
        # 获取当前场景类型的完整配置，如果没有找到则使用lobby的默认配置
        selected_background_config = background_scene_mapping.get(
            self.SCENARIO_TYPE, 
            background_scene_mapping["lobby"]
        )
        
        self.BACKGROUND_ENVIRONMENT = {
            # 场景usd文件路径（根据scenario_type自动选择）
            "usd_path": selected_background_config["usd_path"],
            # 缩放比例（根据场景优化）
            "scale": selected_background_config["scale"],
            # 位置 [x, y, z]（根据场景调整）
            "position": selected_background_config["position"],
            # 旋转（绕z轴，单位度）（根据场景方向优化）
            "rotation_z": selected_background_config["rotation_z"]
        }
        # 自动检测资产路径
        self._detect_asset_paths()
        
        # 构建最终路径
        self.PATHS = {
            "residential_assets_root": os.path.join(
                self.USER_PATHS["isaac_assets_base"], 
                "NVIDIA/Assets/ArchVis/Lobby"
            ),
            "robot_usd_path": os.path.join(
                self.USER_PATHS["isaac_assets_base"], 
                "Isaac/Robots/iRobot/create_3_with_arm_lightbeam.usd"
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
            "graspable_items": 0.01,     # 1% (可抓取物：工具、容器等)
            
            # T类 - 任务区缩放
            "task_areas": 1,          # 2% (回收区、存放区等)

            # 全局缩放
            "global_scale": 1.0,         # 全局缩放倍数
        }
        
        # ==================== OSGT四类物体位置配置 ====================
        # 根据场景类型配置不同的OSGT四类物体位置
        scenario_positions_mapping = {
            "lobby": {
                # Lobby环境边界: [1348,153,0.08]  [-2197,153,0.08] 
                #               [1348,-1985,0.08] [-370,-1960,0.08] [-370,-1339,0.08]  [-2128,-1339,0.08]
                "obstacles": {
                    # "chair_b1": [450.0, 80.0, 0.0, 0.0],
                    # "dining_table": [200.0, -600.0, 0.0, 0.0],
                    # "folding_table": [-1200.0, 120.0, 0.0, 90.0],
                    # "kitchen_table": [800.0, -1200.0, 0.0, 0.0],
                    # "stool_wooden": [-500.0, -800.0, 0.0, 0.0],
                    # "book_stack_01": [1000.0, -400.0, 0.0, 0.0],
                    # "encyclopedia": [-1500.0, -200.0, 0.0, 0.0]
                },
                "sweepable": {
                    "bubble_marble_02": [300.0, 100.0, 0.05],
                    "caster_bearing": [-600.0, -500.0, 0.05],
                    "cheerio_geom": [700.0, -800.0, 0.05],
                    "d20_01": [-1000.0, 50.0, 0.08],
                    "metalballs": [150.0, -1400.0, 0.03],
                    "plasticballs": [-1800.0, -600.0, 0.03]
                },
                "graspable": {
                    "mechanical_pencil": [-400.0, 120.0, 0.1],
                    "cup": [600.0, -300.0, 0.05],
                    "bottle": [-1100.0, -1000.0, 0.1],
                    "fork": [900.0, 80.0, 0.1],
                    "ball": [-200.0, -1200.0, 0.05],
                    "salt_shaker": [1200.0, -700.0, 0.08]
                },
                "task_areas": {
                    "trash_can": [400.0, -1500.0, 0.0, 0.0],
                    "recycling_bin": [-800.0, 100.0, 0.0, 45.0],
                    "storage_box": [-1600.0, -1200.0, 0.0, 0.0]
                }
            },
            "office": {
                # 办公室环境
                "obstacles": {
                    # "chair_b1": [2.0, 1.0, 0.0, 0.0],
                    # "dining_table": [1.0, -3.0, 0.0, 0.0],
                    # "folding_table": [-6.0, 0.5, 0.0, 90.0],
                    # "kitchen_table": [4.0, -6.0, 0.0, 0.0],
                    # "stool_wooden": [-2.5, -4.0, 0.0, 0.0],
                    # "book_stack_01": [5.0, -2.0, 0.0, 0.0],
                    # "encyclopedia": [-7.5, -1.0, 0.0, 0.0]
                },
                "sweepable": {
                    "d20_01": [-1583.0, 917.0, 0.08],
                    "metalballs": [447.0, -715.0, 0.03],
                    "plasticballs": [-1375, 333.0, 0.03]
                },
                "graspable": {
                    "mechanical_pencil": [-1375, 372, 0.1],
                    "bottle": [-904, -5.0, 0.1],
                    "salt_shaker": [412, 905, 0.08]
                },
                "task_areas": {
                    "trash_can": [-1291, 1133, -0.2, 0.0],
                }
            },
            "hospital": {
                # 医院环境
                # [1893,91,0] [1893,949,0] [-3237,949,0] [-3237,269,0] 
                # X轴范围: -3237 到 1893, Y轴范围: 91 到 949
                "obstacles": {
                    # "chair_b1": [3.0, 2.0, 0.0, 0.0],
                    # "dining_table": [1.5, -4.0, 0.0, 0.0],
                    # "folding_table": [-8.0, 1.0, 0.0, 0.0],
                    # "kitchen_table": [6.0, -8.0, 0.0, 0.0],
                    # "stool_wooden": [-3.0, -5.0, 0.0, 0.0],
                    # "book_stack_01": [7.0, -3.0, 0.0, 0.0],
                    # "encyclopedia": [-10.0, -1.5, 0.0, 0.0]
                },
                "sweepable": {
                    # S类 - 可清扫物位置配置 (3个物体在医院边界内随机分布)
                    "bubble_marble_02": [450.0, 720.0, 0.05],      # 气泡弹珠2
                    "metalballs": [-1800.0, 380.0, 0.03],          # 金属球
                    "plasticballs": [-2500.0, 850.0, 0.03],       # 塑料球
                },
                "graspable": {
                    # G类 - 可抓取物位置配置 (3个物体在医院边界内随机分布)
                    "mechanical_pencil": [1200.0, 600.0, 0.1],    # 机械铅笔
                    "bottle": [-1000.0, 280.0, 0.1],              # 瓶子
                    "salt_shaker": [800.0, 900.0, 0.08],          # 盐瓶
                },
                "task_areas": {
                    # T类 - 任务区位置配置 (1个任务区在医院边界内)
                    "trash_can": [-2555.0, 460.0, 0.0, 0.0],      # 垃圾桶
                }
            },
            "kitchen": {
                # 厨房环境 - 功能区域布局
                # 环境边界:
                # [693,94,0],[693,-482,0]
                # [-199,-482,0],[-199,-33,0]
                "obstacles": {
                    # "chair_b1": [1.0, 0.5, 0.0, 0.0],
                    # "dining_table": [0.5, -1.5, 0.0, 0.0],
                    # "folding_table": [-3.0, 0.3, 0.0, 90.0],
                    # "stool_wooden": [-1.25, -2.0, 0.0, 0.0],
                    # "book_stack_01": [2.5, -1.0, 0.0, 0.0],
                    # "encyclopedia": [-3.75, -0.5, 0.0, 0.0]
                },
                "sweepable": {
                    # S类 - 可清扫物位置配置 (4个物体在厨房边界内随机分布)
                    "bubble_marble_02": [120.0, 30.0, 0.05],      # 气泡弹珠2
                    "caster_bearing": [450.0, -200.0, 0.05],      # 脚轮轴承
                    "cheerio_geom": [580.0, -350.0, 0.05],        # 小圆环
                    "d20_01": [-80.0, -120.0, 0.08],              # 20面骰子
                },
                "graspable": {
                    # G类 - 可抓取物位置配置 (4个物体在厨房边界内随机分布)
                    "mechanical_pencil": [200.0, 60.0, 0.1],      # 机械铅笔
                    "cup": [500.0, -150.0, 0.05],                 # 杯子
                    "bottle": [350.0, -400.0, 0.1],               # 瓶子
                    "fork": [-120.0, -250.0, 0.1],                # 叉子
                },
                "task_areas": {
                    "trash_can": [600.0, -450.0, 0.0, 0.0],       # 垃圾桶(靠近边界)
                }
            },
            "restaurant": {
                # 餐厅环境 - 用餐区域布局
                # 环境边界: [-441,903,0] [2187,903,0] [2187,221,0] [523,114,0]
                #  [523,-2413,0] [2133,-1627,0]  [2105,-2418,0]
                # X轴范围: -441 到 2187, Y轴范围: -2418 到 903
                "obstacles": {
                    # "chair_b1": [2.5, 1.5, 0.0, 0.0],
                    # "dining_table": [1.0, -3.5, 0.0, 0.0],
                    # "folding_table": [-7.0, 0.8, 0.0, 0.0],
                    # "kitchen_table": [5.0, -7.0, 0.0, 0.0],
                    # "stool_wooden": [-2.0, -4.5, 0.0, 0.0],
                    # "book_stack_01": [6.0, -2.5, 0.0, 0.0],
                    # "encyclopedia": [-9.0, -1.0, 0.0, 0.0]
                },
                "sweepable": {
                    # S类 - 可清扫物位置配置 (6个物体在餐厅边界内随机分布)
                    "bubble_marble_02": [1200.0, 700.0, 0.05],       # 气泡弹珠2
                    "caster_bearing": [-200.0, 400.0, 0.05],         # 脚轮轴承
                    "metalballs": [1500.0, -1800.0, 0.03],           # 金属球
                    "plasticballs": [-300.0, 800.0, 0.03],           # 塑料球
                },
                "graspable": {
                    # G类 - 可抓取物位置配置 (6个物体在餐厅边界内随机分布)
                    "mechanical_pencil": [1000.0, 500.0, 0.1],       # 机械铅笔
                    "bottle": [1900.0, 482.0, 0.1],                  # 瓶子
                    "salt_shaker": [2000.0, -2000.0, 0.08],          # 盐瓶
                },
                "task_areas": {
                    # T类 - 任务区位置配置 (3个任务区在餐厅边界内随机分布)
                    "trash_can": [2100.0, -2300.0, 0.0, 0.0],        # 垃圾桶
                    "recycling_bin": [-350.0, 850.0, 0.0, 45.0],     # 回收箱
                }
                
            },
            "isaacWarehouse": {
                # 仓库环境 - 用于存储和分拣
                # 环境边界: [-441,903,0] [2187,903,0] [2187,221,0] [523,114,0]
                #  [523,-2413,0] [2133,-1627,0]  [2105,-2418,0]
                # X轴范围: -441 到 2187, Y轴范围: -2418 到 903
                "obstacles": {
                    "carter_v1_physx_lidar":[840, 2180.0, 0.27, 0.0], # Carter机器人
            
                },
                "sweepable": {
                    # S类 - 可清扫物位置配置 (6个物体在餐厅边界内随机分布)
                    "bubble_marble_02": [1200.0, 700.0, 0],       # 气泡弹珠2
                    "metalballs": [1500.0, -1800.0, 0.03],           # 金属球
                    "plasticballs": [-300.0, 800.0, 0],           # 塑料球
                    "CHARGING_BEAM_KFDM": [921.0, -1960.0, -0.25],           # 充电束KFDM
                    "DOCKING_V_KF7L": [482.0, -2511.0, -0.8],           # 充电束KFDM
                },
                "graspable": {
                    # G类 - 可抓取物位置配置 (6个物体在餐厅边界内随机分布)
                    "mechanical_pencil": [1000.0, 500.0, 0.01],       # 机械铅笔
                    "ISO7380": [985.0, 590.0, 1.05],          # ISO7380
                    "_51_large_clamp": [1200.0, -1090.0, 1],  # 大夹具
                    "M5_LOCKNUT__JFn": [987.0, 569.0, 1.05],  # M5锁紧螺母
                    "_35_power_drill": [950.0, 544.0, 1.07, 0.0],     # 35功率drill
                },
                "task_areas": {
                    # T类 - 任务区位置配置 (3个任务区在餐厅边界内随机分布)
                    
                    
                    "small_KLT": [1266.0, -1418.0, 0.235, 0.07],  # 小KLT
                }
            },
            # 其他场景类型使用lobby的配置作为默认值
            "residential": "lobby",
            "school": "office", 
            "factory": "office"
        }
        
        # 获取当前场景类型的位置配置
        selected_positions = scenario_positions_mapping.get(self.SCENARIO_TYPE, "lobby")
        # 如果是字符串引用，则使用引用的配置
        if isinstance(selected_positions, str):
            selected_positions = scenario_positions_mapping[selected_positions]
        
        # O类 - 障碍物位置配置 (Obstacles)
        self.OBSTACLES_POSITIONS = selected_positions["obstacles"]
        
        # S类 - 可清扫物位置配置 (Sweepable Items)
        self.SWEEPABLE_POSITIONS = selected_positions["sweepable"]
        
        # G类 - 可抓取物位置配置 (Graspable Items)
        self.GRASPABLE_POSITIONS = selected_positions["graspable"]
        
        # T类 - 任务区位置配置 (Task Areas)
        self.TASK_AREAS_POSITIONS = selected_positions["task_areas"]
        
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
            # O类 - 障碍物配置 (办公环境障碍物)
            "obstacles": {
                "book_11": "My_asset/O/Book_11.usd",
                "book_stack_01": "My_asset/O/BookStack_01.usd",
                "book_stack_02": "My_asset/O/BookStack_02.usd",
                "chair_b1": "My_asset/O/ChairB_1.usd",
                "dining_table": "My_asset/O/DiningTable_grp.usd",
                "encyclopedia": "My_asset/O/Encyclopedia01.usd",
                "folding_table": "My_asset/O/FoldingTable_grp.usd",
                "fridge_area": "My_asset/O/FridgeArea_grp.usd",
                "iron_board": "My_asset/O/IronBoard_1.usd",
                "kitchen_table": "My_asset/O/KitchenTable_1.usd",
                "paper_bag_crumpled": "My_asset/O/PaperBagCrumpled_1.usd",
                "stool_metal_wire": "My_asset/O/StoolMetalWire_1.usd",
                "stool_wooden": "My_asset/O/StoolWooden_1.usd",
                "stove_area": "My_asset/O/StoveArea_grp.usd",
                "jar": "My_asset/G/assets/Jar/Jar.usd",
                "carter_v1_physx_lidar": "My_asset/O/carter_v1_physx_lidar.usd",
            },
            
            # S类 - 可清扫物配置 (小颗粒吸附收集)
            "sweepable_items": {
                "bubble_marble_02": "My_asset/S/BubbleMarble_02.usd",
                "bubble_marble_03": "My_asset/S/BubbleMarble_03.usd",
                "caster_bearing": "My_asset/S/caster_bearing.usd",
                "cheerio_geom": "My_asset/S/Cheerio_geom.usd",
                "d20_01": "My_asset/S/D20_01.usd",
                "metalballs": "My_asset/S/Metalballs.usd",
                "plasticballs": "My_asset/S/Plasticballs.usd",
                "solid_marble_01": "My_asset/S/Solid_Marble_01.usd",
                "CHARGING_BEAM_KFDM": "My_asset/S/CHARGING_BEAM_KFDM.usd",
                "DOCKING_V_KF7L": "My_asset/S/DOCKING_V_KF7L.usd",
            },
            
            # G类 - 可抓取物配置 (马克笔到小水瓶大小的物体)
            "graspable_items": {
                # 文具类 (马克笔大小)
                "mechanical_pencil": "My_asset/G/Supplies/MechanicalPencil.usd",
                "eraser": "My_asset/G/Supplies/Eraser.usd",
                "makerpen": "My_asset/G/Makerpen.usd",
                "crayon": "My_asset/G/assets/Crayon/Crayon.usd",
                
                # 餐具类 (中等大小)
                "fork": "My_asset/G/assets/Fork/Fork.usd",
                "knife": "My_asset/G/assets/Knife/Knife.usd",
                "spoon": "My_asset/G/assets/Spoon/Spoon.usd",
                "wooden_spoon": "My_asset/G/assets/WoodenSpoon/WoodenSpoon.usd",
                "spatula": "My_asset/G/assets/Spatula/Spatula.usd",
                "whisk": "My_asset/G/assets/Whisk/Whisk.usd",
                "rolling_pin": "My_asset/G/assets/RollingPin/RollingPin.usd",
                
                # 容器类 (小到中等大小)
                "cup": "My_asset/G/assets/Cup/Cup.usd",
                
                "tin_can": "My_asset/G/Containers/TinCan.usd",
                "mason_jar": "My_asset/G/Containers/MasonJar.usd",
                "oil_bottle": "My_asset/G/assets/OilBottle/OilBottle.usd",
                "bottle": "My_asset/G/assets/Bottle/Bottle.usd",
                "bottle_b": "My_asset/G/assets/BottleB/BottleB.usd",
                "salt_shaker": "My_asset/G/assets/SaltShaker/SaltShaker.usd",
                "spice_shaker": "My_asset/G/assets/SpiceShaker/SpiceShaker.usd",
                "measuring_cup": "My_asset/G/assets/MeasuringCup/MeasuringCup.usd",
                "measuring_spoon": "My_asset/G/assets/MeasuringSpoon/MeasuringSpoon.usd",
                
                # 小物件类
                "ball": "My_asset/G/assets/Ball/Ball.usd",
                "ball_walnut": "My_asset/G/Ball_Walnut_01.usd",
                "clock": "My_asset/G/assets/Clock/Clock.usd",
                "soap_dispenser": "My_asset/G/assets/SoapDispenser/SoapDispenser.usd",
                "soap_sponge": "My_asset/G/assets/SoapSponge/SoapSponge.usd",
                "hand_towel": "My_asset/G/assets/HandTowel/HandTowel.usd",
                
                
                # 小工具类
                "nail": "My_asset/G/assets/Nail/Nail.usd",
                "hook": "My_asset/G/assets/Hook/Hook.usd",
                "hanger": "My_asset/G/assets/Hanger/Hanger.usd",
                "ISO7380": "My_asset/G/Tools/ISO7380.usd",
                "_35_power_drill": "My_asset/G/Tools/_35_power_drill.usd",
                "_51_large_clamp": "My_asset/G/Tools/_51_large_clamp.usd",
                "M5_LOCKNUT__JFn": "My_asset/G/Tools/M5_LOCKNUT__JFn.usd",
            },
            
            # T类 - 任务区配置 (功能区域标识)
            "task_areas": {
                "trash_can": "My_asset/T/trash_can.usd",
                "recycling_bin": "My_asset/T/trash_can.usd",     # 复用垃圾桶资产作为回收箱
                "storage_box": "My_asset/T/trash_can.usd",       # 复用垃圾桶资产作为储物箱
                "small_KLT": "My_asset/T/small_KLT.usd",
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
        elif scenario_type == "lobby":
            self.update_navigation(
                tolerance_sweepable=1.0,
                tolerance_graspable=1.0,
                nav_timeout_sweepable=45
            )
        elif scenario_type == "office":
            self.update_navigation(
                tolerance_sweepable=0.9,
                tolerance_graspable=1.0,
                nav_timeout_sweepable=40
            )
        elif scenario_type == "kitchen":
            self.update_navigation(
                tolerance_sweepable=0.7,
                tolerance_graspable=0.8,
                nav_timeout_sweepable=35
            )
        elif scenario_type == "restaurant":
            self.update_navigation(
                tolerance_sweepable=1.1,
                tolerance_graspable=1.2,
                nav_timeout_sweepable=50
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
        
        print(f"🏠 背景场景配置:")
        print(f"   - 文件路径: {self.BACKGROUND_ENVIRONMENT['usd_path']}")
        print(f"   - 缩放比例: {self.BACKGROUND_ENVIRONMENT['scale']}")
        print(f"   - 位置坐标: {self.BACKGROUND_ENVIRONMENT['position']}")
        print(f"   - 旋转角度: {self.BACKGROUND_ENVIRONMENT['rotation_z']}°")
        
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
        