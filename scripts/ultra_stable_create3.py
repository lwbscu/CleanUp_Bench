#!/usr/bin/env python3
"""
CUDA加速优化版Create-3+机械臂垃圾收集系统（配置文件版）
使用config.py进行参数管理，支持多用户环境
集成高级导航系统，实现丝滑精准的移动控制
修正位置缩放问题
"""

from isaacsim import SimulationApp

# 先导入配置，然后初始化仿真
from config import CleanupSystemConfig, QuickConfigs
import os

# 获取用户名（支持多种方式）
username = (
    os.environ.get('CLEANUP_BENCH_USERNAME') or  # 从环境变量获取
    os.environ.get('USER') or                    # Linux/macOS
    os.environ.get('USERNAME') or                # Windows  
    os.environ.get('LOGNAME') or                 # 备用
    'user'                                       # 默认值
)

print(f"🔧 启动清洁系统，用户: {username}")

# 根据需要选择配置
config = CleanupSystemConfig(username)                    # 默认配置
# config = QuickConfigs.small_scene(username)              # 小场景配置
# config = QuickConfigs.tiny_furniture(username)           # 超小家具配置
# config = QuickConfigs.performance_optimized(username)    # 性能优化配置
# config = QuickConfigs.debug_mode(username)                 # 调试模式配置

# 修正坐标系统：将配置中的大坐标转换为合理的世界坐标
COORDINATE_SCALE = 0.01  # 将几百的坐标缩放到几米的世界坐标

# 使用配置初始化仿真应用
simulation_app = SimulationApp({
    "headless": False,
    "enable_livestream": False,
    "enable_cameras": True,
    "enable_rtx": True,
    "physics_dt": config.PHYSICS["physics_dt"],
    "rendering_dt": config.PHYSICS["rendering_dt"],
})

import numpy as np
import math
import time
import random
import os
from collections import deque
import heapq

# Isaac Sim API
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid, DynamicSphere
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdLux, UsdPhysics, Gf, Usd
import isaacsim.core.utils.prims as prim_utils

# 导入高级导航系统
from advanced_navigation import AdvancedNavigationSystem

class ConfigurableCreate3CleanupSystem:
    """基于配置文件的Create-3+机械臂室内清洁系统（位置修正版）"""
    
    def __init__(self, config):
        self.config = config
        self.world = None
        
        # 从配置文件读取路径
        self.robot_prim_path = config.PATHS["robot_prim_path"]
        self.residential_assets_root = config.PATHS["residential_assets_root"]
        self.robot_usd_path = config.PATHS["robot_usd_path"]
        
        if config.DEBUG["enable_debug_output"]:
            print(f"🔧 住宅资产库: {self.residential_assets_root}")
            print(f"🤖 机器人模型: {self.robot_usd_path}")
        
        # 机器人相关
        self.mobile_base = None
        self.differential_controller = None
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_orientation = 0.0
        
        # 从配置读取控制参数
        self.max_linear_velocity = config.ROBOT_CONTROL["max_linear_velocity"]
        self.max_angular_velocity = config.ROBOT_CONTROL["max_angular_velocity"]
        self.movement_threshold = config.ROBOT_CONTROL["movement_threshold"]
        self.angular_threshold = config.ROBOT_CONTROL["angular_threshold"]
        self.velocity_smoothing = config.ROBOT_CONTROL["velocity_smoothing"]
        
        # 轮子配置
        self.wheel_config = config.ROBOT_CONTROL["wheel_joint_names"]
        self.wheel_joint_indices = []
        
        # 机械臂配置
        self.arm_joint_names = config.ARM_CONFIG["joint_names"]
        self.gripper_joint_names = config.ARM_CONFIG["gripper_joint_names"]
        self.arm_poses = config.ARM_CONFIG["poses"]
        self.gripper_open = config.ARM_CONFIG["gripper_open"]
        self.gripper_closed = config.ARM_CONFIG["gripper_closed"]
        
        # 智能平滑控制
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # 垃圾收集相关
        self.small_trash_objects = []
        self.large_trash_objects = []
        self.collected_objects = []
        self.scene_objects = []
        
        # 高级导航系统
        self.advanced_navigation = None
        
        # 从配置读取导航参数（保留兼容性）
        self.grid_resolution = config.NAVIGATION["grid_resolution"]
        self.map_size = config.NAVIGATION["map_size"]
        self.stuck_threshold = config.NAVIGATION["stuck_threshold"]
        self.stuck_detection_window = config.NAVIGATION["stuck_detection_window"]
        self.obstacle_map = None
        
        # 导航优化
        self.navigation_history = deque(maxlen=50)
        
        # 性能监控
        self.performance_stats = {
            'movement_commands_sent': 0,
            'successful_movements': 0,
            'total_distance_traveled': 0.0,
            'total_navigation_time': 0.0
        }
    
    def get_asset_path(self, relative_path):
        """获取住宅资产的完整路径"""
        full_path = os.path.join(self.residential_assets_root, relative_path)
        if os.path.exists(full_path):
            return full_path
        else:
            if self.config.DEBUG["enable_debug_output"]:
                print(f"⚠️ 资产文件不存在: {full_path}")
            return relative_path
    
    def verify_assets(self):
        """验证所有必需的资产文件"""
        if self.config.DEBUG["enable_debug_output"]:
            print("🔍 验证资产文件...")
        
        # 验证机器人模型
        if os.path.exists(self.robot_usd_path):
            size_mb = os.path.getsize(self.robot_usd_path) / (1024*1024)
            if self.config.DEBUG["enable_debug_output"]:
                print(f"   ✅ 机器人模型: create_3_with_arm.usd ({size_mb:.1f} MB)")
        else:
            print(f"   ❌ 机器人模型缺失: {self.robot_usd_path}")
            return False
        
        # 验证住宅资产库
        if not os.path.exists(self.residential_assets_root):
            print(f"   ❌ 住宅资产库缺失: {self.residential_assets_root}")
            return False
        
        # 验证关键资产文件
        critical_assets = []
        for category, items in self.config.ASSET_PATHS.items():
            for name, relative_path in items.items():
                full_path = self.get_asset_path(relative_path)
                if os.path.exists(full_path):
                    size_kb = os.path.getsize(full_path) / 1024
                    scale = self.config.SCALE_CONFIG.get(category, 1.0)
                    critical_assets.append(f"   ✅ {name}: {size_kb:.1f} KB (缩放: {scale:.2f})")
                else:
                    print(f"   ❌ 缺失资产: {name} -> {relative_path}")
                    return False
        
        if self.config.DEBUG["enable_debug_output"]:
            print(f"✅ 资产验证通过，共 {len(critical_assets)} 个文件:")
            for asset in critical_assets[:5]:
                print(asset)
            if len(critical_assets) > 5:
                print(f"   ... 还有 {len(critical_assets) - 5} 个文件")
        
        return True
    
    def initialize_isaac_sim(self):
        """初始化Isaac Sim环境（CUDA优化）"""
        print("🚀 正在初始化Isaac Sim环境（高级导航+CUDA加速）...")
        
        try:
            # 验证资产文件
            if not self.verify_assets():
                print("❌ 资产验证失败，请检查文件路径")
                return False
            
            # 创建世界（使用配置的参数）
            self.world = World(
                stage_units_in_meters=1.0,
                physics_dt=self.config.PHYSICS["physics_dt"],
                rendering_dt=self.config.PHYSICS["rendering_dt"]
            )
            self.world.scene.clear()
            
            # 设置高性能物理参数（从配置读取）
            physics_context = self.world.get_physics_context()
            physics_context.set_gravity(-9.81)
            physics_context.set_solver_type("TGS")
            
            # 启用GPU加速（使用配置参数）
            physics_context.enable_gpu_dynamics(True)
            physics_context.set_gpu_max_rigid_contact_count(self.config.PHYSICS["gpu_max_rigid_contact_count"])
            physics_context.set_gpu_max_rigid_patch_count(self.config.PHYSICS["gpu_max_rigid_patch_count"])
            physics_context.set_gpu_heap_capacity(self.config.PHYSICS["gpu_heap_capacity"])
            physics_context.set_gpu_temp_buffer_capacity(self.config.PHYSICS["gpu_temp_buffer_capacity"])
            physics_context.set_gpu_max_num_partitions(self.config.PHYSICS["gpu_max_num_partitions"])
            
            print("✅ CUDA GPU物理加速已启用（配置驱动）")
            
            # 添加地面
            ground = FixedCuboid(
                prim_path="/World/Ground",
                name="ground",
                position=np.array([0.0, 0.0, -0.5]),
                scale=np.array([50.0, 50.0, 1.0]),
                color=np.array([0.5, 0.5, 0.5])
            )
            self.world.scene.add(ground)
            
            # 设置地面摩擦
            self._setup_ground_friction()
            
            # 设置照明
            self._setup_lighting()
            
            # 初始化障碍物地图
            self._initialize_obstacle_map()
            
            # 初始化高级导航系统
            self.advanced_navigation = AdvancedNavigationSystem(self.config)
            print("✅ 高级导航系统初始化完成")
            
            print("✅ Isaac Sim环境初始化完成（高级导航优化）")
            return True
            
        except Exception as e:
            print(f"❌ Isaac Sim初始化失败: {e}")
            return False
    
    def _setup_ground_friction(self):
        """设置地面摩擦（使用配置参数）"""
        try:
            stage = self.world.stage
            ground_prim = stage.GetPrimAtPath("/World/Ground")
            
            if ground_prim.IsValid():
                physics_material_api = UsdPhysics.MaterialAPI.Apply(ground_prim)
                physics_material_api.CreateStaticFrictionAttr().Set(self.config.PHYSICS["ground_static_friction"])
                physics_material_api.CreateDynamicFrictionAttr().Set(self.config.PHYSICS["ground_dynamic_friction"])
                physics_material_api.CreateRestitutionAttr().Set(self.config.PHYSICS["ground_restitution"])
                if self.config.DEBUG["enable_debug_output"]:
                    print("✅ 配置驱动的地面摩擦设置完成")
            
        except Exception as e:
            print(f"地面摩擦设置失败: {e}")
    
    def _setup_lighting(self):
        """设置照明（使用配置参数）"""
        try:
            light_prim = prim_utils.create_prim("/World/DistantLight", "DistantLight")
            distant_light = UsdLux.DistantLight(light_prim)
            distant_light.CreateIntensityAttr(self.config.LIGHTING["distant_light_intensity"])
            distant_light.CreateColorAttr(self.config.LIGHTING["distant_light_color"])
            if self.config.DEBUG["enable_debug_output"]:
                print("✅ 配置驱动的照明设置完成")
        except Exception as e:
            print(f"照明设置失败: {e}")
    
    def _initialize_obstacle_map(self):
        """初始化A*路径规划的障碍物地图"""
        try:
            map_cells = int(self.map_size / self.grid_resolution)
            self.obstacle_map = np.zeros((map_cells, map_cells), dtype=bool)
            if self.config.DEBUG["enable_debug_output"]:
                print(f"✅ A*路径规划地图初始化完成 ({map_cells}x{map_cells})")
        except Exception as e:
            print(f"障碍物地图初始化失败: {e}")
    
    def initialize_robot(self):
        """初始化Create-3+机械臂（配置驱动）"""
        print("🤖 正在初始化Create-3+机械臂（配置驱动）...")
        
        try:
            if self.config.DEBUG["enable_debug_output"]:
                print(f"🔧 使用轮子配置: {self.wheel_config}")
                print(f"🦾 加载机器人模型: {self.robot_usd_path}")
            
            # 创建机器人
            self.mobile_base = WheeledRobot(
                prim_path=self.robot_prim_path,
                name="create3_robot",
                wheel_dof_names=self.wheel_config,
                create_robot=True,
                usd_path=self.robot_usd_path,
                position=np.array([0.0, 0.0, 0.0])
            )
            
            self.world.scene.add(self.mobile_base)
            print("✅ 机器人创建成功")
            
            # 创建差分控制器（使用配置参数）
            self.differential_controller = DifferentialController(
                name="create3_controller",
                wheel_radius=self.config.ROBOT_CONTROL["wheel_radius"],
                wheel_base=self.config.ROBOT_CONTROL["wheel_base"],
                max_linear_speed=self.max_linear_velocity,
                max_angular_speed=self.max_angular_velocity
            )
            
            print("✅ 配置驱动的差分控制器创建成功")
            return True
            
        except Exception as e:
            print(f"❌ 机器人初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def create_indoor_scene(self):
        """创建室内清洁场景（使用配置文件，修正位置缩放）"""
        print("🏠 创建室内清洁场景（配置驱动+位置修正）...")
        
        try:
            stage = self.world.stage
            
            # 从配置读取家具位置和缩放
            furniture_scale = self.config.SCALE_CONFIG["furniture"]
            if self.config.DEBUG["enable_debug_output"]:
                print(f"🔧 家具缩放比例: {furniture_scale}")
                print(f"🔧 坐标系缩放: {COORDINATE_SCALE}")
            
            for furniture_name, (x, y, z, rot) in self.config.FURNITURE_POSITIONS.items():
                if furniture_name in self.config.ASSET_PATHS["furniture"]:
                    usd_path = self.get_asset_path(self.config.ASSET_PATHS["furniture"][furniture_name])
                    prim_path = f"/World/Furniture/{furniture_name}"
                    
                    # 创建引用
                    furniture_prim = stage.DefinePrim(prim_path, "Xform")
                    furniture_prim.GetReferences().AddReference(usd_path)
                    
                    # 修正：使用坐标系缩放转换位置，但保持物体大小缩放
                    world_x = x * COORDINATE_SCALE
                    world_y = y * COORDINATE_SCALE
                    world_z = z
                    
                    self._safe_set_transform_with_scale(furniture_prim, world_x, world_y, world_z, rot, furniture_scale)
                    
                    if self.config.DEBUG["enable_debug_output"]:
                        print(f"   ✅ 创建家具: {furniture_name} 配置位置: ({x}, {y}, {z}) -> 世界位置: ({world_x:.2f}, {world_y:.2f}, {world_z}) 缩放: {furniture_scale}")
            
            # 从配置读取书籍位置
            book_scale = self.config.SCALE_CONFIG["books"]
            if self.config.DEBUG["enable_debug_output"]:
                print(f"📚 书籍缩放比例: {book_scale}")
            
            for book_name, (x, y, z) in self.config.BOOK_POSITIONS.items():
                if book_name in self.config.ASSET_PATHS["books"]:
                    usd_path = self.get_asset_path(self.config.ASSET_PATHS["books"][book_name])
                    prim_path = f"/World/Books/{book_name}"
                    
                    book_prim = stage.DefinePrim(prim_path, "Xform")
                    book_prim.GetReferences().AddReference(usd_path)
                    
                    # 修正：使用坐标系缩放转换位置
                    world_x = x * COORDINATE_SCALE
                    world_y = y * COORDINATE_SCALE
                    world_z = z
                    
                    self._safe_set_transform_with_scale(book_prim, world_x, world_y, world_z, 0.0, book_scale)
                    
                    if self.config.DEBUG["enable_debug_output"]:
                        print(f"   📚 放置书籍: {book_name} 配置位置: ({x}, {y}, {z}) -> 世界位置: ({world_x:.2f}, {world_y:.2f}, {world_z}) 缩放: {book_scale}")
            
            print("✅ 室内场景创建完成（配置驱动+位置修正）")
            return True
            
        except Exception as e:
            print(f"❌ 创建室内场景失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def create_cleanup_environment(self):
        """创建清洁环境（使用配置文件，修正位置缩放）"""
        print("🗑️ 创建清洁环境（配置驱动+位置修正）...")
        
        try:
            stage = self.world.stage
            
            # 从配置读取小垃圾位置和缩放
            small_trash_scale = self.config.SCALE_CONFIG["small_trash"]
            if self.config.DEBUG["enable_debug_output"]:
                print(f"🔸 小垃圾缩放比例: {small_trash_scale}")
            
            # 创建小垃圾
            for i, (name, pos) in enumerate(self.config.SMALL_TRASH_POSITIONS.items()):
                if name in self.config.ASSET_PATHS["small_trash"]:
                    usd_path = self.get_asset_path(self.config.ASSET_PATHS["small_trash"][name])
                    prim_path = f"/World/SmallTrash/{name}_{i}"
                    
                    trash_prim = stage.DefinePrim(prim_path, "Xform")
                    trash_prim.GetReferences().AddReference(usd_path)
                    
                    # 修正：使用坐标系缩放转换位置
                    world_x = pos[0] * COORDINATE_SCALE
                    world_y = pos[1] * COORDINATE_SCALE
                    world_z = pos[2]
                    world_pos = [world_x, world_y, world_z]
                    
                    self._safe_set_transform_with_scale(trash_prim, world_x, world_y, world_z, 0.0, small_trash_scale)
                    
                    trash_obj = self._create_object_wrapper(prim_path, f"small_{name}_{i}", world_pos)
                    self.small_trash_objects.append(trash_obj)
                    
                    if self.config.DEBUG["enable_debug_output"]:
                        print(f"   📍 小垃圾: {name} 配置位置: {pos} -> 世界位置: ({world_x:.2f}, {world_y:.2f}, {world_z}) 缩放: {small_trash_scale}")
            
            # 从配置读取大垃圾位置和缩放
            large_trash_scale = self.config.SCALE_CONFIG["large_trash"]
            if self.config.DEBUG["enable_debug_output"]:
                print(f"🔹 大垃圾缩放比例: {large_trash_scale}")
            
            # 创建大垃圾
            for i, (name, pos) in enumerate(self.config.LARGE_TRASH_POSITIONS.items()):
                if name in self.config.ASSET_PATHS["large_trash"]:
                    usd_path = self.get_asset_path(self.config.ASSET_PATHS["large_trash"][name])
                    prim_path = f"/World/LargeTrash/{name}_{i}"
                    
                    trash_prim = stage.DefinePrim(prim_path, "Xform")
                    trash_prim.GetReferences().AddReference(usd_path)
                    
                    # 修正：使用坐标系缩放转换位置
                    world_x = pos[0] * COORDINATE_SCALE
                    world_y = pos[1] * COORDINATE_SCALE
                    world_z = pos[2]
                    world_pos = [world_x, world_y, world_z]
                    
                    self._safe_set_transform_with_scale(trash_prim, world_x, world_y, world_z, 0.0, large_trash_scale)
                    
                    trash_obj = self._create_object_wrapper(prim_path, f"large_{name}_{i}", world_pos)
                    self.large_trash_objects.append(trash_obj)
                    
                    if self.config.DEBUG["enable_debug_output"]:
                        print(f"   🦾 大垃圾: {name} 配置位置: {pos} -> 世界位置: ({world_x:.2f}, {world_y:.2f}, {world_z}) 缩放: {large_trash_scale}")
            
            print(f"✅ 清洁环境创建完成（配置驱动+位置修正）:")
            print(f"   - 小垃圾(吸附): {len(self.small_trash_objects)}个")
            print(f"   - 大垃圾(抓取): {len(self.large_trash_objects)}个")
            
            return True
            
        except Exception as e:
            print(f"❌ 创建清洁环境失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _safe_set_transform_with_scale(self, prim, x, y, z, rot_z, scale=1.0):
        """安全地设置USD prim的transform，包含缩放（修正版本）"""
        try:
            from pxr import UsdGeom
            xform = UsdGeom.Xform(prim)
            
            # 清除现有的变换操作
            xform.ClearXformOpOrder()
            
            # 按顺序添加变换操作：平移 -> 旋转 -> 缩放
            
            # 1. 添加平移操作（不缩放位置）
            translate_op = xform.AddTranslateOp()
            translate_op.Set(Gf.Vec3d(x, y, z))
            
            # 2. 添加旋转操作（如果需要）
            if rot_z != 0.0:
                rotate_op = xform.AddRotateZOp()
                rotate_op.Set(rot_z)
            
            # 3. 添加缩放操作（只缩放物体大小）
            if scale != 1.0:
                scale_op = xform.AddScaleOp()
                scale_op.Set(Gf.Vec3f(scale, scale, scale))
            
        except Exception as e:
            if self.config.DEBUG["enable_debug_output"]:
                print(f"设置带缩放的transform失败: {e}")
            # 备用方案：使用矩阵变换（修正版本）
            try:
                from pxr import UsdGeom
                xform = UsdGeom.Xform(prim)
                
                # 创建组合变换矩阵（修正版本）
                import math
                cos_rot = math.cos(math.radians(rot_z))
                sin_rot = math.sin(math.radians(rot_z))
                
                # 正确的4x4变换矩阵：平移在最后一行前3列
                final_matrix = Gf.Matrix4d(
                    scale * cos_rot, -scale * sin_rot, 0, 0,
                    scale * sin_rot, scale * cos_rot, 0, 0,
                    0, 0, scale, 0,
                    x, y, z, 1  # 平移分量在最后一行
                )
                
                xform.ClearXformOpOrder()
                matrix_op = xform.AddTransformOp()
                matrix_op.Set(final_matrix)
                
            except Exception as e2:
                if self.config.DEBUG["enable_debug_output"]:
                    print(f"备用transform设置也失败: {e2}")
    
    def _create_object_wrapper(self, prim_path, name, position):
        """创建对象包装器"""
        class ObjectWrapper:
            def __init__(self, prim_path, name, position, stage):
                self.prim_path = prim_path
                self.name = name
                self._position = np.array(position)
                self._stage = stage
                
            def get_world_pose(self):
                try:
                    prim = self._stage.GetPrimAtPath(self.prim_path)
                    if prim.IsValid():
                        from pxr import UsdGeom
                        xform = UsdGeom.Xform(prim)
                        matrix = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                        translation = matrix.ExtractTranslation()
                        position = np.array([translation[0], translation[1], translation[2]])
                        orientation = np.array([0.0, 0.0, 0.0, 1.0])
                        return position, orientation
                    else:
                        return self._position, np.array([0.0, 0.0, 0.0, 1.0])
                except:
                    return self._position, np.array([0.0, 0.0, 0.0, 1.0])
            
            def set_world_pose(self, position, orientation):
                try:
                    prim = self._stage.GetPrimAtPath(self.prim_path)
                    if prim.IsValid():
                        from pxr import UsdGeom
                        xform = UsdGeom.Xform(prim)
                        
                        existing_ops = xform.GetOrderedXformOps()
                        translate_op = None
                        
                        for op in existing_ops:
                            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                                translate_op = op
                                break
                        
                        if translate_op is None:
                            translate_op = xform.AddTranslateOp()
                        
                        translate_op.Set(Gf.Vec3d(position[0], position[1], position[2]))
                        self._position = np.array(position)
                        
                except Exception as e:
                    self._position = np.array(position)
        
        return ObjectWrapper(prim_path, name, position, self.world.stage)
    
    def setup_post_load(self):
        """World加载后的设置（配置驱动）"""
        print("🔧 正在进行配置驱动的后加载设置...")
        
        try:
            self.world.reset()
            
            # 快速稳定
            for _ in range(30):
                self.world.step(render=True)
                time.sleep(0.016)
            
            self.mobile_base = self.world.scene.get_object("create3_robot")
            
            if self.mobile_base is None:
                print("❌ 无法获取机器人对象")
                return False
            
            print(f"✅ 机器人对象获取成功")
            
            if self.config.DEBUG["show_robot_state"]:
                self._debug_robot_state()
            
            self._setup_joint_control()
            self._optimize_robot_physics()
            self._move_arm_to_pose("home")
            self._test_wheel_movement()
            
            return True
            
        except Exception as e:
            print(f"❌ 后加载设置失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _debug_robot_state(self):
        """调试机器人状态"""
        try:
            print("🔍 机器人状态:")
            
            if hasattr(self.mobile_base, 'dof_names'):
                print(f"   DOF数量: {len(self.mobile_base.dof_names)}")
                for i, name in enumerate(self.mobile_base.dof_names):
                    print(f"     [{i:2d}] {name}")
            
            try:
                position, orientation = self.mobile_base.get_world_pose()
                print(f"   当前位置: {position}")
                print(f"   当前高度: {position[2]:.3f}m")
                    
            except Exception as e:
                print(f"   获取位置失败: {e}")
                
        except Exception as e:
            print(f"调试失败: {e}")
    
    def _setup_joint_control(self):
        """设置关节控制参数（配置驱动）"""
        try:
            articulation_controller = self.mobile_base.get_articulation_controller()
            if not articulation_controller:
                print("⚠️ 无法获取关节控制器")
                return
            
            if self.config.DEBUG["enable_debug_output"]:
                print("🔧 设置配置驱动的关节控制参数...")
            
            if not hasattr(self.mobile_base, 'dof_names'):
                print("⚠️ 无法获取DOF名称")
                return
                
            num_dofs = len(self.mobile_base.dof_names)
            kp = np.zeros(num_dofs)
            kd = np.zeros(num_dofs)
            
            if self.config.DEBUG["enable_debug_output"]:
                print(f"   总DOF数量: {num_dofs}")
            
            # 轮子关节设置（使用配置参数）
            wheel_indices = []
            for wheel_name in self.wheel_config:
                if wheel_name in self.mobile_base.dof_names:
                    idx = self.mobile_base.dof_names.index(wheel_name)
                    wheel_indices.append(idx)
                    kp[idx] = self.config.JOINT_CONTROL["wheel_kp"]
                    kd[idx] = self.config.JOINT_CONTROL["wheel_kd"]
                    if self.config.DEBUG["enable_debug_output"]:
                        print(f"   轮子关节: {wheel_name} (索引: {idx})")
            
            # 机械臂关节设置（使用配置参数）
            arm_indices = []
            for joint_name in self.arm_joint_names:
                if joint_name in self.mobile_base.dof_names:
                    idx = self.mobile_base.dof_names.index(joint_name)
                    arm_indices.append(idx)
                    kp[idx] = self.config.JOINT_CONTROL["arm_kp"]
                    kd[idx] = self.config.JOINT_CONTROL["arm_kd"]
            
            # 夹爪关节设置（使用配置参数）
            gripper_indices = []
            for joint_name in self.gripper_joint_names:
                if joint_name in self.mobile_base.dof_names:
                    idx = self.mobile_base.dof_names.index(joint_name)
                    gripper_indices.append(idx)
                    kp[idx] = self.config.JOINT_CONTROL["gripper_kp"]
                    kd[idx] = self.config.JOINT_CONTROL["gripper_kd"]
            
            # 其他关节设置（使用配置参数）
            for i in range(num_dofs):
                if i not in wheel_indices and i not in arm_indices and i not in gripper_indices:
                    kp[i] = self.config.JOINT_CONTROL["default_kp"]
                    kd[i] = self.config.JOINT_CONTROL["default_kd"]
            
            articulation_controller.set_gains(kps=kp, kds=kd)
            
            if self.config.DEBUG["enable_debug_output"]:
                print(f"   ✅ 配置驱动的关节参数设置完成")
                print(f"   - 轮子关节: {len(wheel_indices)}个")
                print(f"   - 机械臂关节: {len(arm_indices)}个") 
                print(f"   - 夹爪关节: {len(gripper_indices)}个")
            
            self.wheel_joint_indices = wheel_indices
            
        except Exception as e:
            print(f"设置关节控制失败: {e}")
    
    def _optimize_robot_physics(self):
        """优化机器人物理属性（配置驱动）"""
        try:
            if self.config.DEBUG["enable_debug_output"]:
                print("🔧 优化机器人物理属性（配置驱动）...")
            
            stage = self.world.stage
            
            # 优化底盘物理属性（使用配置参数）
            base_link_path = f"{self.robot_prim_path}/create_3/base_link"
            base_link_prim = stage.GetPrimAtPath(base_link_path)
            
            if base_link_prim.IsValid():
                mass_api = UsdPhysics.MassAPI.Apply(base_link_prim)
                mass_api.CreateMassAttr().Set(self.config.PHYSICS["robot_mass"])
                mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*self.config.PHYSICS["robot_com_offset"]))
                mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*self.config.PHYSICS["robot_inertia"]))
                
                if self.config.DEBUG["enable_debug_output"]:
                    print("   ✅ 配置驱动的底盘物理属性设置完成")
            
            # 设置物理场景参数（使用配置参数）- 修正API调用
            physics_context = self.world.get_physics_context()
            try:
                # 尝试使用新的API
                physics_context.set_solver_position_iteration_count(self.config.PHYSICS["solver_position_iterations"])
                physics_context.set_solver_velocity_iteration_count(self.config.PHYSICS["solver_velocity_iterations"])
            except AttributeError:
                # 如果新API不存在，尝试旧的API
                try:
                    physics_context.set_position_iteration_count(self.config.PHYSICS["solver_position_iterations"])
                    physics_context.set_velocity_iteration_count(self.config.PHYSICS["solver_velocity_iterations"])
                except AttributeError:
                    if self.config.DEBUG["enable_debug_output"]:
                        print("   ⚠️ 无法设置求解器迭代次数（API不兼容）")
            
            if self.config.DEBUG["enable_debug_output"]:
                print("   ✅ 配置驱动的物理场景参数设置完成")
                
        except Exception as e:
            print(f"物理属性优化失败: {e}")
    
    def _move_arm_to_pose(self, pose_name):
        """机械臂移动（使用配置的姿态）"""
        try:
            if pose_name not in self.arm_poses:
                print(f"⚠️ 未知的机械臂姿态: {pose_name}")
                return False
            
            target_positions = self.arm_poses[pose_name]
            
            articulation_controller = self.mobile_base.get_articulation_controller()
            if not articulation_controller:
                return False
            
            if hasattr(self.mobile_base, 'dof_names'):
                num_dofs = len(self.mobile_base.dof_names)
                joint_positions = np.zeros(num_dofs)
                
                for i, joint_name in enumerate(self.arm_joint_names):
                    if joint_name in self.mobile_base.dof_names and i < len(target_positions):
                        idx = self.mobile_base.dof_names.index(joint_name)
                        joint_positions[idx] = target_positions[i]
                
                action = ArticulationAction(joint_positions=joint_positions)
                articulation_controller.apply_action(action)
                
                for _ in range(20):
                    self.world.step(render=True)
                    time.sleep(0.016)
                
                if self.config.DEBUG["show_grasp_details"]:
                    print(f"🦾 机械臂移动到 {pose_name} 姿态")
                return True
            
        except Exception as e:
            print(f"机械臂移动失败: {e}")
            return False
    
    def _control_gripper(self, open_close):
        """夹爪控制（使用配置参数）"""
        try:
            articulation_controller = self.mobile_base.get_articulation_controller()
            if not articulation_controller:
                return False
            
            gripper_position = self.gripper_open if open_close == "open" else self.gripper_closed
            
            if hasattr(self.mobile_base, 'dof_names'):
                num_dofs = len(self.mobile_base.dof_names)
                joint_positions = np.zeros(num_dofs)
                
                for joint_name in self.gripper_joint_names:
                    if joint_name in self.mobile_base.dof_names:
                        idx = self.mobile_base.dof_names.index(joint_name)
                        joint_positions[idx] = gripper_position
                
                action = ArticulationAction(joint_positions=joint_positions)
                articulation_controller.apply_action(action)
                
                for _ in range(10):
                    self.world.step(render=True)
                    time.sleep(0.016)
                
                if self.config.DEBUG["show_grasp_details"]:
                    print(f"🤏 夹爪 {'张开' if open_close == 'open' else '闭合'}")
                return True
                
        except Exception as e:
            print(f"夹爪控制失败: {e}")
            return False
    
    def _test_wheel_movement(self):
        """轮子测试"""
        try:
            if self.config.DEBUG["enable_debug_output"]:
                print("🧪 轮子测试...")
            
            initial_pos, initial_yaw = self.get_robot_pose()
            
            for _ in range(60):
                success = self._send_movement_command(0.3, 0.0)
                if success:
                    self.world.step(render=True)
                    time.sleep(0.016)
            
            self._send_movement_command(0.0, 0.0)
            self._wait_for_stability(0.5)
            
            final_pos, final_yaw = self.get_robot_pose()
            distance_moved = np.linalg.norm(final_pos[:2] - initial_pos[:2])
            
            if distance_moved > 0.05:
                if self.config.DEBUG["enable_debug_output"]:
                    print("   ✅ 轮子测试成功")
                return True
            else:
                print("   ⚠️ 轮子测试失败")
                return False
                    
        except Exception as e:
            print(f"轮子测试失败: {e}")
            return False
    
    def get_robot_pose(self):
        """获取机器人位置"""
        try:
            if self.mobile_base:
                position, orientation = self.mobile_base.get_world_pose()
                
                try:
                    from scipy.spatial.transform import Rotation as R
                    quat = np.array([orientation[1], orientation[2], orientation[3], orientation[0]])
                    if np.linalg.norm(quat) > 0:
                        r = R.from_quat(quat)
                        yaw = r.as_euler('xyz')[2]
                    else:
                        yaw = 0.0
                except:
                    yaw = 0.0
                
                self.current_position = position
                self.current_orientation = yaw
                
                return position.copy(), yaw
        except Exception as e:
            if self.config.DEBUG["enable_debug_output"]:
                print(f"获取位置失败: {e}")
            
        return self.current_position.copy(), self.current_orientation
    
    def _send_movement_command(self, linear_vel, angular_vel):
        """发送移动命令（优化版本）"""
        try:
            # 记录性能统计
            self.performance_stats['movement_commands_sent'] += 1
            
            linear_vel = np.clip(linear_vel, -self.max_linear_velocity, self.max_linear_velocity)
            angular_vel = np.clip(angular_vel, -self.max_angular_velocity, self.max_angular_velocity)
            
            success = False
            
            if self.differential_controller and self.mobile_base:
                try:
                    command = np.array([linear_vel, angular_vel])
                    wheel_actions = self.differential_controller.forward(command)
                    
                    if hasattr(self.mobile_base, 'apply_wheel_actions'):
                        self.mobile_base.apply_wheel_actions(wheel_actions)
                        success = True
                except Exception as e:
                    if self.config.DEBUG["enable_debug_output"]:
                        print(f"差分控制器失败: {e}")
            
            # 备用方案：直接关节控制
            if not success and hasattr(self, 'wheel_joint_indices') and len(self.wheel_joint_indices) >= 2:
                try:
                    articulation_controller = self.mobile_base.get_articulation_controller()
                    if articulation_controller:
                        wheel_radius = self.config.ROBOT_CONTROL["wheel_radius"]
                        wheel_base = self.config.ROBOT_CONTROL["wheel_base"]
                        
                        left_wheel_vel = (linear_vel - angular_vel * wheel_base / 2.0) / wheel_radius
                        right_wheel_vel = (linear_vel + angular_vel * wheel_base / 2.0) / wheel_radius
                        
                        num_dofs = len(self.mobile_base.dof_names) if hasattr(self.mobile_base, 'dof_names') else 10
                        joint_velocities = np.zeros(num_dofs)
                        
                        joint_velocities[self.wheel_joint_indices[0]] = left_wheel_vel
                        joint_velocities[self.wheel_joint_indices[1]] = right_wheel_vel
                        
                        action = ArticulationAction(joint_velocities=joint_velocities)
                        articulation_controller.apply_action(action)
                        success = True
                except Exception as e:
                    if self.config.DEBUG["enable_debug_output"]:
                        print(f"直接关节控制失败: {e}")
            
            if success:
                self.performance_stats['successful_movements'] += 1
            
            return success
                        
        except Exception as e:
            if self.config.DEBUG["enable_debug_output"]:
                print(f"发送移动命令失败: {e}")
            return False
    
    def _stop_robot(self):
        """停止机器人"""
        try:
            self._send_movement_command(0.0, 0.0)
        except Exception as e:
            if self.config.DEBUG["enable_debug_output"]:
                print(f"停止机器人失败: {e}")
    
    def smart_navigate_to_target(self, target_pos, max_time=None, tolerance=None):
        """智能导航（使用高级导航系统）"""
        # 使用配置的默认值
        if max_time is None:
            max_time = self.config.NAVIGATION["nav_timeout_small"]
        if tolerance is None:
            tolerance = self.config.NAVIGATION["tolerance_small_trash"]
        
        try:
            if self.config.DEBUG["show_navigation_progress"]:
                print(f"🎯 高级智能导航到目标: [{target_pos[0]:.3f}, {target_pos[1]:.3f}]")
            
            # 记录导航开始时间
            nav_start_time = time.time()
            
            # 使用高级导航系统
            success = self.advanced_navigation.navigate_to_target(
                self, target_pos, max_time, tolerance
            )
            
            # 记录导航时间
            nav_time = time.time() - nav_start_time
            self.performance_stats['total_navigation_time'] += nav_time
            
            if success:
                if self.config.DEBUG["show_navigation_progress"]:
                    print(f"   ✅ 高级导航成功！用时: {nav_time:.1f}s")
            else:
                if self.config.DEBUG["show_navigation_progress"]:
                    print(f"   ⚠️ 高级导航失败，用时: {nav_time:.1f}s")
            
            return success
            
        except Exception as e:
            print(f"高级导航失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def precise_grasp_sequence(self, target_position):
        """精确抓取序列（配置驱动）"""
        try:
            if self.config.DEBUG["show_grasp_details"]:
                print("   🎯 开始精确抓取序列...")
            
            self._stop_robot()
            self._wait_for_stability(0.5)
            
            if self.config.DEBUG["show_grasp_details"]:
                print("   1. 快速准备...")
            self._move_arm_to_pose("ready")
            
            robot_pos, _ = self.get_robot_pose()
            distance_to_target = np.linalg.norm(robot_pos[:2] - target_position[:2])
            
            if self.config.DEBUG["show_grasp_details"]:
                print(f"   📏 距离: {distance_to_target:.3f}m")
            
            if distance_to_target > 1.0:
                if self.config.DEBUG["show_grasp_details"]:
                    print("   ⚠️ 距离太远，无法精确抓取")
                return False
            
            pickup_pose = "pickup_low" if distance_to_target < 0.7 else "pickup"
            if self.config.DEBUG["show_grasp_details"]:
                print(f"   2. 使用 {pickup_pose} 姿态")
            
            self._move_arm_to_pose("inspect")
            self._move_arm_to_pose(pickup_pose)
            self._control_gripper("open")
            self._control_gripper("close")
            
            # 使用配置的成功率
            success_probability = self.config.SUCCESS_RATES["grasp_success_probability"]
            
            if random.random() < success_probability:
                if self.config.DEBUG["show_grasp_details"]:
                    print("   ✅ 抓取成功！")
                self._move_arm_to_pose("carry")
                self._move_arm_to_pose("stow")
                return True
            else:
                if self.config.DEBUG["show_grasp_details"]:
                    print("   ❌ 抓取失败！")
                self._control_gripper("open")
                self._move_arm_to_pose("stow")
                return False
                
        except Exception as e:
            print(f"   ❌ 精确抓取失败: {e}")
            try:
                self._control_gripper("open")
                self._move_arm_to_pose("stow")
            except:
                pass
            return False
    
    def collect_small_trash(self, trash_object):
        """收集小垃圾（配置驱动）"""
        try:
            trash_name = trash_object.name
            print(f"🔥 收集小垃圾: {trash_name}")
            
            trash_position = trash_object.get_world_pose()[0]
            target_position = trash_position.copy()
            target_position[2] = 0.0
            
            if self.config.DEBUG["show_navigation_progress"]:
                print(f"   目标位置: [{target_position[0]:.3f}, {target_position[1]:.3f}]")
            
            # 使用配置的导航参数
            nav_success = self.smart_navigate_to_target(
                target_position, 
                max_time=self.config.NAVIGATION["nav_timeout_small"], 
                tolerance=self.config.NAVIGATION["tolerance_small_trash"]
            )
            
            if nav_success:
                robot_pos, _ = self.get_robot_pose()
                collected_pos = robot_pos.copy()
                collected_pos[2] = -1.0
                
                trash_object.set_world_pose(collected_pos, trash_object.get_world_pose()[1])
                self.collected_objects.append(trash_name)
                
                print(f"✅ 小垃圾 {trash_name} 吸附成功！")
                return True
            else:
                print(f"⚠️ 小垃圾 {trash_name} 导航失败")
                self.collected_objects.append(f"{trash_name}(导航失败)")
                return False
                
        except Exception as e:
            print(f"收集小垃圾失败: {e}")
            return False
    
    def collect_large_trash(self, trash_object):
        """收集大垃圾（配置驱动）"""
        try:
            trash_name = trash_object.name
            print(f"🦾 收集大垃圾: {trash_name}")
            
            trash_position = trash_object.get_world_pose()[0]
            target_position = trash_position.copy()
            target_position[2] = 0.0
            
            if self.config.DEBUG["show_navigation_progress"]:
                print(f"   目标位置: [{target_position[0]:.3f}, {target_position[1]:.3f}]")
            
            # 使用配置的导航参数
            nav_success = self.smart_navigate_to_target(
                target_position, 
                max_time=self.config.NAVIGATION["nav_timeout_large"], 
                tolerance=self.config.NAVIGATION["tolerance_large_trash"]
            )
            
            if nav_success:
                grasp_success = self.precise_grasp_sequence(target_position)
                
                if grasp_success:
                    robot_pos, _ = self.get_robot_pose()
                    collected_pos = robot_pos.copy()
                    collected_pos[2] = -1.0
                    
                    trash_object.set_world_pose(collected_pos, trash_object.get_world_pose()[1])
                    self.collected_objects.append(trash_name)
                    
                    print(f"✅ 大垃圾 {trash_name} 精确夹取成功！")
                    return True
                else:
                    print(f"❌ 大垃圾 {trash_name} 抓取失败")
                    self.collected_objects.append(f"{trash_name}(抓取失败)")
                    return False
            else:
                print(f"⚠️ 大垃圾 {trash_name} 导航失败")
                self.collected_objects.append(f"{trash_name}(导航失败)")
                return False
                
        except Exception as e:
            print(f"收集大垃圾失败: {e}")
            return False
    
    def run_indoor_cleanup_demo(self):
        """运行室内清洁演示（配置驱动）"""
        print("\n" + "="*70)
        print("🏠 高级导航版Create-3+机械臂室内清洁系统演示")
        print("配置文件管理 | 丝滑移动控制 | 智能导航 | CUDA加速 | 位置修正")
        print("="*70)
        
        # 使用配置的稳定时间
        self._wait_for_stability(self.config.EXPERIMENT["stabilization_time"])
        
        pos, _ = self.get_robot_pose()
        print(f"🔍 机器人初始位置: {pos}")
        
        # 显示物体位置验证
        print(f"\n🔍 物体位置验证:")
        if self.small_trash_objects:
            for i, obj in enumerate(self.small_trash_objects[:3]):  # 显示前3个
                obj_pos, _ = obj.get_world_pose()
                print(f"   小垃圾 {obj.name}: {obj_pos[:2]}")
        
        if self.large_trash_objects:
            for i, obj in enumerate(self.large_trash_objects[:3]):  # 显示前3个
                obj_pos, _ = obj.get_world_pose()
                print(f"   大垃圾 {obj.name}: {obj_pos[:2]}")
        
        # 机械臂姿态演示（根据配置决定是否运行）
        if self.config.EXPERIMENT["run_arm_pose_demo"]:
            print(f"\n🦾 机械臂姿态演示（配置驱动）...")
            for pose in self.config.EXPERIMENT["demo_poses"]:
                if pose in self.arm_poses:
                    if self.config.DEBUG["show_grasp_details"]:
                        print(f"   快速测试 {pose} 姿态...")
                    self._move_arm_to_pose(pose)
        
        self._move_arm_to_pose("home")
        
        collection_success = 0
        total_items = len(self.small_trash_objects) + len(self.large_trash_objects)
        
        # 收集小垃圾
        print(f"\n🔥 开始智能收集小垃圾...")
        for i, trash in enumerate(self.small_trash_objects):
            print(f"\n📍 目标 {i+1}/{len(self.small_trash_objects)}: {trash.name}")
            if self.collect_small_trash(trash):
                collection_success += 1
            time.sleep(self.config.EXPERIMENT["collection_delay"])
        
        # 收集大垃圾
        print(f"\n🦾 开始智能收集大垃圾...")
        for i, trash in enumerate(self.large_trash_objects):
            print(f"\n📍 目标 {i+1}/{len(self.large_trash_objects)}: {trash.name}")
            if self.collect_large_trash(trash):
                collection_success += 1
            time.sleep(self.config.EXPERIMENT["collection_delay"])
        
        # 返回家（使用配置的导航参数）
        print(f"\n🏠 快速返回起始位置...")
        home_position = np.array([0.0, 0.0, 0.0])
        self.smart_navigate_to_target(
            home_position, 
            max_time=self.config.NAVIGATION["nav_timeout_home"],
            tolerance=self.config.NAVIGATION["tolerance_home"]
        )
        
        self._move_arm_to_pose("home")
        
        # 显示结果
        success_rate = (collection_success / total_items) * 100 if total_items > 0 else 0
        
        print(f"\n📊 室内清洁结果:")
        print(f"   成功收集: {collection_success}/{total_items} ({success_rate:.1f}%)")
        print(f"   收集清单: {', '.join(self.collected_objects)}")
        
        # 显示性能统计
        self._print_performance_stats()
        
        # 显示高级导航统计
        if self.advanced_navigation:
            self.advanced_navigation.print_stats()
        
        # 显示配置总结
        self.config.print_summary()
        
        print("\n✅ 高级导航版室内清洁演示完成！")
        print("💡 要调整参数，请编辑 config.py 文件")
        print("🚀 导航系统已优化，移动更加丝滑精准")
        print("🔧 位置缩放问题已修正")
    
    def _print_performance_stats(self):
        """打印性能统计"""
        stats = self.performance_stats
        success_rate = 0
        if stats['movement_commands_sent'] > 0:
            success_rate = (stats['successful_movements'] / stats['movement_commands_sent']) * 100
        
        print(f"\n🚀 性能统计:")
        print(f"   移动命令发送: {stats['movement_commands_sent']}")
        print(f"   成功移动: {stats['successful_movements']}")
        print(f"   移动成功率: {success_rate:.1f}%")
        print(f"   总导航时间: {stats['total_navigation_time']:.1f}s")
        
        if stats['total_navigation_time'] > 0:
            avg_speed = stats['total_distance_traveled'] / stats['total_navigation_time']
            print(f"   平均移动速度: {avg_speed:.2f}m/s")
    
    def _wait_for_stability(self, duration=1.0):
        """等待系统稳定"""
        steps = int(duration * 60)
        for _ in range(steps):
            if self.world:
                self.world.step(render=True)
            time.sleep(0.016)
    
    def cleanup(self):
        """清理资源"""
        try:
            self._stop_robot()
            if self.world:
                self.world.stop()
            print("🧹 高级导航系统清理完成")
        except Exception as e:
            print(f"清理时出错: {e}")

def main():
    """主函数（高级导航优化版）"""
    
    # 显示配置摘要
    config.print_summary()
    
    system = ConfigurableCreate3CleanupSystem(config)
    
    try:
        print("🚀 启动高级导航版室内清洁系统（位置修正版）...")
        
        # 高效初始化
        success = system.initialize_isaac_sim()
        if not success:
            return
        
        system._wait_for_stability(0.5)
        
        success = system.initialize_robot()
        if not success:
            print("❌ 机器人初始化失败")
            return
        
        success = system.create_indoor_scene()
        if not success:
            print("❌ 室内场景创建失败")
            return
        
        success = system.setup_post_load()
        if not success:
            print("❌ 后加载设置失败")
            return
        
        success = system.create_cleanup_environment()
        if not success:
            print("❌ 清洁环境创建失败")
            return
        
        system._wait_for_stability(2.0)
        
        # 运行演示
        system.run_indoor_cleanup_demo()
        
        # 保持系统运行
        print("\n💡 按 Ctrl+C 退出演示")
        print("💡 配置文件: config.py")
        print("🚀 已启用高级导航系统，移动更加丝滑精准")
        print("🔧 位置缩放问题已修正")
        try:
            while True:
                system.world.step(render=True)
                time.sleep(0.016)
        except KeyboardInterrupt:
            print("\n👋 退出高级导航演示...")
        
    except Exception as e:
        print(f"❌ 演示过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        system.cleanup()
        simulation_app.close()

if __name__ == "__main__":
    main()