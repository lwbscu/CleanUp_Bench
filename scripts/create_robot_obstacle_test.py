#!/usr/bin/env python3
"""
Isaac Sim 4.5 LightBeam光束传感器避障系统
使用LightBeam传感器替代激光雷达进行距离检测
"""

import numpy as np
import time
import asyncio
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import omni
from isaacsim.core.api.objects import DynamicCuboid, GroundPlane
from isaacsim.core.api.world import World
from isaacsim.core.utils.extensions import enable_extension
from pxr import Sdf, UsdLux, Gf, UsdPhysics

print("🤖 初始化Isaac Sim 4.5 LightBeam光束传感器避障系统...")

# Set up scene
world = World()
ground_plane = GroundPlane('/World/GroundPlane')

# Add lighting
stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(1000)

# 创建机器人cube（红色）
robot_cube = DynamicCuboid(
    prim_path="/robot",
    name="robot",
    position=np.array([0.0, 0.0, 1.0]),
    scale=np.array([1, 1, 1]),
    size=1.0,
    color=np.array([255, 0, 0]),  # 红色 - 机器人
)

# 创建障碍物cube（蓝色）
obstacle_cube = DynamicCuboid(
    prim_path="/obstacle",
    name="obstacle", 
    position=np.array([3.0, 0.0, 1.0]),
    scale=np.array([1, 1, 1]),
    size=1.0,
    color=np.array([0, 0, 255]),  # 蓝色 - 障碍物
)

print("🔧 为障碍物添加碰撞检测（PhysX LightBeam必需）...")

# 确保障碍物有碰撞属性 - PhysX LightBeam必需！
obstacle_prim = stage.GetPrimAtPath("/obstacle")
if obstacle_prim:
    # 添加碰撞API
    collision_api = UsdPhysics.CollisionAPI.Apply(obstacle_prim)
    print("✅ 障碍物碰撞检测已启用")
else:
    print("❌ 无法找到障碍物prim")

print("📡 设置LightBeam光束传感器...")

# Enable PhysX sensors extension
enable_extension("isaacsim.sensors.physx")
simulation_app.update()

# 添加物理场景（必须的）
UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))

# 创建前方光束传感器
robot_pos, _ = robot_cube.get_world_pose()
lightbeam_height_offset = 0.6
lightbeam_position = robot_pos + np.array([0.0, 0.0, lightbeam_height_offset])

lightbeam_path = "/LightBeam_Sensor"

# 使用命令创建LightBeam传感器
result, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateLightBeamSensor",
    path=lightbeam_path,
    parent=None,
    min_range=0.2,
    max_range=10.0,
    translation=Gf.Vec3d(lightbeam_position[0], lightbeam_position[1], lightbeam_position[2]),
    orientation=Gf.Quatd(1, 0, 0, 0),
    forward_axis=Gf.Vec3d(1, 0, 0),  # 朝向X轴正方向（前方）
    num_rays=5,  # 使用5条光束
    curtain_length=0.5,
)

if result:
    print("✅ LightBeam光束传感器创建成功")
else:
    print("❌ LightBeam传感器创建失败")
    simulation_app.close()
    exit()

# 获取LightBeam传感器接口
from isaacsim.sensors.physx import _range_sensor
import omni.graph.core as og

lightbeam_interface = _range_sensor.acquire_lightbeam_sensor_interface()
timeline = omni.timeline.get_timeline_interface()

def setup_lightbeam_visualization():
    """设置光束可视化"""
    print("🎨 设置光束可视化...")
    
    try:
        # 创建ActionGraph来可视化光束
        (action_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacReadLightBeam", "isaacsim.sensors.physx.IsaacReadLightBeam"),
                    ("DebugDrawRayCast", "isaacsim.util.debug_draw.DebugDrawRayCast"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("IsaacReadLightBeam.inputs:lightbeamPrim", lightbeam_path),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "IsaacReadLightBeam.inputs:execIn"),
                    ("IsaacReadLightBeam.outputs:execOut", "DebugDrawRayCast.inputs:exec"),
                    ("IsaacReadLightBeam.outputs:beamOrigins", "DebugDrawRayCast.inputs:beamOrigins"),
                    ("IsaacReadLightBeam.outputs:beamEndPoints", "DebugDrawRayCast.inputs:beamEndPoints"),
                    ("IsaacReadLightBeam.outputs:numRays", "DebugDrawRayCast.inputs:numRays"),
                ],
            },
        )
        
        print("✅ 光束可视化ActionGraph创建成功")
        return True
        
    except Exception as e:
        print(f"⚠️ 光束可视化设置失败: {e}")
        print("💡 将尝试其他可视化方法")
        return False

def get_front_distance():
    """获取前方LightBeam传感器的最近距离"""
    try:
        # 确保仿真正在运行
        if not timeline.is_playing():
            return None
        
        # 获取光束传感器数据
        linear_depth = lightbeam_interface.get_linear_depth_data(lightbeam_path)
        beam_hit = lightbeam_interface.get_beam_hit_data(lightbeam_path).astype(bool)
        hit_pos = lightbeam_interface.get_hit_pos_data(lightbeam_path)
        
        if linear_depth is not None and len(linear_depth) > 0:
            min_distance = float('inf')
            
            # 遍历所有光束
            for i in range(len(linear_depth)):
                if beam_hit[i]:  # 如果光束命中了物体
                    distance = linear_depth[i]
                    
                    # 过滤有效距离范围
                    if 0.2 < distance < 10.0:
                        min_distance = min(min_distance, distance)
            
            # 返回最近的有效距离
            return min_distance if min_distance != float('inf') else None
        
        return None
        
    except Exception as e:
        # 调试时可以打印错误
        # print(f"获取LightBeam数据时出错: {e}")
        return None

def sync_lightbeam_with_robot():
    """让LightBeam传感器跟随机器人"""
    robot_pos, _ = robot_cube.get_world_pose()
    new_position = robot_pos + np.array([0.0, 0.0, lightbeam_height_offset])
    
    # 更新LightBeam传感器位置
    lightbeam_prim = stage.GetPrimAtPath(lightbeam_path)
    if lightbeam_prim:
        translate_attr = lightbeam_prim.GetAttribute("xformOp:translate")
        if translate_attr:
            translate_attr.Set(Gf.Vec3d(new_position[0], new_position[1], new_position[2]))

def get_detailed_lightbeam_info():
    """获取详细的LightBeam传感器信息"""
    try:
        if not timeline.is_playing():
            return None
        
        linear_depth = lightbeam_interface.get_linear_depth_data(lightbeam_path)
        beam_hit = lightbeam_interface.get_beam_hit_data(lightbeam_path).astype(bool)
        hit_pos = lightbeam_interface.get_hit_pos_data(lightbeam_path)
        
        if linear_depth is not None and len(linear_depth) > 0:
            beam_info = []
            for i in range(len(linear_depth)):
                beam_info.append({
                    'beam_id': i,
                    'hit': beam_hit[i],
                    'distance': linear_depth[i] if beam_hit[i] else None,
                    'hit_position': hit_pos[i] if beam_hit[i] else None
                })
            return beam_info
        
        return None
        
    except Exception as e:
        return None

# 回调函数
last_log_time = 0
log_interval = 1.0

def lightbeam_callback(_):
    global last_log_time
    current_time = time.time()
    
    if current_time - last_log_time >= log_interval:
        # 同步LightBeam传感器位置
        sync_lightbeam_with_robot()
        
        # 获取机器人和障碍物位置
        robot_pos, _ = robot_cube.get_world_pose()
        obstacle_pos, _ = obstacle_cube.get_world_pose()
        
        # 计算理论距离（用于对比）
        theoretical_distance = np.sqrt((obstacle_pos[0] - robot_pos[0])**2 + 
                                     (obstacle_pos[1] - robot_pos[1])**2)
        
        # 获取LightBeam测量距离
        measured_distance = get_front_distance()
        beam_details = get_detailed_lightbeam_info()
        
        # 检查仿真状态
        is_playing = timeline.is_playing()
        
        print("\n" + "="*80)
        print("🤖 Isaac Sim 4.5 LightBeam光束传感器避障监控")
        print(f"▶️  仿真状态: {'运行中' if is_playing else '已暂停'}")
        print(f"📍 机器人位置: [{robot_pos[0]:.2f}, {robot_pos[1]:.2f}, {robot_pos[2]:.2f}]")
        print(f"🔵 障碍物位置: [{obstacle_pos[0]:.2f}, {obstacle_pos[1]:.2f}, {obstacle_pos[2]:.2f}]")
        print(f"📏 理论距离: {theoretical_distance:.2f}m")
        
        if measured_distance is not None:
            print(f"📡 LightBeam测量: {measured_distance:.2f}m")
            
            # 显示详细光束信息
            if beam_details:
                print("🔦 光束详情:")
                for beam in beam_details:
                    if beam['hit']:
                        print(f"   光束{beam['beam_id']}: 命中 - 距离{beam['distance']:.2f}m")
                    else:
                        print(f"   光束{beam['beam_id']}: 未命中")
            
            # 对比理论值和测量值
            if abs(measured_distance - theoretical_distance) < 0.5:
                print("✅ 测量值与理论值接近 - 传感器工作正常")
            else:
                print("⚠️ 测量值与理论值差异较大")
            
            # 避障决策
            if measured_distance < 2.0:
                status = "🔴 危险"
                action = "立即避障"
            elif measured_distance < 3.5:
                status = "🟡 警告"
                action = "注意观察"
            else:
                status = "🟢 安全"
                action = "正常前进"
            
            print(f"🚨 状态: {status}")
            print(f"🧠 建议: {action}")
        else:
            if not is_playing:
                print("📡 LightBeam测量: 仿真未运行")
                print("🚨 状态: ⏸️ 等待仿真启动")
            else:
                print("📡 LightBeam测量: 无碰撞检测")
                print("🚨 状态: ⚠️ 检查障碍物碰撞属性")
                print("💡 提示: 确保障碍物有PhysX碰撞属性")
        
        print("="*80)
        last_log_time = current_time

# 添加回调
world.add_physics_callback("lightbeam_callback", lightbeam_callback)
simulation_app.update()

print("⏳ 正在启动仿真...")

# 重置并启动世界
world.reset()

# 等待场景稳定
for i in range(60):
    world.step(render=True)
    time.sleep(0.02)

# 开始仿真
world.play()

# 设置光束可视化（在仿真开始后）
setup_lightbeam_visualization()

# 等待ActionGraph生效
for i in range(30):
    world.step(render=True)
    time.sleep(0.02)

print("\n" + "="*80)
print("🤖 Isaac Sim 4.5 LightBeam光束传感器避障系统")
print("📋 系统说明:")
print("   🔴 红色方块 = 机器人")
print("   🔦 LightBeam传感器 = 5条前方光束检测（带可视化）")
print("   🔵 蓝色方块 = 障碍物（已启用碰撞检测）")
print("   📏 测距范围: 0.2-10米")
print("   🎨 光束可视化: 应该能看到5条光束线")
print("   🔧 关键特性: 光束幕式检测，适合简单避障")
print("="*80)
print("💡 测试方法:")
print("🎮 1. 拖拽蓝色障碍物靠近/远离红色机器人")
print("🔄 2. 移动机器人改变与障碍物的距离")
print("🔦 3. 观察5条光束的检测状态和可视化线条")
print("📊 4. 终端显示理论距离 vs LightBeam测量距离")
print("✅ 5. 查看每条光束的命中状态和距离")
print("🎨 6. 光束线条颜色会根据检测结果变化")
print("🛑 按 Ctrl+C 退出")
print("\n🔧 LightBeam优势:")
print("   ⚡ 更简单的配置和使用")
print("   🎯 专门用于距离检测和避障")
print("   📏 提供线性深度和命中位置")
print("   🔦 光束幕式检测，覆盖前方区域")
print("   💡 比激光雷达更轻量级")
print("   🎨 内置可视化支持")
print("\n🎨 可视化说明:")
print("   📐 如果看不到光束线，请检查:")
print("   🔍 1. ActionGraph是否创建成功")
print("   ⚙️ 2. Isaac Sim视窗中是否启用了Debug Draw")
print("   🎯 3. 摄像机角度是否合适观察光束")
print("   ✨ 4. 光束会从传感器原点射向前方")

step_count = 0

try:
    while True:
        world.step(render=True)
        step_count += 1
        
        if step_count % 10000 == 0:
            print(f"💓 系统运行中... ({step_count} 步)")
        
        time.sleep(0.016)
        
except KeyboardInterrupt:
    print(f"\n🛑 用户手动退出")

finally:
    print("🧹 正在清理资源...")
    try:
        world.stop()
        simulation_app.close()
        print("✅ 系统测试完成！")
    except:
        pass