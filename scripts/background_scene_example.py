#!/usr/bin/env python3
"""
OSGT背景场景使用示例
演示如何在ultra_stable_create3.py中使用自定义背景场景
"""

from config import OSGTCleanupSystemConfig, OSGTQuickConfigs

def main():
    print("🏠 OSGT背景场景使用示例")
    print("=" * 50)
    
    # 示例1：使用默认配置，然后设置背景场景
    print("\n📖 示例1：修改默认配置的背景场景")
    config1 = OSGTCleanupSystemConfig()
    
    # 设置厨房背景场景
    config1.set_background_scene(
        usd_path="Kitchen_set/Kitchen_set_instanced.usd",
        scale=0.02,
        position=[0.0, 0.0, 0.0],
        rotation_z=0.0
    )
    
    print(f"✅ 背景场景路径: {config1.BACKGROUND_ENVIRONMENT['usd_path']}")
    print(f"✅ 缩放比例: {config1.BACKGROUND_ENVIRONMENT['scale']}")
    
    # 示例2：使用预设的厨房场景
    print("\n📖 示例2：使用预设的厨房场景")
    config2 = OSGTQuickConfigs.kitchen_scene("your_username")
    print(f"✅ 厨房场景已配置: {config2.BACKGROUND_ENVIRONMENT['usd_path']}")
    
    # 示例3：自定义背景场景
    print("\n📖 示例3：自定义背景场景")
    config3 = OSGTQuickConfigs.custom_scene_with_background(
        username="your_username",
        background_usd_path="Kitchen_set/Kitchen_set_instanced.usd",
        scenario_type="residential"
    )
    print(f"✅ 自定义背景场景已配置: {config3.BACKGROUND_ENVIRONMENT['usd_path']}")
    
    # 示例4：禁用背景场景
    print("\n📖 示例4：禁用背景场景")
    config4 = OSGTCleanupSystemConfig()
    config4.disable_background_scene()
    print(f"✅ 背景场景已禁用: {config4.BACKGROUND_ENVIRONMENT['usd_path']}")
    
    # 示例5：设置不同的背景场景文件
    print("\n📖 示例5：设置其他背景场景文件")
    config5 = OSGTCleanupSystemConfig()
    
    # 可以使用住宅资产库中的其他场景
    other_backgrounds = [
        "Kitchen_set/Kitchen_set_instanced.usd",
        "Apartment_01/Apartment_01_instanced.usd",
        "Office_01/Office_01_instanced.usd",
        "Warehouse_01/Warehouse_01_instanced.usd",
    ]
    
    for bg in other_backgrounds:
        print(f"   可用背景场景: {bg}")
    
    # 设置办公室场景
    config5.set_background_scene(
        usd_path="Office_01/Office_01_instanced.usd",
        scale=0.015,  # 办公室可能需要更小的缩放
        position=[0.0, 0.0, 0.0],
        rotation_z=90.0  # 旋转90度
    )
    
    print(f"✅ 办公室场景已配置: {config5.BACKGROUND_ENVIRONMENT['usd_path']}")
    
    # 示例6：根据背景场景调整物体位置
    print("\n📖 示例6：根据背景场景调整OSGT物体位置")
    config6 = OSGTCleanupSystemConfig()
    config6.set_background_scene("Kitchen_set/Kitchen_set_instanced.usd", scale=0.02)
    
    # 在厨房场景中调整物体位置
    config6.add_obstacle_position("kitchen_counter", 150.0, 0.0, 0.0, 0.0)
    config6.add_sweepable_position("kitchen_crumb", 120.0, 20.0, 0.03)
    config6.add_graspable_position("kitchen_utensil", 140.0, -10.0, 0.05)
    config6.add_task_area_position("kitchen_sink", 180.0, -50.0, 0.0, 0.0)
    
    print("✅ 厨房场景OSGT物体位置已调整")
    
    print("\n🎯 在ultra_stable_create3.py中使用:")
    print("=" * 50)
    print("# 方法1：修改现有配置")
    print("config = OSGTCleanupSystemConfig(username, 'residential')")
    print("config.set_background_scene('Kitchen_set/Kitchen_set_instanced.usd', scale=0.02)")
    print("")
    print("# 方法2：使用预设配置")
    print("config = OSGTQuickConfigs.kitchen_scene(username)")
    print("")
    print("# 方法3：自定义背景场景")
    print("config = OSGTQuickConfigs.custom_scene_with_background(")
    print("    username=username,")
    print("    background_usd_path='Kitchen_set/Kitchen_set_instanced.usd',")
    print("    scenario_type='residential'")
    print(")")
    print("")
    print("# 然后创建系统并运行")
    print("cleanup_system = OSGTCreate3CleanupSystem(config)")
    print("cleanup_system.initialize_isaac_sim()")
    print("cleanup_system.create_osgt_scene()  # 会自动加载背景场景")
    
    return config1

if __name__ == "__main__":
    main()
