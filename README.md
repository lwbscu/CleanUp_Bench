# CleanUp_Bench - Create-3+机械臂室内清洁演示

![Isaac Sim](https://img.shields.io/badge/Isaac%20Sim-4.5.0+-blue.svg)
![Python](https://img.shields.io/badge/Python-3.10+-green.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

这是一个基于 Isaac Sim 的 Create-3 机器人加机械臂的演示项目，专注于稳定性和可靠性。系统采用配置驱动设计，支持多用户环境，便于开源共享和定制化使用。

## ✨ 主要特性

- 🤖 **Create-3 + Panda 7DOF 机械臂**：完整的移动操作机器人系统
- 🏠 **室内清洁场景**：住宅环境中的垃圾收集任务
- 🚀 **CUDA GPU 加速**：高性能物理仿真
- 🎯 **A\*路径规划**：智能导航系统
- 🦾 **精确抓取**：机械臂精确操作
- 🔧 **配置驱动**：易于定制和调试
- 👥 **多用户支持**：自动检测用户环境

## 📋 项目结构

```
CleanUp_Bench/
├── scripts/
│   ├── ultra_stable_create3.py    # 主程序文件
│   ├── config.py                  # 配置文件（新增）
│   └── python.sh                  # Python启动脚本（修改版）
├── run.sh                         # 自动启动脚本
├── run_simple.sh                  # 简单启动脚本（可选）
├── README.md                      # 说明文档
└── LICENSE                        # 许可证文件
```

## 🛠️ 环境要求

### 必需软件

- **Isaac Sim**: 4.5.0 或更高版本
- **Python**: 3.10+
- **操作系统**: Ubuntu 20.04/22.04 或兼容系统
- **NVIDIA GPU**: 支持 CUDA 的显卡（推荐 RTX 系列）

### 推荐环境

- **conda**: 建议使用 isaaclab_4_5_0 环境
- **内存**: 16GB+ RAM
- **存储**: 10GB+ 可用空间

## 🚀 快速开始

### 步骤 1: 克隆项目

```bash
git clone https://github.com/lwbscu/CleanUp_Bench.git
cd CleanUp_Bench
```

### 步骤 2: 配置路径

项目支持自动路径检测，但您可能需要根据实际情况调整配置：

#### 方法 A: 自动配置（推荐）

系统会自动检测您的用户名和常见的 Isaac Sim 安装路径：

```bash
chmod +x run.sh
./run.sh
```

#### 方法 B: 手动配置

如果自动检测失败，请编辑 `scripts/config.py` 文件：

```python
# 在config.py中修改以下路径
config = CleanupSystemConfig(username="your_username")

# 或者手动设置路径
config.set_user_paths(
    isaac_assets_base="/path/to/your/isaac/assets",
    isaac_sim_install="/path/to/your/isaac/sim"
)
```

### 步骤 3: 运行演示

```bash
./run.sh
```

## ⚙️ 详细配置指南

### 路径配置

项目使用配置文件 `scripts/config.py` 管理所有路径。主要配置项：

#### 用户相关路径

```python
# 配置您的用户名（通常自动检测）
USERNAME = "your_username"

# Isaac Sim资产库路径
isaac_assets_base = f"/home/{USERNAME}/isaacsim_assets/Assets/Isaac/4.5"

# Isaac Sim安装路径
isaac_sim_install = f"/home/{USERNAME}/isaacsim"
```

#### 常见路径模式

系统会自动尝试以下路径模式：

**Isaac Sim 安装路径：**

- `~/isaacsim`
- `~/.local/share/ov/pkg/isaac_sim-*`
- `/opt/isaac_sim`
- `/usr/local/isaac_sim`

**Isaac 资产路径：**

- `~/isaacsim_assets/Assets/Isaac/4.5`
- `~/isaacsim/assets/Isaac/4.5`
- `/opt/isaac_sim/assets/Isaac/4.5`

### 缩放和位置配置

如果场景中的物体太大或位置不合适，可以在配置文件中调整：

```python
# 缩放配置
config.update_scale(
    furniture=0.02,      # 家具缩放至2%
    small_trash=0.02,    # 小垃圾缩放至2%
    large_trash=0.02,    # 大垃圾缩放至2%
    books=0.02           # 书籍缩放至2%
)

# 添加新的家具位置
config.add_furniture_position("sofa", x=0.0, y=3.0, z=0.0, rotation=180.0)

# 添加新的垃圾位置
config.add_trash_position("small", "pen", x=0.5, y=1.8, z=0.02)
```

### 机器人控制参数

```python
# 调整机器人移动参数
config.update_robot_control(
    max_linear_velocity=0.4,     # 最大线速度 (m/s)
    max_angular_velocity=1.5,    # 最大角速度 (rad/s)
    movement_threshold=0.6       # 到达目标的距离阈值 (m)
)

# 调整导航参数
config.update_navigation(
    stuck_threshold=0.1,         # 卡住检测阈值 (m)
    nav_timeout_small=20,        # 小垃圾导航超时 (s)
    nav_timeout_large=25         # 大垃圾导航超时 (s)
)
```

## 🎮 使用方法

### 启动方式

#### 方法 1: 自动启动（推荐）

```bash
./run.sh
```

#### 方法 2: 手动启动

```bash
# 激活conda环境
conda activate isaaclab_4_5_0

# 进入Isaac Sim目录
cd ~/isaacsim  # 或您的Isaac Sim安装路径

# 运行演示
./python.sh /path/to/CleanUp_Bench/scripts/ultra_stable_create3.py
```

### 快速配置预设

项目提供了几种预设配置：

```python
# 小场景配置（更少物品，更高性能）
config = QuickConfigs.small_scene()

# 超小家具配置（如果默认缩放还是太大）
config = QuickConfigs.tiny_furniture()

# 性能优化配置（降低物理精度提高性能）
config = QuickConfigs.performance_optimized()

# 调试模式配置（更多调试信息，更慢速度）
config = QuickConfigs.debug_mode()
```

## 🎯 演示内容

演示包含以下阶段：

1. **🔧 配置加载和验证** (5 秒)

   - 自动检测用户环境
   - 验证路径有效性
   - 加载配置参数

2. **🏠 室内场景创建** (30 秒)

   - 创建住宅环境
   - 放置家具和书籍
   - 生成垃圾物品

3. **🤖 机器人初始化** (30 秒)

   - 加载 Create-3+机械臂模型
   - 设置物理参数
   - 初始化控制器

4. **🦾 机械臂演示** (30 秒)

   - 展示各种预设姿态
   - 测试关节控制
   - 验证夹爪功能

5. **🔥 小垃圾收集** (2-3 分钟)

   - 智能导航到小垃圾
   - 吸附收集机制
   - A\*路径规划

6. **🦾 大垃圾抓取** (3-4 分钟)

   - 精确导航到大垃圾
   - 机械臂抓取操作
   - 搬运到收集区

7. **🏠 返回起点** (1 分钟)

   - 智能返回初始位置
   - 系统复位

8. **📊 结果统计**
   - 显示收集成功率
   - 性能指标分析

## 🔧 故障排除

### 常见问题及解决方案

#### 1. 路径检测失败

```
[ERROR] 住宅资产库缺失: /home/user/isaacsim_assets/...
```

**解决方案：**

- 检查 Isaac Sim 是否正确安装
- 确认住宅资产包是否已下载
- 手动编辑 `config.py` 中的路径设置

#### 2. 权限问题

```
Permission denied: ./run.sh
```

**解决方案：**

```bash
chmod +x run.sh
chmod +x scripts/python.sh
```

#### 3. conda 环境问题

```
Warning: running in conda env
```

**解决方案：**

```bash
conda activate isaaclab_4_5_0  # 或您的Isaac环境名称
```

#### 4. 用户名检测错误

**解决方案：**

```python
# 在config.py中手动指定用户名
config = CleanupSystemConfig(username="your_actual_username")
```

#### 5. 物体尺寸问题

如果场景中的物体太大或太小：

```python
# 调整缩放比例
config.update_scale(
    furniture=0.01,    # 减小家具
    books=0.1,         # 减小书籍
    small_trash=0.5,   # 增大小垃圾
)
```

### 调试模式

启用详细调试输出：

```python
# 在config.py中启用调试
config.DEBUG.update({
    "enable_debug_output": True,
    "show_robot_state": True,
    "show_navigation_progress": True,
    "show_grasp_details": True,
})
```

### 性能优化

如果系统运行缓慢：

```python
# 使用性能优化配置
config = QuickConfigs.performance_optimized()

# 或手动调整参数
config.PHYSICS["physics_dt"] = 1.0/60.0  # 降低物理频率
config.PHYSICS["solver_position_iterations"] = 4  # 降低求解器精度
```

## 📊 性能指标

演示结束后会显示：

- **收集成功率**: 完成垃圾收集的百分比
- **导航效率**: 路径规划的成功率
- **抓取成功率**: 机械臂抓取的成功率
- **总用时**: 完成任务的总时间
- **平均速度**: 移动的平均速度

## 🤝 贡献指南

欢迎提交 Issue 和 Pull Request！

### 贡献流程

1. Fork 这个项目
2. 创建您的特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交您的更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启一个 Pull Request

### 贡献方向

- 🐛 Bug 修复和性能优化
- ✨ 新的机器人控制策略
- 🏠 更多的场景和任务
- 📚 文档改进和翻译
- 🔧 配置和易用性改进

## 📄 许可证

本项目使用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情。

## 🙏 致谢

- NVIDIA Isaac Sim 团队提供的优秀仿真平台
- iRobot Create-3 和 Franka Panda 的开源模型
- 住宅资产包的设计师们

## 📞 联系方式

- 项目主页: [GitHub Repository](https://github.com/your-username/CleanUp_Bench)
- 问题反馈: [Issues](https://github.com/your-username/CleanUp_Bench/issues)
- 讨论区: [Discussions](https://github.com/your-username/CleanUp_Bench/discussions)

---

**注意**: 请确保您的系统满足 Isaac Sim 的最低硬件要求，包括支持 CUDA 的 GPU 和充足的内存。

## 🔄 版本历史

### v1.1.0 (当前版本)

- ✨ 新增配置驱动的路径管理
- 👥 支持多用户环境
- 🔧 自动路径检测功能
- 📚 改进的文档和配置指南
- 🐛 修复多个路径相关的 bug

### v1.0.0

- 🎉 初始版本发布
- 🤖 基础的 Create-3+机械臂清洁系统
- 🏠 室内场景演示
- 🚀 CUDA 加速物理仿真
