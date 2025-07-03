# CleanUp_Bench - Create-3+机械臂演示

这是一个基于 Isaac Sim 的 Create-3 机器人加机械臂的演示项目，专注于稳定性和可靠性。

## 📋 项目结构

```
CleanUp_Bench/
├── scripts/
│   ├── ultra_stable_create3.py    # 主程序文件
│   └── python.sh                  # Python启动脚本（修改版）
├── run.sh                         # 自动启动脚本
├── run_simple.sh                  # 简单启动脚本
└── README.md                      # 说明文档
```

## 🛠️ 环境要求

- **Isaac Sim**: 4.5.0 或更高版本
- **Python**: 3.10
- **conda**: 建议使用 isaaclab_4_5_0 环境
- **操作系统**: Ubuntu 20.04/22.04 或兼容系统

## 🚀 快速开始

### 方法 1: 自动启动（推荐）

1. **下载项目**

   ```bash
   git clone <repository_url>
   cd CleanUp_Bench
   ```

2. **运行演示**
   ```bash
   chmod +x run.sh
   ./run.sh
   ```

### 方法 2: 手动启动

1. **激活环境**

   ```bash
   conda activate isaaclab_4_5_0  # 或你的Isaac环境名称
   ```

2. **进入 Isaac Sim 目录**

   ```bash
   cd ~/isaacsim  # 或你的Isaac Sim安装路径
   ```

3. **运行演示**
   ```bash
   ./path/to/CleanUp_Bench/run_simple.sh
   ```

### 方法 3: 传统方式

```bash
cd ~/isaacsim
conda activate isaaclab_4_5_0
./python.sh /path/to/CleanUp_Bench/scripts/ultra_stable_create3.py
```

## ⚙️ 配置说明

### 环境变量

项目支持以下环境变量进行配置：

- **ISAAC_SIM_PATH**: Isaac Sim 安装路径

  ```bash
  export ISAAC_SIM_PATH=/path/to/isaac_sim
  ```

- **ISAAC_ASSETS_ROOT**: Isaac 资源文件路径
  ```bash
  export ISAAC_ASSETS_ROOT=/path/to/isaac/assets
  ```

### 自动路径检测

如果没有设置环境变量，脚本会自动尝试检测以下路径：

**Isaac Sim 安装路径:**

- `$HOME/isaacsim`
- `/opt/isaac_sim`
- `$HOME/.local/share/ov/pkg/isaac_sim-*`

**Isaac 资源路径:**

- `$HOME/isaacsim_assets/Assets/Isaac/4.5`
- `$ISAAC_SIM_PATH/assets/Isaac/4.5`
- `/opt/isaac_sim/assets/Isaac/4.5`

## 🎮 演示内容

演示包含以下测试场景：

1. **机器人初始化**: 检查 Create-3+机械臂的正确加载
2. **移动测试**:
   - 向右移动 1 米
   - 移动到右前方 1.4 米处
   - 返回原点
3. **稳定性检查**: 监控机器人高度，防止翻倒
4. **环境交互**: 与测试环境中的目标物体交互

## 🔧 故障排除

### 常见问题

1. **找不到 Isaac Sim**

   ```
   [ERROR] 未找到Isaac Sim安装目录
   ```

   **解决方案**: 手动设置环境变量

   ```bash
   export ISAAC_SIM_PATH=/your/isaac/sim/path
   ```

2. **资源加载失败**

   ```
   无法加载机器人USD文件
   ```

   **解决方案**: 检查资源路径

   ```bash
   export ISAAC_ASSETS_ROOT=/path/to/isaac/assets
   ```

3. **conda 环境问题**

   ```
   Warning: running in conda env
   ```

   **解决方案**: 确保使用正确的 conda 环境

   ```bash
   conda activate isaaclab_4_5_0
   ```

4. **权限问题**
   ```
   Permission denied
   ```
   **解决方案**: 为脚本添加执行权限
   ```bash
   chmod +x run.sh run_simple.sh
   ```

### 调试模式

如果遇到问题，可以查看详细输出：

```bash
# 启用详细输出
export VERBOSE=1
./run.sh
```

## 📊 性能指标

演示结束后会显示以下指标：

- **成功率**: 完成测试任务的百分比
- **稳定性**: 机器人是否保持稳定状态
- **移动距离**: 实际移动的距离
- **用时**: 完成任务所需时间
- **平均速度**: 移动的平均速度

## 🔄 自定义配置

### 修改机器人参数

编辑 `scripts/ultra_stable_create3.py` 中的参数：

```python
# 控制参数
self.max_linear_velocity = 0.5     # 最大线速度
self.max_angular_velocity = 1.0    # 最大角速度
self.movement_threshold = 0.3      # 移动阈值
self.angular_threshold = 0.2       # 角度阈值
```

### 修改测试点

```python
test_points = [
    (np.array([1.0, 0.0, 0.0]), "右方1米"),
    (np.array([1.0, 1.0, 0.0]), "右前方1.4米"),
    (np.array([0.0, 0.0, 0.0]), "返回原点")
]
```

## 📝 版本信息

- **版本**: 1.0.0
- **兼容性**: Isaac Sim 4.5.0+
- **最后更新**: 2025 年 7 月

## 🤝 贡献

欢迎提交 Issue 和 Pull Request 来改进这个项目。

## 📄 许可证

本项目使用 MIT 许可证 - 查看 LICENSE 文件了解详情。

---

**注意**: 请确保你的系统满足 Isaac Sim 的最低硬件要求，包括支持的 GPU 和充足的内存。
