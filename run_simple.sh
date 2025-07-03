#!/bin/bash

# 简单启动脚本 - 在Isaac Sim目录中运行
# 使用方法: 
# 1. cd ~/isaacsim (或你的Isaac Sim安装目录)
# 2. conda activate isaaclab_4_5_0 (或你的环境)
# 3. ./path/to/CleanUp_Bench/run_simple.sh

set -e

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SCRIPTS_DIR="$SCRIPT_DIR/scripts"

echo "🚀 启动Create-3+机械臂演示..."
echo "📁 脚本目录: $SCRIPTS_DIR"

# 检查当前是否在Isaac Sim目录
if [ ! -f "./python.sh" ]; then
    echo "❌ 错误: 请在Isaac Sim安装目录中运行此脚本"
    echo "💡 使用方法:"
    echo "   cd ~/isaacsim  # 或你的Isaac Sim目录"
    echo "   conda activate isaaclab_4_5_0"
    echo "   $0"
    exit 1
fi

# 检查conda环境
if [ -z "$CONDA_DEFAULT_ENV" ]; then
    echo "⚠️  警告: 未检测到conda环境"
    echo "💡 建议先激活环境: conda activate isaaclab_4_5_0"
fi

# 设置资源路径（可选）
if [ -z "$ISAAC_ASSETS_ROOT" ]; then
    echo "💡 提示: 如果遇到资源加载问题，请设置环境变量:"
    echo "   export ISAAC_ASSETS_ROOT=/path/to/your/isaac/assets"
fi

# 运行程序
echo "🎮 运行命令: ./python.sh $SCRIPTS_DIR/ultra_stable_create3.py"
exec ./python.sh "$SCRIPTS_DIR/ultra_stable_create3.py"