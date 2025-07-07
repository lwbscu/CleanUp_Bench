#!/bin/bash

set -e

error_exit()
{
    echo "There was an error running OSGT cleanup system python"
    exit 1
}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 设置OSGT系统资源路径 - 使用相对路径或环境变量
# 用户可以通过环境变量自定义路径，否则使用默认值
if [ -z "$ISAAC_ASSETS_ROOT" ]; then
    # 尝试自动检测常见的Isaac Sim资源路径
    POSSIBLE_PATHS=(
        "$HOME/isaacsim_assets/Assets/Isaac/4.5"
        "$HOME/.local/share/ov/pkg/isaac_sim-*/assets/Isaac/4.5"
        "/opt/isaac_sim/assets/Isaac/4.5"
        "/usr/local/isaac_sim/assets/Isaac/4.5"
        "$HOME/Isaac/4.5"
        "/Isaac/4.5"
    )
    
    echo "🔍 自动检测Isaac Sim资源路径..."
    
    for path in "${POSSIBLE_PATHS[@]}"; do
        # 处理通配符路径
        if [[ "$path" == *"*"* ]]; then
            # 使用glob展开
            for expanded_path in $path; do
                if [ -d "$expanded_path" ]; then
                    export ISAAC_ASSETS_ROOT="$expanded_path"
                    echo "✅ 自动检测到Isaac资源路径: $ISAAC_ASSETS_ROOT"
                    break 2
                fi
            done
        else
            if [ -d "$path" ]; then
                export ISAAC_ASSETS_ROOT="$path"
                echo "✅ 自动检测到Isaac资源路径: $ISAAC_ASSETS_ROOT"
                break
            fi
        fi
    done
    
    # 如果还是没找到，使用默认路径（用户需要自己设置）
    if [ -z "$ISAAC_ASSETS_ROOT" ]; then
        export ISAAC_ASSETS_ROOT="$HOME/isaacsim_assets/Assets/Isaac/4.5"
        echo "⚠️ 使用默认Isaac资源路径: $ISAAC_ASSETS_ROOT"
        echo "💡 如果路径不正确，请设置环境变量 ISAAC_ASSETS_ROOT"
        echo "💡 或在OSGT配置文件 config.py 中设置正确路径"
    fi
fi

# 验证资源路径是否存在
if [ ! -d "$ISAAC_ASSETS_ROOT" ]; then
    echo "❌ Isaac资源路径不存在: $ISAAC_ASSETS_ROOT"
    echo "💡 请检查Isaac Sim是否正确安装"
    echo "💡 或设置正确的环境变量 ISAAC_ASSETS_ROOT"
fi

# 检查住宅资产包
RESIDENTIAL_PATH="$ISAAC_ASSETS_ROOT/NVIDIA/Assets/ArchVis/Residential"
if [ -d "$RESIDENTIAL_PATH" ]; then
    ASSET_COUNT=$(find "$RESIDENTIAL_PATH" -name "*.usd" 2>/dev/null | wc -l)
    echo "✅ 住宅资产包: $ASSET_COUNT 个USD文件"
else
    echo "⚠️ 住宅资产包路径不存在: $RESIDENTIAL_PATH"
    echo "💡 OSGT系统需要住宅资产包支持四类物体场景"
fi

# 检查机器人模型
ROBOT_PATH="$ISAAC_ASSETS_ROOT/Isaac/Robots/iRobot/create_3_with_arm.usd"
if [ -f "$ROBOT_PATH" ]; then
    ROBOT_SIZE=$(du -h "$ROBOT_PATH" 2>/dev/null | cut -f1)
    echo "✅ Create-3+机械臂模型: $ROBOT_SIZE"
else
    echo "⚠️ Create-3+机械臂模型不存在: $ROBOT_PATH"
    echo "💡 OSGT系统需要Create-3+机械臂模型支持G类精确抓取"
fi

# 设置Isaac Sim环境变量
export CARB_SETTINGS__PERSISTENT__ISAAC__ASSET_ROOT__DEFAULT="${ISAAC_ASSETS_ROOT}"
export CARB_SETTINGS__PERSISTENT__ISAAC__ASSET_ROOT__CLOUD="${ISAAC_ASSETS_ROOT}"
export CARB_SETTINGS__PERSISTENT__ISAAC__ASSET_ROOT__NVIDIA="${ISAAC_ASSETS_ROOT}"

echo "🔧 Isaac Sim环境变量已设置"

# Setup python env from generated file
export CARB_APP_PATH=$SCRIPT_DIR/kit
export ISAAC_PATH=$SCRIPT_DIR
export EXP_PATH=$SCRIPT_DIR/apps

# 检查并加载Python环境设置
if [ -f "${SCRIPT_DIR}/setup_python_env.sh" ]; then
    source ${SCRIPT_DIR}/setup_python_env.sh
    echo "✅ Isaac Sim Python环境已加载"
else
    echo "⚠️ 未找到setup_python_env.sh，使用默认Python环境"
fi

# By default use our python, but allow overriding it
python_exe=${PYTHONEXE:-"python"}

# 检查conda环境
if ! [[ -z "${CONDA_PREFIX}" ]]; then
  echo "⚠️ 运行在conda环境中: ${CONDA_DEFAULT_ENV:-未知}"
  echo "💡 建议: 如果遇到问题，请先deactivate conda环境"
  echo "💡 或者在Python 3.10 conda环境中运行 source setup_conda_env.sh"
fi

# Check if we are running in a docker container
if [ -f /.dockerenv ]; then
  echo "🐳 检测到Docker环境"
  # Check for vulkan in docker container
  if [[ -f "${SCRIPT_DIR}/vulkan_check.sh" ]]; then
    ${SCRIPT_DIR}/vulkan_check.sh
  fi
fi

# 检查CUDA环境（OSGT系统可选但推荐）
if command -v nvidia-smi &> /dev/null; then
    GPU_INFO=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    if [ ! -z "$GPU_INFO" ]; then
        echo "🚀 GPU加速可用: $GPU_INFO"
        echo "💡 OSGT系统将启用CUDA加速路径规划和抓取"
    fi
else
    echo "⚠️ 未检测到NVIDIA GPU，OSGT系统将使用CPU模式"
fi

# Show icon if not running headless
export RESOURCE_NAME="OSGT_CleanupSystem"

# WAR for missing libcarb.so
if [ -f "$SCRIPT_DIR/kit/libcarb.so" ]; then
    export LD_PRELOAD=$SCRIPT_DIR/kit/libcarb.so
fi

# 打印OSGT系统信息
echo ""
echo "🏠 OSGT四类物体室内清洁系统"
echo "🚧 O类-障碍物避让 | 🧹 S类-可清扫物吸附 | 🦾 G类-可抓取物精确操作 | 🎯 T类-任务区交互"
echo "📍 Isaac Sim 资源路径: ${ISAAC_ASSETS_ROOT}"
echo ""

# 执行Python程序
$python_exe "$@" $args || error_exit