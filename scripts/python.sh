#!/bin/bash

set -e

error_exit()
{
    echo "There was an error running python"
    exit 1
}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 设置资源路径 - 使用相对路径或环境变量
# 用户可以通过环境变量自定义路径，否则使用默认值
if [ -z "$ISAAC_ASSETS_ROOT" ]; then
    # 尝试自动检测常见的Isaac Sim资源路径
    POSSIBLE_PATHS=(
        "$HOME/isaacsim_assets/Assets/Isaac/4.5"
        "/opt/isaac_sim/assets/Isaac/4.5"
        "$HOME/Isaac/4.5"
        "/Isaac/4.5"
    )
    
    for path in "${POSSIBLE_PATHS[@]}"; do
        if [ -d "$path" ]; then
            export ISAAC_ASSETS_ROOT="$path"
            echo "自动检测到Isaac资源路径: $ISAAC_ASSETS_ROOT"
            break
        fi
    done
    
    # 如果还是没找到，使用默认路径（用户需要自己设置）
    if [ -z "$ISAAC_ASSETS_ROOT" ]; then
        export ISAAC_ASSETS_ROOT="$HOME/isaacsim_assets/Assets/Isaac/4.5"
        echo "使用默认Isaac资源路径: $ISAAC_ASSETS_ROOT"
        echo "如果路径不正确，请设置环境变量 ISAAC_ASSETS_ROOT"
    fi
fi

export CARB_SETTINGS__PERSISTENT__ISAAC__ASSET_ROOT__DEFAULT="${ISAAC_ASSETS_ROOT}"
export CARB_SETTINGS__PERSISTENT__ISAAC__ASSET_ROOT__CLOUD="${ISAAC_ASSETS_ROOT}"
export CARB_SETTINGS__PERSISTENT__ISAAC__ASSET_ROOT__NVIDIA="${ISAAC_ASSETS_ROOT}"

# Setup python env from generated file
export CARB_APP_PATH=$SCRIPT_DIR/kit
export ISAAC_PATH=$SCRIPT_DIR
export EXP_PATH=$SCRIPT_DIR/apps
source ${SCRIPT_DIR}/setup_python_env.sh

# By default use our python, but allow overriding it
python_exe=${PYTHONEXE:-"python"}

if ! [[ -z "${CONDA_PREFIX}" ]]; then
  echo "Warning: running in conda env, please deactivate before executing this script"
  echo "If conda is desired please source setup_conda_env.sh in your python 3.10 conda env and run python normally"
fi

# Check if we are running in a docker container
if [ -f /.dockerenv ]; then
  # Check for vulkan in docker container
  if [[ -f "${SCRIPT_DIR}/vulkan_check.sh" ]]; then
    ${SCRIPT_DIR}/vulkan_check.sh
  fi
fi

# Show icon if not running headless
export RESOURCE_NAME="IsaacSim"
# WAR for missing libcarb.so
export LD_PRELOAD=$SCRIPT_DIR/kit/libcarb.so

# 打印资源路径信息
echo "Isaac Sim 资源路径设置为: ${ISAAC_ASSETS_ROOT}"

$python_exe "$@" $args || error_exit