#!/bin/bash

# CleanUp_Bench 启动脚本
# 用于运行 Create-3+机械臂演示

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SCRIPTS_DIR="$SCRIPT_DIR/scripts"

print_info "CleanUp_Bench Create-3+机械臂演示启动器"
print_info "项目目录: $SCRIPT_DIR"

# 检查必要文件
if [ ! -f "$SCRIPTS_DIR/ultra_stable_create3.py" ]; then
    print_error "找不到主程序文件: $SCRIPTS_DIR/ultra_stable_create3.py"
    exit 1
fi

if [ ! -f "$SCRIPTS_DIR/python.sh" ]; then
    print_error "找不到Python启动脚本: $SCRIPTS_DIR/python.sh"
    exit 1
fi

# 检查Isaac Sim安装
ISAAC_SIM_PATHS=(
    "$HOME/isaacsim"
    "/opt/isaac_sim"
    "$HOME/.local/share/ov/pkg/isaac_sim-*"
)

ISAAC_SIM_PATH=""
for path in "${ISAAC_SIM_PATHS[@]}"; do
    if [ -d "$path" ]; then
        ISAAC_SIM_PATH="$path"
        break
    fi
done

if [ -z "$ISAAC_SIM_PATH" ]; then
    print_error "未找到Isaac Sim安装目录"
    print_info "请确保Isaac Sim已正确安装在以下位置之一:"
    for path in "${ISAAC_SIM_PATHS[@]}"; do
        echo "  - $path"
    done
    print_info "或者手动设置环境变量 ISAAC_SIM_PATH"
    exit 1
fi

# 如果用户没有设置ISAAC_SIM_PATH，则使用检测到的路径
if [ -z "$ISAAC_SIM_PATH_USER" ]; then
    export ISAAC_SIM_PATH="$ISAAC_SIM_PATH"
fi

print_success "找到Isaac Sim: $ISAAC_SIM_PATH"

# 检查conda环境
if [ -z "$CONDA_DEFAULT_ENV" ]; then
    print_warning "未检测到conda环境"
    print_info "建议激活适当的conda环境，例如:"
    print_info "  conda activate isaaclab_4_5_0"
fi

# 设置资源路径（可选）
if [ -z "$ISAAC_ASSETS_ROOT" ]; then
    ASSET_PATHS=(
        "$HOME/isaacsim_assets/Assets/Isaac/4.5"
        "$ISAAC_SIM_PATH/assets/Isaac/4.5"
        "/opt/isaac_sim/assets/Isaac/4.5"
    )
    
    for path in "${ASSET_PATHS[@]}"; do
        if [ -d "$path" ]; then
            export ISAAC_ASSETS_ROOT="$path"
            print_success "设置资源路径: $ISAAC_ASSETS_ROOT"
            break
        fi
    done
    
    if [ -z "$ISAAC_ASSETS_ROOT" ]; then
        print_warning "未找到Isaac资源文件夹，将使用默认路径"
    fi
fi

# 进入Isaac Sim目录
print_info "切换到Isaac Sim目录: $ISAAC_SIM_PATH"
cd "$ISAAC_SIM_PATH"

# 运行程序
print_info "启动Create-3+机械臂演示..."
print_info "使用命令: ./python.sh $SCRIPTS_DIR/ultra_stable_create3.py"

# 复制必要的脚本到Isaac Sim目录（如果需要）
if [ ! -f "$ISAAC_SIM_PATH/python.sh" ]; then
    print_warning "Isaac Sim目录中未找到python.sh，使用项目提供的版本"
    cp "$SCRIPTS_DIR/python.sh" "$ISAAC_SIM_PATH/"
    chmod +x "$ISAAC_SIM_PATH/python.sh"
fi

# 执行主程序
exec ./python.sh "$SCRIPTS_DIR/ultra_stable_create3.py"