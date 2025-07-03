#!/bin/bash

# CleanUp_Bench 项目设置脚本
# 用于设置文件权限和检查环境

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

print_info "CleanUp_Bench 项目设置"
print_info "项目目录: $SCRIPT_DIR"

# 设置文件权限
print_info "设置脚本权限..."

# 主启动脚本
if [ -f "$SCRIPT_DIR/run.sh" ]; then
    chmod +x "$SCRIPT_DIR/run.sh"
    print_success "设置 run.sh 可执行权限"
else
    print_error "找不到 run.sh"
fi

# 简单启动脚本
if [ -f "$SCRIPT_DIR/run_simple.sh" ]; then
    chmod +x "$SCRIPT_DIR/run_simple.sh"
    print_success "设置 run_simple.sh 可执行权限"
else
    print_error "找不到 run_simple.sh"
fi

# Python启动脚本
if [ -f "$SCRIPT_DIR/scripts/python.sh" ]; then
    chmod +x "$SCRIPT_DIR/scripts/python.sh"
    print_success "设置 scripts/python.sh 可执行权限"
else
    print_error "找不到 scripts/python.sh"
fi

# 检查项目结构
print_info "检查项目文件..."

required_files=(
    "scripts/ultra_stable_create3.py"
    "scripts/python.sh"
    "run.sh"
    "run_simple.sh"
    "README.md"
)

missing_files=()
for file in "${required_files[@]}"; do
    if [ -f "$SCRIPT_DIR/$file" ]; then
        print_success "✓ $file"
    else
        print_error "✗ $file (缺失)"
        missing_files+=("$file")
    fi
done

if [ ${#missing_files[@]} -ne 0 ]; then
    print_error "项目文件不完整，缺失以下文件:"
    for file in "${missing_files[@]}"; do
        echo "  - $file"
    done
    exit 1
fi

# 检查Python脚本语法
print_info "检查Python脚本语法..."
if command -v python3 >/dev/null 2>&1; then
    if python3 -m py_compile "$SCRIPT_DIR/scripts/ultra_stable_create3.py" 2>/dev/null; then
        print_success "Python脚本语法检查通过"
    else
        print_warning "Python脚本语法检查失败，但可能是由于缺少依赖"
    fi
else
    print_warning "未找到python3，跳过语法检查"
fi

# 检查Isaac Sim环境
print_info "检查Isaac Sim环境..."

isaac_paths=(
    "$HOME/isaacsim"
    "/opt/isaac_sim"
    "$HOME/.local/share/ov/pkg/isaac_sim-*"
)

isaac_found=false
for path in "${isaac_paths[@]}"; do
    if [ -d "$path" ]; then
        print_success "找到Isaac Sim: $path"
        isaac_found=true
        break
    fi
done

if [ "$isaac_found" = false ]; then
    print_warning "未自动检测到Isaac Sim安装"
    print_info "请确保Isaac Sim已正确安装，或设置环境变量:"
    print_info "  export ISAAC_SIM_PATH=/path/to/isaac_sim"
fi

# 检查conda
print_info "检查conda环境..."
if command -v conda >/dev/null 2>&1; then
    print_success "找到conda"
    if [ ! -z "$CONDA_DEFAULT_ENV" ]; then
        print_success "当前conda环境: $CONDA_DEFAULT_ENV"
    else
        print_warning "未激活conda环境"
        print_info "建议激活Isaac环境，例如:"
        print_info "  conda activate isaaclab_4_5_0"
    fi
else
    print_warning "未找到conda"
fi

print_success "项目设置完成！"
print_info "使用方法:"
print_info "  ./run.sh          # 自动启动"
print_info "  ./run_simple.sh   # 简单启动（需在Isaac Sim目录中运行）"
print_info ""
print_info "更多信息请查看 README.md"