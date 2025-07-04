#!/bin/bash

# CleanUp_Bench 启动脚本
# 用于运行 Create-3+机械臂室内清洁演示
# 使用原始USD资产库，无需复制文件

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
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

print_header() {
    echo -e "${PURPLE}[HEADER]${NC} $1"
}

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SCRIPTS_DIR="$SCRIPT_DIR/scripts"
RESIDENTIAL_ASSETS="/home/lwb/isaacsim/extension_examples/CleanUp_Bench/Residential"
ROBOT_USD="/home/lwb/isaacsim_assets/Assets/Isaac/4.5/Isaac/Robots/iRobot/create_3_with_arm.usd"

print_header "CleanUp_Bench Create-3+机械臂室内清洁系统启动器"
print_info "项目目录: $SCRIPT_DIR"

# 检查必要文件
if [ ! -f "$SCRIPTS_DIR/ultra_stable_create3.py" ]; then
    print_error "找不到主程序文件: $SCRIPTS_DIR/ultra_stable_create3.py"
    exit 1
fi

print_success "主程序文件检查通过"

# 检查资产文件
print_info "验证资产文件路径..."

# 检查机器人模型
if [ ! -f "$ROBOT_USD" ]; then
    print_error "机器人模型文件不存在: $ROBOT_USD"
    print_info "请确保以下路径存在并可访问:"
    print_info "  $ROBOT_USD"
    exit 1
else
    ROBOT_SIZE=$(du -h "$ROBOT_USD" | cut -f1)
    print_success "机器人模型: create_3_with_arm.usd ($ROBOT_SIZE)"
fi

# 检查住宅资产库
if [ ! -d "$RESIDENTIAL_ASSETS" ]; then
    print_error "住宅资产库不存在: $RESIDENTIAL_ASSETS"
    print_info "请确保CleanUp_Bench项目和Residential资产包已正确安装"
    exit 1
else
    ASSET_COUNT=$(find "$RESIDENTIAL_ASSETS" -name "*.usd" | wc -l)
    print_success "住宅资产库: $ASSET_COUNT 个USD文件"
fi

# 验证关键资产文件
print_info "验证关键资产文件..."
CRITICAL_ASSETS=(
    "$RESIDENTIAL_ASSETS/Furniture/Desks/Desk_01.usd"
    "$RESIDENTIAL_ASSETS/Furniture/Chairs/Chair_Desk.usd"
    "$RESIDENTIAL_ASSETS/Furniture/CoffeeTables/Midtown.usd"
    "$RESIDENTIAL_ASSETS/Decor/Tchotchkes/Orange_01.usd"
    "$RESIDENTIAL_ASSETS/Food/Containers/TinCan.usd"
    "$RESIDENTIAL_ASSETS/Misc/Supplies/Eraser.usd"
    "$RESIDENTIAL_ASSETS/Entertainment/Games/Solid_Marble.usd"
)

MISSING_ASSETS=0
for asset in "${CRITICAL_ASSETS[@]}"; do
    if [ -f "$asset" ]; then
        ASSET_SIZE=$(du -h "$asset" | cut -f1)
        print_success "✓ $(basename "$asset") ($ASSET_SIZE)"
    else
        print_error "✗ 缺失: $(basename "$asset")"
        print_error "   完整路径: $asset"
        MISSING_ASSETS=$((MISSING_ASSETS + 1))
    fi
done

if [ $MISSING_ASSETS -gt 0 ]; then
    print_error "缺失 $MISSING_ASSETS 个关键资产文件"
    print_info "请检查CleanUp_Bench项目是否完整安装"
    exit 1
fi

print_success "所有关键资产文件验证通过"

# 检查Isaac Sim安装
print_info "检查Isaac Sim安装..."
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
else
    print_success "当前conda环境: $CONDA_DEFAULT_ENV"
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

# 检查Python环境和依赖
print_info "检查Python环境..."
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version 2>&1 | cut -d' ' -f2)
    print_success "Python版本: $PYTHON_VERSION"
else
    print_error "未找到Python3"
    exit 1
fi

# 检查CUDA（如果可用）
print_info "检查CUDA环境..."
if command -v nvidia-smi &> /dev/null; then
    GPU_INFO=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)
    print_success "GPU: $GPU_INFO"
    CUDA_VERSION=$(nvidia-smi | grep "CUDA Version" | sed 's/.*CUDA Version: \([0-9.]*\).*/\1/')
    if [ ! -z "$CUDA_VERSION" ]; then
        print_success "CUDA版本: $CUDA_VERSION"
    fi
else
    print_warning "未检测到NVIDIA GPU或驱动"
fi

# 进入Isaac Sim目录
print_info "切换到Isaac Sim目录: $ISAAC_SIM_PATH"
cd "$ISAAC_SIM_PATH"

# 检查Python启动脚本
if [ ! -f "$ISAAC_SIM_PATH/python.sh" ]; then
    print_warning "Isaac Sim目录中未找到python.sh"
    
    # 创建简单的python.sh脚本
    cat > "$ISAAC_SIM_PATH/python.sh" << 'EOF'
#!/bin/bash
# 简单的Isaac Sim Python启动脚本
exec python "$@"
EOF
    chmod +x "$ISAAC_SIM_PATH/python.sh"
    print_success "创建了python.sh启动脚本"
fi

# 显示启动信息
print_header "准备启动室内清洁演示..."
print_info "系统配置:"
echo "  🤖 机器人: Create-3 + Panda 7DOF 机械臂"
echo "  🏠 场景: 住宅室内环境"
echo "  🗑️ 垃圾: 12个垃圾物品 (7个小+5个大)"
echo "  🏠 家具: 6件家具 + 3本书"
echo ""
print_info "性能特性:"
echo "  🚀 CUDA GPU物理加速"
echo "  🎯 A*路径规划导航"
echo "  🦾 精确机械臂抓取"
echo "  📁 原始USD资产库 (无复制)"
echo "  ⚡ 120Hz物理 + 60FPS渲染"
echo "  🧠 智能卡住检测和突破"
echo ""
print_info "控制说明:"
echo "  👁️ 鼠标: 拖拽旋转视角，滚轮缩放"
echo "  ⌨️ Ctrl+C: 安全退出演示"

print_info "启动命令: ./python.sh $SCRIPTS_DIR/ultra_stable_create3.py"

# 设置环境变量
export CLEANUP_BENCH_RESIDENTIAL="$RESIDENTIAL_ASSETS"
export CLEANUP_BENCH_ROBOT="$ROBOT_USD"

# 执行主程序
print_success "启动Create-3+机械臂室内清洁演示..."
print_info "演示将包括："
echo "  1. 🏠 创建室内家具场景 (30秒)"
echo "  2. 🤖 机器人系统初始化 (30秒)"
echo "  3. 🦾 机械臂姿态演示 (30秒)"
echo "  4. 🔥 小垃圾智能收集 (2-3分钟)"
echo "  5. 🦾 大垃圾精确抓取 (3-4分钟)"
echo "  6. 🏠 智能返回起点 (1分钟)"
echo "  7. 📊 收集结果统计"
echo ""

exec ./python.sh "$SCRIPTS_DIR/ultra_stable_create3.py"