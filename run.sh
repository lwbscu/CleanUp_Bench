#!/bin/bash

# CleanUp_Bench 启动脚本
# 用于运行 Create-3+机械臂室内清洁演示
# 使用配置文件自动检测路径，支持多用户环境

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

print_header "CleanUp_Bench Create-3+机械臂室内清洁系统启动器"
print_info "项目目录: $SCRIPT_DIR"

# 检查用户名配置
CURRENT_USER=${USER:-${USERNAME:-$(whoami)}}
print_info "当前用户: $CURRENT_USER"

# 检查必要文件
if [ ! -f "$SCRIPTS_DIR/ultra_stable_create3.py" ]; then
    print_error "找不到主程序文件: $SCRIPTS_DIR/ultra_stable_create3.py"
    exit 1
fi

if [ ! -f "$SCRIPTS_DIR/config.py" ]; then
    print_error "找不到配置文件: $SCRIPTS_DIR/config.py"
    exit 1
fi

print_success "主程序和配置文件检查通过"

# 使用Python检测配置的路径
print_info "使用配置文件检测系统路径..."

PYTHON_CHECK_SCRIPT="
import sys
sys.path.insert(0, '$SCRIPTS_DIR')
try:
    from config import CleanupSystemConfig
    config = CleanupSystemConfig()
    
    print('RESIDENTIAL_ASSETS=' + config.PATHS['residential_assets_root'])
    print('ROBOT_USD=' + config.PATHS['robot_usd_path'])
    print('ISAAC_SIM_PATH=' + config.USER_PATHS['isaac_sim_install'])
    print('USERNAME=' + config.USERNAME)
    
    # 输出验证结果
    if hasattr(config, '_path_validation_results'):
        valid_paths = sum(1 for status in config._path_validation_results.values() if '✅' in status)
        total_paths = len(config._path_validation_results)
        print('VALIDATION_RESULT=' + str(valid_paths) + '/' + str(total_paths))
    else:
        print('VALIDATION_RESULT=unknown')
    
except Exception as e:
    print('ERROR: Failed to load config: ' + str(e))
    sys.exit(1)
"

# 执行Python检测脚本
DETECTION_OUTPUT=$(python3 -c "$PYTHON_CHECK_SCRIPT" 2>/dev/null)
if [ $? -ne 0 ]; then
    print_error "配置文件检测失败，请检查Python环境和配置文件"
    print_info "请确保以下条件满足："
    print_info "  1. Python3 已安装"
    print_info "  2. config.py 文件存在且语法正确"
    print_info "  3. 路径配置正确"
    exit 1
fi

# 解析检测结果
while IFS= read -r line; do
    if [[ $line == RESIDENTIAL_ASSETS=* ]]; then
        RESIDENTIAL_ASSETS="${line#RESIDENTIAL_ASSETS=}"
    elif [[ $line == ROBOT_USD=* ]]; then
        ROBOT_USD="${line#ROBOT_USD=}"
    elif [[ $line == ISAAC_SIM_PATH=* ]]; then
        ISAAC_SIM_PATH="${line#ISAAC_SIM_PATH=}"
    elif [[ $line == USERNAME=* ]]; then
        DETECTED_USERNAME="${line#USERNAME=}"
    elif [[ $line == VALIDATION_RESULT=* ]]; then
        VALIDATION_RESULT="${line#VALIDATION_RESULT=}"
    elif [[ $line == ERROR:* ]]; then
        print_error "${line#ERROR: }"
        exit 1
    fi
done <<< "$DETECTION_OUTPUT"

print_success "配置检测完成"
print_info "检测到的配置:"
echo "  👤 用户: $DETECTED_USERNAME"
echo "  🤖 机器人模型: $ROBOT_USD"
echo "  🏠 住宅资产库: $RESIDENTIAL_ASSETS"
echo "  🔧 Isaac Sim: $ISAAC_SIM_PATH"
echo "  ✅ 路径验证: $VALIDATION_RESULT"

# 检查路径有效性
print_info "验证关键路径..."

MISSING_PATHS=0

# 检查机器人模型
if [ -f "$ROBOT_USD" ]; then
    ROBOT_SIZE=$(du -h "$ROBOT_USD" | cut -f1)
    print_success "✓ 机器人模型: create_3_with_arm.usd ($ROBOT_SIZE)"
else
    print_error "✗ 机器人模型缺失: $ROBOT_USD"
    MISSING_PATHS=$((MISSING_PATHS + 1))
fi

# 检查住宅资产库
if [ -d "$RESIDENTIAL_ASSETS" ]; then
    ASSET_COUNT=$(find "$RESIDENTIAL_ASSETS" -name "*.usd" 2>/dev/null | wc -l)
    print_success "✓ 住宅资产库: $ASSET_COUNT 个USD文件"
else
    print_error "✗ 住宅资产库缺失: $RESIDENTIAL_ASSETS"
    MISSING_PATHS=$((MISSING_PATHS + 1))
fi

# 检查Isaac Sim安装
if [ -d "$ISAAC_SIM_PATH" ]; then
    print_success "✓ Isaac Sim安装目录"
else
    print_error "✗ Isaac Sim安装目录缺失: $ISAAC_SIM_PATH"
    MISSING_PATHS=$((MISSING_PATHS + 1))
fi

if [ $MISSING_PATHS -gt 0 ]; then
    print_error "发现 $MISSING_PATHS 个路径问题"
    print_warning "请检查并更新配置文件中的路径设置"
    print_info "配置文件位置: $SCRIPTS_DIR/config.py"
    print_info "您可以："
    print_info "  1. 在config.py中手动设置正确的路径"
    print_info "  2. 确认用户名是否正确 (当前: $DETECTED_USERNAME)"
    print_info "  3. 检查Isaac Sim是否正确安装"
    print_info "  4. 检查住宅资产包是否已下载"
    
    read -p "是否仍要继续运行？(y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "已取消启动"
        exit 1
    fi
fi

print_success "路径验证完成"

# 检查conda环境
if [ -z "$CONDA_DEFAULT_ENV" ]; then
    print_warning "未检测到conda环境"
    print_info "建议激活适当的conda环境，例如:"
    print_info "  conda activate isaaclab_4_5_0"
else
    print_success "当前conda环境: $CONDA_DEFAULT_ENV"
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
print_info "系统配置摘要:"
echo "  👤 用户: $DETECTED_USERNAME"
echo "  🤖 机器人: Create-3 + Panda 7DOF 机械臂"
echo "  🏠 场景: 住宅室内环境"
echo "  🗑️ 垃圾: 12个垃圾物品 (7个小+5个大)"
echo "  🏠 家具: 6件家具 + 3本书"
echo ""
print_info "性能特性:"
echo "  🚀 CUDA GPU物理加速"
echo "  🎯 A*路径规划导航"
echo "  🦾 精确机械臂抓取"
echo "  📁 配置驱动的路径管理"
echo "  ⚡ 120Hz物理 + 60FPS渲染"
echo "  🧠 智能卡住检测和突破"
echo ""
print_info "控制说明:"
echo "  👁️ 鼠标: 拖拽旋转视角，滚轮缩放"
echo "  ⌨️ Ctrl+C: 安全退出演示"

print_info "启动命令: ./python.sh $SCRIPTS_DIR/ultra_stable_create3.py"

# 设置环境变量（保持向后兼容）
export CLEANUP_BENCH_RESIDENTIAL="$RESIDENTIAL_ASSETS"
export CLEANUP_BENCH_ROBOT="$ROBOT_USD"
export CLEANUP_BENCH_USERNAME="$DETECTED_USERNAME"

# 执行主程序
print_success "启动Create-3+机械臂室内清洁演示..."
print_info "演示将包括："
echo "  1. 🔧 配置文件加载和路径验证 (5秒)"
echo "  2. 🏠 创建室内家具场景 (30秒)"
echo "  3. 🤖 机器人系统初始化 (30秒)"
echo "  4. 🦾 机械臂姿态演示 (30秒)"
echo "  5. 🔥 小垃圾智能收集 (2-3分钟)"
echo "  6. 🦾 大垃圾精确抓取 (3-4分钟)"
echo "  7. 🏠 智能返回起点 (1分钟)"
echo "  8. 📊 收集结果统计"
echo ""
print_info "💡 配置文件: $SCRIPTS_DIR/config.py"
print_info "💡 如需修改路径或参数，请编辑配置文件"
echo ""

exec ./python.sh "$SCRIPTS_DIR/ultra_stable_create3.py"