#!/bin/bash

# OSGT四类物体CleanUp_Bench 启动脚本
# 用于运行 Create-3+机械臂 OSGT标准室内清洁演示
# 🚧 O类-障碍物避让 | 🧹 S类-可清扫物吸附 | 🦾 G类-可抓取物精确操作 | 🎯 T类-任务区交互
# 使用配置文件自动检测路径，支持多用户环境

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
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
    echo -e "${PURPLE}[OSGT]${NC} $1"
}

print_osgt() {
    echo -e "${CYAN}[OSGT]${NC} $1"
}

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SCRIPTS_DIR="$SCRIPT_DIR/scripts"

print_header "OSGT四类物体标准 Create-3+机械臂室内清洁系统启动器"
print_osgt "🚧 O类-障碍物避让 | 🧹 S类-可清扫物吸附 | 🦾 G类-可抓取物精确操作 | 🎯 T类-任务区交互"
print_info "项目目录: $SCRIPT_DIR"

# 检查用户名配置
CURRENT_USER=${USER:-${USERNAME:-$(whoami)}}
print_info "当前用户: $CURRENT_USER"

# 检查必要文件
print_info "检查OSGT系统文件..."

REQUIRED_FILES=(
    "$SCRIPTS_DIR/ultra_stable_create3.py"
    "$SCRIPTS_DIR/config.py"
    "$SCRIPTS_DIR/advanced_navigation.py"
    "$SCRIPTS_DIR/pick_and_place.py"
)

MISSING_FILES=0
for file in "${REQUIRED_FILES[@]}"; do
    if [ ! -f "$file" ]; then
        print_error "找不到文件: $(basename $file)"
        MISSING_FILES=$((MISSING_FILES + 1))
    else
        print_success "✓ $(basename $file)"
    fi
done

if [ $MISSING_FILES -gt 0 ]; then
    print_error "缺少 $MISSING_FILES 个必需文件，请检查项目完整性"
    exit 1
fi

print_success "OSGT系统文件检查通过"

# 使用Python检测OSGT配置的路径
print_info "使用OSGT配置文件检测系统路径..."

PYTHON_CHECK_SCRIPT="
import sys
sys.path.insert(0, '$SCRIPTS_DIR')
try:
    from config import OSGTCleanupSystemConfig
    config = OSGTCleanupSystemConfig()
    
    print('RESIDENTIAL_ASSETS=' + config.PATHS['residential_assets_root'])
    print('ROBOT_USD=' + config.PATHS['robot_usd_path'])
    print('ISAAC_SIM_PATH=' + config.USER_PATHS['isaac_sim_install'])
    print('USERNAME=' + config.USERNAME)
    print('SCENARIO_TYPE=' + config.SCENARIO_TYPE)
    
    # 输出OSGT物体数量
    print('OSGT_OBSTACLES=' + str(len(config.OBSTACLES_POSITIONS)))
    print('OSGT_SWEEPABLE=' + str(len(config.SWEEPABLE_POSITIONS)))
    print('OSGT_GRASPABLE=' + str(len(config.GRASPABLE_POSITIONS)))
    print('OSGT_TASK_AREAS=' + str(len(config.TASK_AREAS_POSITIONS)))
    
    # 输出验证结果
    if hasattr(config, '_path_validation_results'):
        valid_paths = sum(1 for status in config._path_validation_results.values() if '✅' in status)
        total_paths = len(config._path_validation_results)
        print('VALIDATION_RESULT=' + str(valid_paths) + '/' + str(total_paths))
    else:
        print('VALIDATION_RESULT=unknown')
    
except Exception as e:
    print('ERROR: Failed to load OSGT config: ' + str(e))
    sys.exit(1)
"

# 执行Python检测脚本
DETECTION_OUTPUT=$(python3 -c "$PYTHON_CHECK_SCRIPT" 2>/dev/null)
if [ $? -ne 0 ]; then
    print_error "OSGT配置文件检测失败，请检查Python环境和配置文件"
    print_info "请确保以下条件满足："
    print_info "  1. Python3 已安装"
    print_info "  2. config.py 文件存在且语法正确（使用OSGTCleanupSystemConfig）"
    print_info "  3. OSGT路径配置正确"
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
    elif [[ $line == SCENARIO_TYPE=* ]]; then
        SCENARIO_TYPE="${line#SCENARIO_TYPE=}"
    elif [[ $line == OSGT_OBSTACLES=* ]]; then
        OSGT_OBSTACLES="${line#OSGT_OBSTACLES=}"
    elif [[ $line == OSGT_SWEEPABLE=* ]]; then
        OSGT_SWEEPABLE="${line#OSGT_SWEEPABLE=}"
    elif [[ $line == OSGT_GRASPABLE=* ]]; then
        OSGT_GRASPABLE="${line#OSGT_GRASPABLE=}"
    elif [[ $line == OSGT_TASK_AREAS=* ]]; then
        OSGT_TASK_AREAS="${line#OSGT_TASK_AREAS=}"
    elif [[ $line == VALIDATION_RESULT=* ]]; then
        VALIDATION_RESULT="${line#VALIDATION_RESULT=}"
    elif [[ $line == ERROR:* ]]; then
        print_error "${line#ERROR: }"
        exit 1
    fi
done <<< "$DETECTION_OUTPUT"

print_success "OSGT配置检测完成"
print_info "检测到的OSGT配置:"
echo "  👤 用户: $DETECTED_USERNAME"
echo "  🏢 场景类型: $SCENARIO_TYPE"
echo "  🤖 机器人模型: $ROBOT_USD"
echo "  🏠 住宅资产库: $RESIDENTIAL_ASSETS"
echo "  🔧 Isaac Sim: $ISAAC_SIM_PATH"
echo "  ✅ 路径验证: $VALIDATION_RESULT"

print_osgt "OSGT四类物体配置:"
echo "  🚧 O类障碍物: $OSGT_OBSTACLES 个"
echo "  🧹 S类可清扫物: $OSGT_SWEEPABLE 个"
echo "  🦾 G类可抓取物: $OSGT_GRASPABLE 个"
echo "  🎯 T类任务区: $OSGT_TASK_AREAS 个"

# 检查路径有效性
print_info "验证OSGT系统关键路径..."

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
    
    # 检查OSGT关键资产类别
    FURNITURE_COUNT=$(find "$RESIDENTIAL_ASSETS/Furniture" -name "*.usd" 2>/dev/null | wc -l)
    DECOR_COUNT=$(find "$RESIDENTIAL_ASSETS/Decor" -name "*.usd" 2>/dev/null | wc -l)
    MISC_COUNT=$(find "$RESIDENTIAL_ASSETS/Misc" -name "*.usd" 2>/dev/null | wc -l)
    
    if [ $FURNITURE_COUNT -gt 0 ]; then
        print_success "  ✓ O类障碍物资产: $FURNITURE_COUNT 个家具模型"
    fi
    if [ $DECOR_COUNT -gt 0 ]; then
        print_success "  ✓ S类/G类物品资产: $DECOR_COUNT 个装饰模型"
    fi
    if [ $MISC_COUNT -gt 0 ]; then
        print_success "  ✓ G类工具资产: $MISC_COUNT 个杂项模型"
    fi
else
    print_error "✗ 住宅资产库缺失: $RESIDENTIAL_ASSETS"
    print_warning "  OSGT系统需要住宅资产库支持四类物体场景"
    MISSING_PATHS=$((MISSING_PATHS + 1))
fi

# 检查Isaac Sim安装
if [ -d "$ISAAC_SIM_PATH" ]; then
    print_success "✓ Isaac Sim安装目录"
    
    # 检查Isaac Sim Python脚本
    if [ -f "$ISAAC_SIM_PATH/python.sh" ]; then
        print_success "  ✓ Isaac Sim Python启动脚本"
    else
        print_warning "  ⚠ python.sh 未找到，将使用系统Python"
    fi
else
    print_error "✗ Isaac Sim安装目录缺失: $ISAAC_SIM_PATH"
    MISSING_PATHS=$((MISSING_PATHS + 1))
fi

if [ $MISSING_PATHS -gt 0 ]; then
    print_error "发现 $MISSING_PATHS 个路径问题"
    print_warning "请检查并更新OSGT配置文件中的路径设置"
    print_info "配置文件位置: $SCRIPTS_DIR/config.py"
    print_info "您可以："
    print_info "  1. 在config.py中手动设置正确的路径"
    print_info "  2. 确认用户名是否正确 (当前: $DETECTED_USERNAME)"
    print_info "  3. 检查Isaac Sim是否正确安装"
    print_info "  4. 检查住宅资产包是否已下载"
    print_info "  5. 使用快速配置预设：OSGTQuickConfigs.residential_scene()"
    
    read -p "是否仍要继续运行OSGT系统？(y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "已取消OSGT系统启动"
        exit 1
    fi
fi

print_success "OSGT路径验证完成"

# 检查conda环境
if [ -z "$CONDA_DEFAULT_ENV" ]; then
    print_warning "未检测到conda环境"
    print_info "建议激活适当的conda环境，例如:"
    print_info "  conda activate isaaclab_4_5_0"
    print_info "  或者 conda activate isaac-sim"
else
    print_success "当前conda环境: $CONDA_DEFAULT_ENV"
fi

# 检查Python环境和依赖
print_info "检查OSGT Python环境..."
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version 2>&1 | cut -d' ' -f2)
    print_success "Python版本: $PYTHON_VERSION"
    
    # 检查关键Python包
    PYTHON_PACKAGES_CHECK="
import sys
try:
    import numpy as np
    print('✓ NumPy:', np.__version__)
except ImportError:
    print('✗ NumPy: 未安装')

try:
    import time, math, random, collections, heapq
    print('✓ 标准库: 完整')
except ImportError:
    print('✗ 标准库: 缺失组件')

try:
    import cupy as cp
    print('✓ CuPy:', cp.__version__, '(CUDA加速)')
except ImportError:
    print('⚠ CuPy: 未安装 (将使用CPU模式)')

try:
    from scipy.spatial.transform import Rotation
    print('✓ SciPy: 可用')
except ImportError:
    print('⚠ SciPy: 未安装 (将使用简化旋转)')
"
    echo "  检查OSGT依赖包:"
    python3 -c "$PYTHON_PACKAGES_CHECK" 2>/dev/null | sed 's/^/    /'
else
    print_error "未找到Python3"
    exit 1
fi

# 检查CUDA（如果可用）
print_info "检查OSGT CUDA环境..."
if command -v nvidia-smi &> /dev/null; then
    GPU_INFO=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    if [ ! -z "$GPU_INFO" ]; then
        print_success "GPU: $GPU_INFO"
        CUDA_VERSION=$(nvidia-smi | grep "CUDA Version" | sed 's/.*CUDA Version: \([0-9.]*\).*/\1/')
        if [ ! -z "$CUDA_VERSION" ]; then
            print_success "CUDA版本: $CUDA_VERSION"
            print_osgt "🚀 OSGT系统将启用CUDA加速路径规划和抓取算法"
        fi
    fi
else
    print_warning "未检测到NVIDIA GPU或驱动"
    print_osgt "⚡ OSGT系统将使用CPU模式（性能较低但兼容性更好）"
fi

# 进入Isaac Sim目录
print_info "切换到Isaac Sim目录: $ISAAC_SIM_PATH"
cd "$ISAAC_SIM_PATH"

# 检查Python启动脚本
if [ ! -f "$ISAAC_SIM_PATH/python.sh" ]; then
    print_warning "Isaac Sim目录中未找到python.sh"
    
    # 复制我们的OSGT优化版python.sh脚本
    if [ -f "$SCRIPTS_DIR/python.sh" ]; then
        cp "$SCRIPTS_DIR/python.sh" "$ISAAC_SIM_PATH/python.sh"
        chmod +x "$ISAAC_SIM_PATH/python.sh"
        print_success "使用OSGT优化版python.sh启动脚本"
    else
        # 创建简单的python.sh脚本
        cat > "$ISAAC_SIM_PATH/python.sh" << 'EOF'
#!/bin/bash
# 简单的Isaac Sim Python启动脚本
exec python "$@"
EOF
        chmod +x "$ISAAC_SIM_PATH/python.sh"
        print_success "创建了python.sh启动脚本"
    fi
fi

# 显示OSGT启动信息
print_header "准备启动OSGT四类物体室内清洁演示..."
print_info "OSGT系统配置摘要:"
echo "  👤 用户: $DETECTED_USERNAME"
echo "  🏢 场景: $SCENARIO_TYPE (家庭/学校/医院/工厂通用)"
echo "  🤖 机器人: Create-3 + Panda 7DOF 机械臂"
echo "  🏠 环境: 住宅室内环境"
echo ""
print_osgt "OSGT四类物体配置:"
echo "  🚧 O类障碍物: $OSGT_OBSTACLES 个 (避让导航)"
echo "  🧹 S类可清扫物: $OSGT_SWEEPABLE 个 (吸附收集)"
echo "  🦾 G类可抓取物: $OSGT_GRASPABLE 个 (精确抓取)"
echo "  🎯 T类任务区: $OSGT_TASK_AREAS 个 (任务交互)"
echo ""
print_info "OSGT性能特性:"
echo "  🚀 CUDA GPU物理加速"
echo "  🧭 OSGT四类导航策略"
echo "  🦾 类型特定精确抓取"
echo "  📁 配置驱动的路径管理"
echo "  ⚡ 120Hz物理 + 60FPS渲染"
echo "  🧠 智能OSGT物体识别"
echo "  🌐 多场景通用设计"
echo ""
print_info "控制说明:"
echo "  👁️ 鼠标: 拖拽旋转视角，滚轮缩放"
echo "  ⌨️ Ctrl+C: 安全退出演示"

print_info "启动命令: ./python.sh $SCRIPTS_DIR/ultra_stable_create3.py"

# 设置OSGT环境变量（保持向后兼容）
export CLEANUP_BENCH_RESIDENTIAL="$RESIDENTIAL_ASSETS"
export CLEANUP_BENCH_ROBOT="$ROBOT_USD"
export CLEANUP_BENCH_USERNAME="$DETECTED_USERNAME"
export OSGT_SCENARIO_TYPE="$SCENARIO_TYPE"

# 显示运行预期
print_success "启动OSGT四类物体室内清洁演示..."
print_info "演示将包括："
echo "  1. 🔧 OSGT配置文件加载和路径验证 (5秒)"
echo "  2. 🏠 创建OSGT四类物体场景 (30秒)"
echo "  3. 🤖 机器人系统初始化 (30秒)"
echo "  4. 🦾 机械臂OSGT姿态演示 (30秒)"
echo "  5. 🧹 S类可清扫物智能收集 (2-3分钟)"
echo "  6. 🦾 G类可抓取物精确抓取 (3-4分钟)"
echo "  7. 🎯 T类任务区访问交互 (1-2分钟)"
echo "  8. 🏠 智能返回起点 (1分钟)"
echo "  9. 📊 OSGT收集结果统计"
echo ""
print_osgt "🚧 O类避障 | 🧹 S类吸附 | 🦾 G类精确抓取 | 🎯 T类任务执行"
print_info "💡 OSGT配置文件: $SCRIPTS_DIR/config.py"
print_info "💡 如需修改路径或参数，请编辑配置文件"
print_info "💡 支持场景切换: residential, school, hospital, factory"
echo ""

# 执行主程序
exec ./python.sh "$SCRIPTS_DIR/ultra_stable_create3.py"