#!/bin/bash
# 功能：自动提取 liblcm.so 的所有依赖库，并复制到指定目录
# 使用方式：./copy_lcm_deps.sh [LCM库路径] [目标目录]
# 示例：./copy_lcm_deps.sh ~/lcm/debug/lib/liblcm.so ./lcm_all_deps

# ===================== 配置项 =====================
# 默认 LCM 库路径（若未传参，使用此默认值）
DEFAULT_LCM_LIB="$HOME/lcm/debug/lib/liblcm.so"
# 默认目标目录（若未传参，使用此默认值）
DEFAULT_DEST_DIR="./lcm_deps"

# ===================== 函数定义 =====================
# 打印彩色日志
log_info() {
    echo -e "\033[32m[INFO] $1\033[0m"
}

log_error() {
    echo -e "\033[31m[ERROR] $1\033[0m"
    exit 1
}

# 检查命令是否存在
check_command() {
    if ! command -v "$1" &> /dev/null; then
        log_error "缺少必要命令：$1，请先安装（如 sudo apt install $1）"
    fi
}

# 提取并复制依赖库
copy_dependencies() {
    local lib_path="$1"
    local dest_dir="$2"

    # 创建目标目录
    mkdir -p "$dest_dir" || log_error "创建目录失败：$dest_dir"
    log_info "目标目录已创建：$dest_dir"

    # 复制 liblcm.so 本身
    cp -fv "$lib_path" "$dest_dir/" || log_error "复制 liblcm.so 失败"
    log_info "已复制 liblcm.so 到目标目录"

    # 提取依赖库路径（过滤掉空行、系统虚拟库、非文件路径）
    ldd "$lib_path" | awk '{print $3}' | grep -v -E '^$|^\(|linux-vdso|ld-linux' | while read -r dep_lib; do
        if [ -f "$dep_lib" ]; then
            # 复制依赖库（-n 避免覆盖已存在的文件）
            cp -fnv "$dep_lib" "$dest_dir/"
            log_info "已复制依赖库：$dep_lib"
        else
            log_info "跳过无效依赖路径：$dep_lib"
        fi
    done

    # 可选：修改 liblcm.so 的 RPATH，让其优先加载当前目录的依赖
    if command -v patchelf &> /dev/null; then
        patchelf --set-rpath '$ORIGIN' "$dest_dir/liblcm.so"
        log_info "已修改 liblcm.so 的 RPATH 为 \$ORIGIN（优先加载当前目录依赖）"
    else
        log_info "未安装 patchelf，跳过 RPATH 修改（可选：sudo apt install patchelf）"
    fi
}

# ===================== 主逻辑 =====================
# 检查必要命令
check_command "ldd"
check_command "cp"
check_command "mkdir"

# 处理入参
LCM_LIB="${1:-$DEFAULT_LCM_LIB}"
DEST_DIR="${2:-$DEFAULT_DEST_DIR}"

# 检查 liblcm.so 是否存在
if [ ! -f "$LCM_LIB" ]; then
    log_error "LCM 库文件不存在：$LCM_LIB，请检查路径是否正确"
fi

# 开始复制依赖
log_info "开始提取 liblcm.so 依赖：$LCM_LIB"
copy_dependencies "$LCM_LIB" "$DEST_DIR"

# 完成提示
log_info "依赖库复制完成！所有文件已保存到：$DEST_DIR"
log_info "目录内容："
ls -lh "$DEST_DIR"
