#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# build_install_hixl.sh —— 一键编译并安装 HIXL 算子（数据面前置）
#
# 给"不清楚怎么编译"的人用：自动探测路径 → 检查前置 → 让 CANN 满足要求
# （按需调用 scripts/hixl_union.sh）→ 编译 → 验证产物 → 安装到 CANN 的 opp
# 路径 → 验证安装 → 打印下一步。全程有明确的分步提示与可执行的排错建议。
#
# 典型用法：
#   bash scripts/build_install_hixl.sh                       # 全自动（推荐先试这个）
#   bash scripts/build_install_hixl.sh --cann /usr/local/Ascend/ascend-toolkit/latest
#   bash scripts/build_install_hixl.sh --hixl ../hixl --build-only
#   bash scripts/build_install_hixl.sh --install-cmake      # 本机没 cmake 时让脚本代装
#   bash scripts/build_install_hixl.sh --official-installer # 用官方 .run 装整套 hixl
#
# 选项：--hixl/--cann/--cmake <目录>、--union-out、--metadef/--geexec/--bisheng、
#       --jobs N、--build-only、--no-union、--install-cmake、--official-installer、-h
#
# 产出（安装成功后 spdk_hcomm 运行期即可加载 HixlBatchSend）：
#   $CANN/opp/built-in/op_impl/aicpu/config/libcann_hixl_kernel.json
#   $CANN/opp/built-in/op_impl/aicpu/kernel/cann-hixl-compat.tar.gz
#
# 安装方式（默认 = 最小化拷贝，只落上面两个产物）：
#   本项目只需要上述两个文件，故默认**只拷这两个**，不碰 CANN 目录的其它内容。
#   官方 .run 装的是"整套 hixl"（含 python wheel、软链脚本、x86_64-linux 脚手架…）；
#   实测它还会在安装目标里建 `x86_64-linux/`、`hixl/`、`var/` 等目录并覆盖同名软链——
#   若目标是 union 那种软链目录，会把 union 弄坏（`include/ge` 被指向空目录 → 下次编译
#   找不到 ge_allocator.h）。所以它改成显式选项，且脚本会拒绝把它装进软链目录。
# ---------------------------------------------------------------------------
set -uo pipefail

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SELF_DIR")"          # spdk_hcomm 仓库根
WS_DIR="$(dirname "$REPO_DIR")"            # 工作区（三个仓库的父目录）

HIXL_DIR=""; CANN_DIR=""; CMAKE_BIN=""; UNION_OUT=""
METADEF=""; GEEXEC=""; BISHENG=""
JOBS="$(nproc 2>/dev/null || echo 4)"
BUILD_LOG_EARLY="/tmp/build_install_hixl_probe.log"
DO_INSTALL=1; SKIP_UNION=0; INSTALL_CMAKE=0; OFFICIAL_INSTALLER=0

# ---------- 输出助手 ----------
c_r=$'\033[31m'; c_g=$'\033[32m'; c_y=$'\033[33m'; c_b=$'\033[36m'; c_0=$'\033[0m'
step() { echo; echo "${c_b}==== $* ====${c_0}"; }
ok()   { echo "  ${c_g}✔${c_0} $*"; }
info() { echo "  · $*"; }
warn() { echo "  ${c_y}!${c_0} $*"; }
die()  { echo >&2; echo "${c_r}✘ 失败：$1${c_0}" >&2; [ -n "${2:-}" ] && echo "  → $2" >&2; exit 1; }

usage() { awk 'NR>1 { if ($0 ~ /^set -uo/) exit; print }' "$0"; }

while [ $# -gt 0 ]; do
  case "$1" in
    --hixl)      HIXL_DIR="$2"; shift 2 ;;
    --cann)      CANN_DIR="$2"; shift 2 ;;
    --cmake)     CMAKE_BIN="$2"; shift 2 ;;
    --union-out) UNION_OUT="$2"; shift 2 ;;
    --metadef)   METADEF="$2"; shift 2 ;;
    --geexec)    GEEXEC="$2"; shift 2 ;;
    --bisheng)   BISHENG="$2"; shift 2 ;;
    --jobs)      JOBS="$2"; shift 2 ;;
    --build-only) DO_INSTALL=0; shift ;;
    --no-union)  SKIP_UNION=1; shift ;;
    --install-cmake) INSTALL_CMAKE=1; shift ;;
    --official-installer) OFFICIAL_INSTALLER=1; shift ;;
    -h|--help)   usage; exit 0 ;;
    *) die "未知参数 $1" "用 --help 看用法" ;;
  esac
done

abs() { if [ -e "$1" ]; then (cd "$1" && pwd); else echo "$1"; fi; }

step "0/6 探测路径"
# --- hixl 仓库 ---
if [ -z "$HIXL_DIR" ]; then
  for cand in "$WS_DIR/hixl" "$REPO_DIR/../hixl" "$PWD/hixl" "$PWD"; do
    [ -d "$cand/src/ops/hixl_kernel" ] && { HIXL_DIR="$cand"; break; }
  done
fi
[ -n "$HIXL_DIR" ] && [ -d "$HIXL_DIR/src/ops/hixl_kernel" ] \
  || die "找不到 hixl 仓库" "用 --hixl <hixl 仓库路径> 指定（应含 src/ops/hixl_kernel/）"
HIXL_DIR="$(abs "$HIXL_DIR")"
ok "hixl 仓库      : $HIXL_DIR"

# --- CANN 目录 ---
if [ -z "$CANN_DIR" ] && [ -n "${ASCEND_HOME_PATH:-}" ]; then CANN_DIR="$ASCEND_HOME_PATH"; fi
if [ -z "$CANN_DIR" ]; then
  for cand in /usr/local/Ascend/ascend-toolkit/latest /usr/local/Ascend/latest "$HOME/Ascend/ascend-toolkit/latest"; do
    [ -d "$cand/include/hcomm" ] && { CANN_DIR="$cand"; break; }
  done
fi
[ -n "$CANN_DIR" ] && [ -d "$CANN_DIR" ] \
  || die "找不到 CANN 目录" "用 --cann <CANN 安装目录> 指定（即 ASCEND_HOME_PATH）"
CANN_DIR="$(abs "$CANN_DIR")"
ok "CANN 目录      : $CANN_DIR"
[ -n "${ASCEND_HOME_PATH:-}" ] && [ "$ASCEND_HOME_PATH" != "$CANN_DIR" ] && \
  warn "注意：环境里 ASCEND_HOME_PATH=$ASCEND_HOME_PATH 与本次使用的 $CANN_DIR 不一致——如果都用于同一套部署，建议统一。"

# --- cmake（找“能真正跑起来”的那一个；pip 的顶层 bin/cmake 常是需 PYTHONPATH 的包装器）---
cmake_ok() { [ -n "${1:-}" ] && [ -x "$1" ] && "$1" --version >/dev/null 2>&1; }

find_cmake_dir() {   # 成功则 echo 出“含可用 cmake 的目录”
  local c d
  # 1) PATH 里的（正常安装最常见）
  c="$(command -v cmake 2>/dev/null || true)"
  if cmake_ok "$c"; then dirname "$(readlink -f "$c")"; return 0; fi
  # 2) pip --target 布局：<top>/bin/cmake 是包装器，真二进制在 <top>/cmake/data/bin/cmake
  if [ -n "$c" ]; then
    d="$(dirname "$(dirname "$(readlink -f "$c")")")/cmake/data/bin/cmake"
    if cmake_ok "$d"; then dirname "$d"; return 0; fi
  fi
  # 3) 直接问 python 的 cmake 包在哪（pip 正常安装时可用）
  d="$(python3 - <<'EOP' 2>/dev/null
import os
try:
    import cmake
    p = os.path.join(os.path.dirname(cmake.__file__), "data", "bin", "cmake")
    print(os.path.dirname(p) if os.access(p, os.X_OK) else "")
except Exception:
    print("")
EOP
)"
  if cmake_ok "$d/cmake"; then echo "$d"; return 0; fi
  # 4) 常见安装位置
  for d in "$HOME/.local/bin" /usr/local/bin /usr/bin /opt/cmake/bin /opt/cmake*/bin; do
    if cmake_ok "$d/cmake"; then echo "$d"; return 0; fi
  done
  # 5) 有界搜索（工作区 / 家目录 / opt / usr/local），兜住偏门布局
  for root in "$WS_DIR" "$HOME" /opt /usr/local; do
    [ -d "$root" ] || continue
    d="$(find "$root" -maxdepth 6 -type f -path "*cmake/data/bin/cmake" 2>/dev/null | head -1)"
    if [ -n "$d" ] && cmake_ok "$d"; then dirname "$d"; return 0; fi
  done
  return 1
}

if [ -n "$CMAKE_BIN" ]; then
  case "$CMAKE_BIN" in */cmake) CMAKE_DIR="$(dirname "$CMAKE_BIN")" ;; *) CMAKE_DIR="$CMAKE_BIN" ;; esac
  cmake_ok "$CMAKE_DIR/cmake" || die "--cmake 指的地方没有可用的 cmake：$CMAKE_DIR" "传『目录』（含可执行的 cmake）"
else
  CMAKE_DIR="$(find_cmake_dir || true)"
fi
if ! cmake_ok "${CMAKE_DIR:-}/cmake" && [ "$INSTALL_CMAKE" -eq 1 ]; then
  info "未找到 cmake，尝试 pip3 install --user cmake ..."
  if pip3 install --user cmake >> "$BUILD_LOG_EARLY" 2>&1 || pip3 install --user cmake >> /dev/null 2>&1; then
    CMAKE_DIR="$(find_cmake_dir || true)"
  fi
fi
cmake_ok "${CMAKE_DIR:-}/cmake" || die "找不到可用的 cmake（>=3.16）" \
  "三选一：① 让脚本代装：加 --install-cmake 重跑；② 自己装：pip3 install --user cmake 或 apt install cmake；③ 指路径：--cmake <含可执行 cmake 的目录>"
ok "cmake          : $CMAKE_DIR/cmake ($("$CMAKE_DIR/cmake" --version 2>/dev/null | head -1))"

ARCH="$(uname -m)"
ok "架构 / 并行度  : $ARCH / -j$JOBS"

# --official-installer 的前置守卫：放在编译前，避免白等十几分钟才报错
if [ "$OFFICIAL_INSTALLER" -eq 1 ]; then
  # union 的特征就是 include/ 下挂着指向**目录外**的软链；真机安装不会是这种形态
  ge_target="$(readlink -f "$CANN_DIR/include/ge" 2>/dev/null || true)"
  case "$ge_target" in
    ""|"$CANN_DIR"/*) ;;
    *) die "拒绝把官方安装器装进软链目录：$CANN_DIR" \
           "该目录的 include/ge 指向目录外的 $ge_target，是 union/软链目录；安装器会覆盖其软链并弄坏它。请指向真正的 CANN 安装目录，或去掉 --official-installer 用默认的最小化拷贝" ;;
  esac
fi

step "1/6 前置检查（CANN 是否满足 hixl 的 find_package）"
[ -f "$SELF_DIR/hixl_union.sh" ] || die "缺 $SELF_DIR/hixl_union.sh" "它与本脚本同目录，应随仓库一起 clone"
UNION_ARGS=(--cann "$CANN_DIR" --arch "$ARCH")
[ -n "$UNION_OUT" ] && UNION_ARGS+=(--out "$UNION_OUT")
[ -n "$METADEF" ] && UNION_ARGS+=(--metadef "$METADEF")
[ -n "$GEEXEC"  ] && UNION_ARGS+=(--geexec  "$GEEXEC")
[ -n "$BISHENG" ] && UNION_ARGS+=(--bisheng "$BISHENG")

if [ "$SKIP_UNION" -eq 1 ]; then
  AICANN="$CANN_DIR"; info "已指定 --no-union，直接用 $CANN_DIR 作为 ASCEND_HOME_PATH"
else
  union_log="$(mktemp)"
  bash "$SELF_DIR/hixl_union.sh" "${UNION_ARGS[@]}" > "$union_log" 2>&1
  union_rc=$?
  AICANN="$(sed -n 's/^ASCEND_HOME_PATH=//p' "$union_log" | tail -1)"
  if [ $union_rc -ne 0 ] || [ -z "$AICANN" ]; then
    echo "--- hixl_union.sh 输出 ---"; cat "$union_log"; echo "-------------------------"
    rm -f "$union_log"
    die "CANN 缺件无法自动补齐" \
        "CANN 若是『手工合并的子集』，请把缺的包解出来，用 --metadef/--geexec/--bisheng 指过来；完整安装的 CANN 通常不需要这一步（详见 hixl_kernel_build_install.md §2）"
  fi
  if [ "$AICANN" = "$CANN_DIR" ]; then ok "CANN 已直接满足，无需 union"
  else ok "已生成 union：$AICANN"; fi
  rm -f "$union_log"
fi

step "2/6 编译 hixl"
BUILD_LOG="$HIXL_DIR/build_install.log"
info "日志：$BUILD_LOG"
info "执行：ASCEND_HOME_PATH=$AICANN bash build.sh -j$JOBS  （大版本首次编译约 5~15 分钟）"
( cd "$HIXL_DIR" && rm -rf build build_out && \
  ASCEND_HOME_PATH="$AICANN" PATH="$CMAKE_DIR:$PATH" \
  bash build.sh -j"$JOBS" ) > "$BUILD_LOG" 2>&1
rc=$?
if [ $rc -ne 0 ] || ! grep -q "Build success!" "$BUILD_LOG"; then
  echo; echo "--- 构建日志尾部（完整日志见 $BUILD_LOG）---"; tail -25 "$BUILD_LOG"; echo "-----"
  die "hixl 编译失败" "常见原因见 hixl_kernel_build_install.md §7 排障表；日志里搜 'Error' / 'Could NOT find' / 'fatal error'"
fi
ok "编译成功（Build success!）"

step "3/6 验证产物（4 项）"
L="$HIXL_DIR/build/device_install/lib/libcann_hixl_kernel.so"
J="$HIXL_DIR/build/device_install/hixl/aicpu_kernel/libcann_hixl_kernel.json"
T="$HIXL_DIR/build/device_install/hixl/aicpu_kernel/cann-hixl-compat.tar.gz"
[ -f "$L" ] || die "缺 $L" "编译未产出算子；看 $BUILD_LOG"
[ -f "$J" ] || die "缺 $J" "算子注册信息未生成"
[ -f "$T" ] || die "缺 $T" "设备侧内核包未生成"
ok "产物齐备（.so / .json / .tar.gz）"

nm -D --defined-only "$L" 2>/dev/null | grep -q " T HixlBatchSend" \
  && ok "导出 T HixlBatchSend" \
  || die "算子未导出 HixlBatchSend" "构建方式异常，检查 $BUILD_LOG"

grep -q "HixlBatchSend" "$J" \
  && ok "JSON 已登记 HixlBatchSend" \
  || die "JSON 里没有 HixlBatchSend" "检查 src/ops/hixl_kernel/libcann_hixl_kernel.ini"

undef="$(nm -D -u "$L" 2>/dev/null | grep -c '^ *U Hcomm' || true)"
[ "${undef:-0}" -eq 0 ] \
  && ok "对 hcomm 的引用均为弱符号（设备侧运行时解析）" \
  || warn "有 $undef 个强未定义 Hcomm* 符号（期望全是 w）"

file "$L" 2>/dev/null | grep -qi aarch64 \
  && ok "设备侧目标为 aarch64（AICPU 在 NPU 的 ARM 核上执行，与宿主架构无关）" \
  || warn "产物不是 aarch64？请确认 CMakeLists 的交叉工具链生效"

if [ "$DO_INSTALL" -eq 0 ]; then
  step "完成（--build-only，未安装）"
  echo "  产物：$L"
  echo "  如需安装：去掉 --build-only 重跑"
  exit 0
fi

step "4/6 安装到 CANN opp 路径"
DEST="$CANN_DIR/opp/built-in/op_impl/aicpu"
RUNPKG="$(ls "$HIXL_DIR"/build_out/cann-hixl_*.run 2>/dev/null | head -1)"

# 本项目只需要 kernel/config 两个产物 → 默认只拷这两个，不动 CANN 目录的其它内容。
# 官方 .run 会往安装目标里塞整套 hixl 脚手架（x86_64-linux/、hixl/、var/、卸载脚本…）
# 并覆盖同名软链，所以它只在 --official-installer 时使用，且拒绝装进软链目录。
if [ "$OFFICIAL_INSTALLER" -eq 1 ]; then
  [ -n "$RUNPKG" ] || die "没找到官方安装包" "期望 $HIXL_DIR/build_out/cann-hixl_*.run；去掉 --official-installer 可跳过它"
  info "用官方安装包：$(basename "$RUNPKG") --run --install-path=$CANN_DIR"
  if ( cd "$HIXL_DIR" && "$RUNPKG" --run --install-path="$CANN_DIR" --quiet ) >> "$BUILD_LOG" 2>&1; then
    ok "官方安装器执行完成"
  else
    # 官方包顺序是「先落算子产物、最后装 python wheel」，wheel 常被系统 Python 的
    # PEP 668 externally-managed-environment 拦下——那与本项目无关，产物其实已就位。
    warn "官方安装器返回非 0（常见：装 python wheel 被 PEP 668 拦下，无关本项目）"
  fi
else
  if [ ! -w "$CANN_DIR" ] && [ ! -w "$DEST" ]; then
    die "无写权限：$CANN_DIR" "用 sudo 重跑，或让管理员执行；也可先用 --build-only 只出包"
  fi
  info "最小化拷贝（只落地 SPDK 需要的两个产物）"
  install -d "$DEST/kernel" "$DEST/config" 2>/dev/null
  cp -f "$T" "$DEST/kernel/"      || die "拷贝 tar.gz 失败：$DEST/kernel" "检查权限"
  cp -f "$J" "$DEST/config/"      || die "拷贝 json 失败：$DEST/config" "检查权限"
  ok "已拷入 $DEST/{kernel,config}"
fi

step "5/6 验证安装"
fail=0
for f in "$DEST/config/libcann_hixl_kernel.json" "$DEST/kernel/cann-hixl-compat.tar.gz"; do
  if [ -f "$f" ]; then ok "$f"; else warn "缺 $f"; fail=1; fi
done
[ "$fail" -eq 0 ] || die "安装不完整" "把 $DEST 下的实际内容贴出来，或改用手动拷贝再核对"

step "6/6 完成 —— 下一步"
cat <<EOF

  ${c_g}HIXL 算子已就绪。${c_0}

  运行 spdk_hcomm 时，必须让 ${c_y}ASCEND_HOME_PATH${c_0} 指向本次安装的 CANN 目录
  （SPDK 用它拼算子 JSON 路径；不设会回落到 /usr/local/Ascend/cann，通常对不上）：

      export ASCEND_HOME_PATH="$CANN_DIR"

  然后启动应用，日志里应看到（SPDK_LOG_FLAGS=nvme_hcomm SPDK_LOG_LEVEL=debug）：

      loaded HixlBatchSend kernel: bin=... func=...

  没有这一行 → 算子未被加载，send 会返回 -EIO 并打
  "aclrtBinaryLoadFromFile(...) failed"，报错里会打印它找的具体路径，
  对照上面的 \$CANN_DIR 即可定位。

  后端说明：CANN 的 TSD 侧走 ASCEND_AICPU_PATH（默认 /usr/local/Ascend/）扫描
  opp/built-in/ 并解压 *-compat.tar.gz 到 /usr/lib64/aicpu_kernels/<vfId>/
  aicpu_kernels_device/，kernelSo=libcann_hixl_kernel.so 由此解析。若运行期仍加载
  失败，请确认 CANN 环境脚本已 source（它同时导出 ASCEND_HOME_PATH 与
  ASCEND_AICPU_PATH，使二者一致）。

  完整排障见 hixl_kernel_build_install.md §7。
EOF
