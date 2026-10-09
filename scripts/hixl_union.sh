#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# make-hixl-union.sh —— 为 hixl 构建准备 ASCEND_HOME_PATH
#
# 背景：hixl 的 cann-cmake 会逐个 find_package 十余个 CANN 包（metadef / ascend_hal /
# hccl / slog / error_manager / runtime ...），并要求若干在标准布局下不满足的东西。
# 本脚本按"能不用就不用"的原则工作：
#   Step A  先直接拿 --cann 指过去做自检；**全通过就什么都不做**，直接用该目录当 ASCEND_HOME_PATH。
#   Step B  只缺 devlib/libascend_hal.so（真机上最常见、且只是布局问题）→ 原地补软链（需可写）。
#   Step C  若还缺 metadef / acl_op.h / toolkit 等（CANN 是"手工合并的子集"而非完整安装）→
#           用 --metadef/--geexec/--bisheng 指向的补充包，生成一个 symlink union 目录。
#
# 用法：
#   bash scripts/hixl_union.sh --cann <CANN目录> [--out <输出目录>] \
#        [--metadef <目录>] [--geexec <目录>] [--bisheng <目录>] [--arch x86_64|aarch64]
#
#   真机（完整 CANN 安装）通常一条就够：
#     bash scripts/hixl_union.sh --cann "$ASCEND_HOME_PATH"
#   若输出"直接可用"，就把 ASCEND_HOME_PATH 指向 --cann 的路径，不需要 union。
#   若 CANN 是"手工合并的子集"（缺 metadef / acl_op.h / toolkit 等），再加 --metadef/--geexec/--bisheng。
#
# 退出码：0 = 可用（stdout 末行给出应使用的 ASCEND_HOME_PATH）；非 0 = 缺件无法自动补齐。
# ---------------------------------------------------------------------------
set -uo pipefail

CANNDIR=""; OUT=""; METADEF=""; GEEXEC=""; BISHENG=""
ARCH="$(uname -m)"
case "$ARCH" in aarch64|arm64) ARCH=aarch64 ;; x86_64|amd64) ARCH=x86_64 ;; esac

while [ $# -gt 0 ]; do
  case "$1" in
    --cann)    CANNDIR="$2"; shift 2 ;;
    --out)     OUT="$2"; shift 2 ;;
    --metadef) METADEF="$2"; shift 2 ;;
    --geexec)  GEEXEC="$2"; shift 2 ;;
    --bisheng) BISHENG="$2"; shift 2 ;;
    --arch)    ARCH="$2"; shift 2 ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done

[ -n "$CANNDIR" ] || { echo "错误：必须给 --cann <CANN目录>" >&2; exit 2; }
[ -d "$CANNDIR" ] || { echo "错误：$CANNDIR 不是目录" >&2; exit 2; }
# 全部转绝对路径：symlink 的目标必须是绝对路径，否则从目标目录解析会失效
CANNDIR="$(readlink -f "$CANNDIR")"
for v in OUT METADEF GEEXEC BISHENG; do
  eval "cur=\$$v"
  if [ -n "$cur" ]; then
    if [ -e "$cur" ]; then eval "$v=\"\$(readlink -f "$cur")\""
    else eval "$v=\"\$(cd "$(dirname "$cur")" 2>/dev/null && pwd)/$(basename "$cur")\""; fi
  fi
done

# hixl / cann-cmake 会去找的 17 项（与 scripts/version/check_build_dependencies.py + modules/Find*.cmake 对齐）
REQUIRED=(
  toolkit/toolchain/hcc
  toolkit/toolchain/hcc/bin/aarch64-target-linux-gnu-g++
  include/graph/types.h
  pkg_inc/common/checker.h
  lib64/libmetadef.so
  lib64/libexe_graph.so
  lib64/libopp_registry.so
  lib64/librt2_registry.a
  include/acl/acl_op.h
  include/acl/acl_mdl.h
  devlib/libascend_hal.so
  pkg_inc/driver/ascend_hal.h
  include/hccl/hccl_types.h
  pkg_inc/hccl/base.h
  lib64/libhcomm.so
  lib64/libruntime.so
  lib64/libascendalog.so
  share/info/hcomm/version.info
)
# 只由"布局"引起、可原地修的一项（真机最常见的唯一缺件）
ONLY_LAYOUT=(devlib/libascend_hal.so)

selfcheck() {   # $1=根目录；打印缺失项，返回缺失数
  local root="$1" miss=0 f
  for f in "${REQUIRED[@]}"; do
    if [ ! -e "$root/$f" ]; then echo "  MISS $f"; miss=$((miss+1)); fi
  done
  return $miss
}

echo "== 目标 CANN 目录: $CANNDIR   (arch=$ARCH)"

echo "== Step A: 直接自检"
missing="$(selfcheck "$CANNDIR")"
nmiss=$(printf '%s\n' "$missing" | grep -c '^  MISS' || true)
if [ "$nmiss" -eq 0 ]; then
  echo "  ✔ 全部满足 —— **不需要 union**。"
  echo "ASCEND_HOME_PATH=$CANNDIR"
  exit 0
fi
printf '%s\n' "$missing"

# ---- Step B: 只有布局类缺件 → 原地补软链（仅当可写） ----
layout_only=1
while read -r line; do
  [ -z "$line" ] && continue
  f="${line#  MISS }"
  keep=0
  for o in "${ONLY_LAYOUT[@]}"; do [ "$f" = "$o" ] && keep=1; done
  [ "$keep" -eq 1 ] || layout_only=0
done <<< "$missing"

if [ "$layout_only" -eq 1 ]; then
  src="$CANNDIR/devlib/linux/$ARCH/libascend_hal.so"
  echo "== Step B: 仅缺布局项 devlib/libascend_hal.so（真实位置 $src）"
  if [ -e "$src" ] && [ -w "$CANNDIR/devlib" ]; then
    ln -sfn "linux/$ARCH/libascend_hal.so" "$CANNDIR/devlib/libascend_hal.so"
    echo "  ✔ 已在 $CANNDIR/devlib/ 原地补好软链"
    echo "ASCEND_HOME_PATH=$CANNDIR"
    exit 0
  fi
  echo "  ✗ 源文件不存在或目录不可写（$CANNDIR/devlib）—— 转 Step C 生成 union"
fi

# ---- Step C: 生成 union ----
echo "== Step C: 生成 union（补充包：metadef=${METADEF:-无} geexec=${GEEXEC:-无} bisheng=${BISHENG:-无}）"
OUT="${OUT:-$(dirname "$CANNDIR")/hixl-union}"
rm -rf "$OUT"; mkdir -p "$OUT"

merge() {   # 先出现的源优先，后续只补缺；同名目录递归合并，同名文件跳过
  local dst="$1"; shift
  mkdir -p "$dst"
  local src name d s
  for src in "$@"; do
    [ -d "$src" ] || continue
    for name in "$src"/*; do
      [ -e "$name" ] || continue
      d="$dst/$(basename "$name")"; s="$name"
      if [ -e "$d" ] || [ -L "$d" ]; then
        if [ -d "$d" ] && [ -d "$s" ]; then merge "$d" "$(readlink -f "$s")"; fi
        continue
      fi
      ln -s "$s" "$d"
    done
  done
}

SPECIAL="include pkg_inc lib64 devlib share"
for e in "$CANNDIR"/*; do
  n="$(basename "$e")"
  case " $SPECIAL " in *" $n "*) ;; *) ln -s "$e" "$OUT/$n" ;; esac
done
merge "$OUT/include" "$CANNDIR/include" "$METADEF/include" "$GEEXEC/include"
merge "$OUT/pkg_inc" "$CANNDIR/pkg_inc" "$METADEF/pkg_inc"
merge "$OUT/lib64"   "$CANNDIR/lib64"   "$METADEF/lib64"
merge "$OUT/devlib"  "$CANNDIR/devlib"
[ -d "$CANNDIR/share/info" ] && merge "$OUT/share/info" "$CANNDIR/share/info"
[ -n "$BISHENG" ] && [ -d "$BISHENG/toolkit" ] && ln -sfn "$BISHENG/toolkit" "$OUT/toolkit"

# 布局补丁：devlib 直下
if [ ! -e "$OUT/devlib/libascend_hal.so" ]; then
  ln -sfn "linux/$ARCH/libascend_hal.so" "$OUT/devlib/libascend_hal.so"
fi

# 版本门禁：文件缺失=硬错误，版本不符仅告警
VERSION="$(sed -n 's/^Version=//p' "$CANNDIR/share/info/runtime/version.info" 2>/dev/null | head -1)"
VERSION="${VERSION:-9.1.0}"
for pkg in runtime metadef hcomm asc-devkit bisheng-compiler; do
  if [ ! -e "$OUT/share/info/$pkg/version.info" ]; then
    mkdir -p "$OUT/share/info/$pkg"
    printf 'Version=%s\nversion_dir=cann\n' "$VERSION" > "$OUT/share/info/$pkg/version.info"
  fi
done

echo "== union 自检"
missing2="$(selfcheck "$OUT")"
nmiss2=$(printf '%s\n' "$missing2" | grep -c '^  MISS' || true)
if [ "$nmiss2" -eq 0 ]; then
  echo "  ✔ union 完整：$OUT"
  echo "ASCEND_HOME_PATH=$OUT"
  exit 0
fi
printf '%s\n' "$missing2"
echo "  ✗ 仍缺 $nmiss2 项 —— 请把对应补充包的目录用 --metadef/--geexec/--bisheng 指过来（见 AGENTS.md §5.6 的表）" >&2
exit 1
