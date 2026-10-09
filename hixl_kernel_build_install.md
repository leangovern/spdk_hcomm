# HIXL 算子：编译 · 验证 · 安装（真机一步到位）

**目的**：让 SPDK 运行期能经 `aclrtBinaryLoadFromFile` 加载 `libcann_hixl_kernel.so` 里的
`HixlBatchSend` 算子——它是本仓数据面发送的**唯一入口**（链路见 `AGENTS.md` §2.6）。

本文**不含任何固定绝对路径**：`<...>` 与下面定义的变量按你的真机实际填写即可。
配套脚本：`scripts/hixl_union.sh`（同仓随 clone 带过去）。

---

## 0. 产出与落点

| 产出 | 落到哪 |
|---|---|
| `cann-hixl-compat.tar.gz`（内含 `libcann_hixl_kernel.so`） | `$CANN/opp/built-in/op_impl/aicpu/kernel/` |
| `libcann_hixl_kernel.json` | `$CANN/opp/built-in/op_impl/aicpu/config/` |

> ⚠️ **落的是 `.tar.gz`，不是裸 `.so`**：TSD 侧扫 `.../aicpu/config/` 的 JSON，再把同级的
> `.../aicpu/kernel/cann-hixl-compat.tar.gz` 解压到 `/usr/lib64/aicpu_kernels/<vfId>/aicpu_kernels_device/`，
> 按 JSON 里的 `kernelSo=libcann_hixl_kernel.so` 解析。tar 内路径就是 `aicpu_kernels_device/libcann_hixl_kernel.so`。

> ⚠️ **hixl 是"部署期依赖"，不是 spdk_hcomm 的编译/链接依赖**：没装也编得过，
> 只是运行期 send 报 `aclrtBinaryLoadFromFile ... failed` 并返回 `-EIO`。

---

## 0. 最快路径：一键脚本（推荐，尤其给不熟悉流程的人）

```bash
bash scripts/build_install_hixl.sh
```

一条命令做完：**自动探测** hixl / CANN / cmake → 前置检查 → 按需调用 `scripts/hixl_union.sh`
让 CANN 满足要求 → 编译 → **4 项产物验证** → 安装到 `$CANN/opp/built-in/op_impl/aicpu/` →
**验证安装** → 打印运行期注意事项。全程分步输出，失败时给出可执行的排错建议。

**安装默认走"最小化拷贝"**：只落地上面那两个产物，不碰 CANN 目录的其它内容。
（官方 `.run` 装的是整套 hixl，还会在安装目标里建 `x86_64-linux/`、`hixl/`、`var/`、
卸载脚本等脚手架并覆盖同名软链——**实测会把 union 弄坏**，见 §5。所以它需要显式开启。）

```bash
bash scripts/build_install_hixl.sh --help               # 全部选项
bash scripts/build_install_hixl.sh --build-only         # 只编译不安装（先验证环境）
bash scripts/build_install_hixl.sh --cann <CANN> --hixl <hixl 仓库>
bash scripts/build_install_hixl.sh --official-installer # 改走官方 .run 装整套 hixl
# CANN 是"手工合并子集"时，再补三个补充包目录：
bash scripts/build_install_hixl.sh --cann <CANN> \
  --metadef <metadef目录> --geexec <ge-executor目录> --bisheng <bisheng目录>
```

**下面的 §1~§6 是等价的手工步骤**——脚本失败时用它逐步定位；只想搞清楚每步在做什么也看这里。

---

## 1. 变量与前置检查

```bash
# ---- 按真机实际填写 ----
export CANN=/usr/local/Ascend/ascend-toolkit/latest   # = ASCEND_HOME_PATH，CANN 安装目录
export WS=$HOME/work                                   # 工作区（三个仓库的父目录，按实际填）
export HIXL=$WS/hixl                                  # hixl 仓库
export SPDK=$WS/spdk_hcomm                            # spdk_hcomm 仓库（本仓）
```

```bash
# ---- 一次跑完的前置检查 ----
echo "arch       : $(uname -m)"
echo "CANN       : $CANN"; [ -d "$CANN" ] && echo "  ✔ 存在" || echo "  ✘ 不存在"
echo -n "交叉编译器 : "; ls "$CANN/toolkit/toolchain/hcc/bin/aarch64-target-linux-gnu-g++" >/dev/null 2>&1 \
  && echo "✔ 有（设备侧必需）" || echo "✘ 缺 → 见 Step 1 档 C"
echo -n "cmake      : "; cmake --version 2>/dev/null | head -1 || echo "✘ 缺"
echo -n "hixl 源    : "; [ -d "$HIXL/src/ops/hixl_kernel" ] && echo "✔" || echo "✘ 路径不对"
echo -n "本仓脚本   : "; [ -f "$SPDK/scripts/hixl_union.sh" ] && echo "✔" || echo "✘ 缺"
```

**设备侧固定用 aarch64 交叉编译器**（`$CANN/toolkit/toolchain/hcc/bin/aarch64-target-linux-gnu-g++`）——
因为 AICPU 算子在 NPU 的 ARM 核上跑，与宿主架构无关（详见 §8）。这项缺了必构建失败。

## 2. Step 1：让 CANN 满足 hixl（先跑自检，按提示分档）

hixl 的 `cann-cmake` 会逐个 `find_package` 十余个 CANN 包。**先让脚本告诉你需不需要额外处理**：

```bash
bash "$SPDK/scripts/hixl_union.sh" --cann "$CANN"
```

| 输出 | 含义 | 你要做的 |
|---|---|---|
| `✔ 全部满足 —— 不需要 union`，并打印 `ASCEND_HOME_PATH=$CANN` | **档 A**：CANN 完整（正常安装的机器通常就是这档） | **什么都不用改**，Step 2 直接用 `$CANN` |
| `✔ 已在 $CANN/devlib/ 原地补好软链` | **档 B**：只缺 `devlib/libascend_hal.so`（纯布局问题） | 无需额外动作（若 `$CANN/devlib` 不可写，脚本会转档 C，需加 `--out`） |
| `✗ 仍缺 N 项` | **档 C**：CANN 是"手工合并的子集" | 按下表加补充包，生成 union |

**档 C 的补充包从哪来**：CANN 安装包（`Ascend-cann-toolkit-*.run`）解包出的 `run_package/` 里对应子包，
用 `--extract=<dir> --noexec` 解出来即可：

| 补充包 | 提供什么（缺哪项就用它） |
|---|---|
| `cann-metadef` | `include/graph`、`pkg_inc/common`、`lib64/{libmetadef,libexe_graph,libopp_registry}.so`、`librt2_registry.a` |
| `cann-ge-executor` | `include/acl/{acl_op.h,acl_mdl.h,ops/}`（CANN 打包 bug：这些头没进 npu-runtime） |
| `cann-bisheng-compiler` | `toolkit/toolchain/hcc`（含 aarch64 交叉编译器） |

```bash
bash "$SPDK/scripts/hixl_union.sh" --cann "$CANN" --out "$WS/cann-hixl-union" \
  --metadef <解出的 metadef 目录> \
  --geexec  <解出的 ge-executor 目录> \
  --bisheng <解出的 bisheng 目录>
# 末行打印 ASCEND_HOME_PATH=<union 目录>
```

> 脚本是 **symlink 树，不复制文件**；可重复运行；自检不过会明确列出还缺哪几项。

## 3. Step 2：编译 hixl

```bash
export ASCEND_HOME_PATH=<Step 1 给出的路径>     # 档 A = $CANN；档 B/C = 脚本末行打印的路径
cd "$HIXL"
bash build.sh -j"$(nproc)"
```

- 产物：`build_out/cann-hixl_9.1.0_linux-<arch>.run`（`<arch>` = `uname -m`）
- 无外网时，给本地 cann-cmake（默认会从 `gitcode.com/cann/cmake.git` 拉 tag `master-021`）：
  ```bash
  bash build.sh -j"$(nproc)" --cann_3rd_lib_path=<含 cmake-master-021.tar.gz 的目录>
  ```
- 期望结尾：`Build success!` / `hixl package success!`

## 4. Step 3：验证产物（4 项，缺一不可）

```bash
cd "$HIXL"
L=build/device_install/lib/libcann_hixl_kernel.so
J=build/device_install/hixl/aicpu_kernel/libcann_hixl_kernel.json

# ① 算子已导出
nm -D --defined-only "$L" | grep HixlBatchSend          # 期望：T HixlBatchSend

# ② 注册信息里登记了 HixlBatchSend，且 kernelSo 名字对
grep -o '"functionName": *"[^"]*"' "$J"                 # 期望含 HixlBatchSend
grep -o '"kernelSo": *"[^"]*"' "$J"                     # 期望 libcann_hixl_kernel.so

# ③ 对 hcomm 的引用全是弱符号 w（由设备侧 libccl_kernel 运行时解析）
nm -D -u "$L" | grep Hcomm | head                       # 期望全 'w'；出现 'U' 说明链接方式不对

# ④ 是 aarch64（设备侧目标）
file "$L"                                               # 期望 ELF 64-bit ... ARM aarch64
```

## 5. Step 4：安装到 opp 路径

**方式一：最小化拷贝（推荐）**——本项目只需要两个产物，就只拷这两个：

```bash
DEST="$CANN/opp/built-in/op_impl/aicpu"
install -d "$DEST/kernel" "$DEST/config"
cp "$HIXL/build/device_install/hixl/aicpu_kernel/cann-hixl-compat.tar.gz" "$DEST/kernel/"
cp "$HIXL/build/device_install/hixl/aicpu_kernel/libcann_hixl_kernel.json" "$DEST/config/"
```

**方式二：官方安装脚本**（想连带装整套 hixl——运行库、python 模块等——时用）

```bash
cd "$HIXL"
./build_out/cann-hixl_9.1.0_linux-<arch>.run --run --install-path="$CANN"
#   默认安装路径 /usr/local/Ascend（非 root 时为 $HOME/Ascend）；加 --install-for-all 可给所有用户装
```

> ⚠️ **别把它装到 union 之类的软链目录**。安装器会在安装目标里建 `x86_64-linux/`、`hixl/`、
> `var/`、`cann_uninstall.sh` 等一整套脚手架，并**覆盖同名软链**——实测会把 union 的
> `include/ge` 换成指向空目录的软链，之后编译报
> `ge/ge_allocator.h: No such file or directory`（本机踩过，重建 union 才恢复）。
> 装到真正的 CANN 安装目录没问题；用 union 编译时请走方式一。
>
> **另**：安装器最后会 `pip install` 一个 python wheel（llm_datadist）。系统 Python 的
> PEP 668（`externally-managed-environment`）会让它返回非 0，但**此时算子产物其实已经落好了**
> ——实测安装器顺序是"先落 kernel/config，最后装 wheel"，失败信息与本项目无关，可忽略
> （想让它彻底成功可加 `PIP_BREAK_SYSTEM_PACKAGES=1`）。

**确认（路径与文件名必须逐字一致）**：

```bash
ls -l "$CANN/opp/built-in/op_impl/aicpu/config/libcann_hixl_kernel.json" \
      "$CANN/opp/built-in/op_impl/aicpu/kernel/cann-hixl-compat.tar.gz"
tar -tzf "$CANN/opp/built-in/op_impl/aicpu/kernel/cann-hixl-compat.tar.gz"   # 应见 aicpu_kernels_device/./libcann_hixl_kernel.so
```

> spdk_hcomm 侧拼的就是 `$ASCEND_HOME_PATH/opp/built-in/op_impl/aicpu/config/libcann_hixl_kernel.json`
> （`lib/nvme/nvme_hcomm.c` 的 `NVME_HCOMM_HIXL_KERNEL_JSON_SUFFIX`）。改名或挪位置要同步改代码。

## 6. Step 5：确认 spdk_hcomm 能加载

```bash
cd "$SPDK" && make -j"$(nproc)"
readelf -d build/lib/libspdk_nvme.so.12.0 | grep NEEDED | grep -E "hcomm|ascendcl"
#   期望看到 libhcomm.so 与 libascendcl.so
```

> ⚠️ **运行期必须设 `ASCEND_HOME_PATH`**：spdk_hcomm 是这么拼 JSON 路径的
> （`lib/nvme/nvme_hcomm.c`）：
> ```c
> ascend_home = getenv("ASCEND_HOME_PATH");
> json_path = <ascend_home ?: "/usr/local/Ascend/cann"> + "/opp/built-in/op_impl/aicpu/config/libcann_hixl_kernel.json";
> ```
> 所以**跑 SPDK 的进程环境里** `ASCEND_HOME_PATH` 必须指向**装了算子的那个 CANN 目录**；
> 不设的话会回落到 `/usr/local/Ascend/cann`——而常规安装的 CANN 目录通常是
> `/usr/local/Ascend/ascend-toolkit/latest`，**回落值对不上就会加载失败**。
> 建议显式 export（与 Step 1~4 用的 `$CANN` 保持一致）：
> ```bash
> export ASCEND_HOME_PATH="$CANN"        # 或该 CANN 安装自带的 set_env.sh 已设好，则核对一下即可
> ```

真机跑起来后，日志里出现下面这条即算子加载成功（`SPDK_LOG_FLAGS=nvme_hcomm SPDK_LOG_LEVEL=debug`）：

```
loaded HixlBatchSend kernel: bin=... func=...
```

没有这条 → 算子没装对：send 会返回 `-EIO` 并打 `aclrtBinaryLoadFromFile ... failed`。
之后的完整链路对照见 `AGENTS.md` §2.6 ⑥ 的 DFX 对照链。

## 7. 排障表

| 现象 | 原因 | 处置 |
|---|---|---|
| `CANN toolchain path does not exist: .../toolkit/toolchain/hcc` | CANN 不含 hcc 工具链 | Step 1 档 C，`--bisheng` |
| `aarch64-target-linux-gnu-g++: No such file` | 缺交叉编译器（设备侧必需） | 同上 |
| `Could NOT find metadef` | 缺 metadef 头/库 | `--metadef` |
| `fatal error: acl_op.h: No such file or directory` | acl 头被打进 ge-executor 子包 | `--geexec` |
| `Could NOT find ascend_hal (missing: _CANN_ascend_hal_SHARED_LIBRARY)` | `devlib/` 下无直下的 `libascend_hal.so` | 脚本 Step B 自动补；不可写则 union |
| `.../share/info/<pkg>/version.info does not exist` | 版本门禁：**文件缺失=硬错误**（版本不符仅告警） | 脚本自动补 `Version=<CANN版本>` |
| 编译突然报 `ge/ge_allocator.h: No such file or directory`（之前能编过） | **官方 `.run` 被装进了 union/软链目录**，其脚手架覆盖了 `include/ge` 软链 | 别把 `.run` 装进 union（见 §5 警告）；用最小化拷贝；已被污染的 union 用 `hixl_union.sh` 重建 |
| `build.sh: line N: cmake: command not found` | cmake 不在 PATH | 装 cmake，或把真二进制所在目录加进 PATH（勿用 pip 的 py 包装器） |
| 拉 `gitcode.com/cann/cmake.git` 超时 | 无外网 | `--cann_3rd_lib_path=<含 cmake-master-021.tar.gz 的目录>` |
| **`Build failed.` 但退出码 0** | `build.sh` 把失败吞在管道里 | 用 `bash build.sh -j 2>&1 \| tail -30; echo ${PIPESTATUS[0]}`，或直接看构建日志尾部 |
| 运行期 `aclrtBinaryLoadFromFile ... failed` | ① 算子未装/路径或文件名不符（Step 4）；② **运行期未设 `ASCEND_HOME_PATH`**，回落到了 `/usr/local/Ascend/cann`（见 Step 5 提示） | 看报错里打印的**具体路径**即可区分是①还是② |
| 运行期加载成功但 send 无反应 | 非 A5 芯片 | `HcommSendWithImmOnThread` 仅 A5（`DEV_TYPE_950`）生效 |

## 8. 背景：为什么设备侧必须交叉编译

`hixl/CMakeLists.txt:60-67` 把设备侧目标（`cann_hixl_kernel`）作为 ExternalProject 构建，并**固定**传入
`-DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-hcc-toolchain.cmake`（`aarch64-target-linux-gnu-gcc/g++`）：

```
CMakeLists.txt:50  if(NOT EXISTS "${ASCEND_INSTALL_PATH}/toolkit/toolchain/hcc") → FATAL_ERROR
CMakeLists.txt:53  set(TOOLCHAIN_DIR "${ASCEND_INSTALL_PATH}/toolkit/toolchain/hcc")
CMakeLists.txt:67  -DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/cmake/aarch64-hcc-toolchain.cmake
```

原因：AICPU 算子在 NPU 的 **ARM 核**上执行，与宿主架构无关。因此

- 在 **x86_64 开发机**上编出的 `libcann_hixl_kernel.so` 也是 **aarch64** 的；
- 在 **aarch64 真机**上，宿主侧（`libhixl.so` 等）走原生编译器，设备侧仍走这套交叉工具链。

这也解释了为什么真机上同样必须先满足 `toolkit/toolchain/hcc` 的存在性检查。
