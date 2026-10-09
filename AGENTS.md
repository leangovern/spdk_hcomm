# AGENTS.md

本文件是**本仓库**（SPDK 24.01 fork，新增 HCOMM NVMe transport，INI 侧）的**唯一权威入门文档**。读完本文件即可：理解项目在做什么、把仓编出来、知道代码在哪、知道当前进展、知道有哪些坑，从而快速推进协作开发。

**一句话定位**：通过华为 HCOMM 通信库经昇腾 UB（UnifiedBus，灵衢）总线实现 NVMe-over-Fabrics 发起方，对标 `nvme_rdma` / `nvme_pcie` / `nvme_tcp`。

> **路径约定**：本文件**不硬编码任何绝对路径**（各开发者仓库与工具链位置不同）。下文用 `$REPO` 表示本仓库根目录（`git rev-parse --show-toplevel`），`$CANN_MERGED` 表示 CANN 解包后的 merged 目录（各开发者本机自定）。

### 快速上手（TL;DR）

1. 把 CANN 9.1.0 三个子包解包成 `$CANN_MERGED`（`include/` + `lib64/` + hccp 头），打好 §5.5 的 4 处补丁。
2. `./configure --with-hcomm=$CANN_MERGED --with-hccp-incdir=$CANN_MERGED/asc/... --with-dpdk --with-shared ...`（完整命令见 §5.2）。
3. `make -j$(nproc)`，再跑 §5.4 的 `nm -D ... | grep ' U Hcomm'` 校验符号（编译通过 ≠ 符号完整）。
4. 改代码先读 `lib/nvme/nvme_hcomm.c` 头部架构注释 + §4 函数索引 + §9 问题清单。

---

## 1. 项目概述与硬件拓扑

### 1.1 我们要做什么

HOST 侧的 SPDK 应用借助本机 NPU（AI CPU）上的 UB 网口，访问远端 NVMe 存储（TGT）。关键约束：**HOST 不能直接访问 UB fabric**，建链/注册/写命令由 HOST 编排，实际收发由 Device（AI CPU）执行。

```
┌────────────────────────────────────────────────────────────────────┐
│ INI 侧（本节点）                          TGT 侧（远端 NVMe 存储）  │
│                                                                    │
│  ┌──────────┐ 共享内存  ┌───────────┐        ┌──────────────────┐   │
│  │  HOST    │◄────────►│  Device   │◄─UB────│      TGT         │   │
│  │ (普通CPU)│  cqe/sq   │ (AI CPU)  │ Jetty  │  (NVMe SSD)      │   │
│  │ 跑 SPDK  │  缓冲     │  HCOMM库  │        │  UB网口在Device侧 │   │
│  └──────────┘           └───────────┘        └──────────────────┘   │
│   UB 网口在 Device（AI CPU）侧，HOST 无法直接访问 UB fabric           │
└────────────────────────────────────────────────────────────────────┘
```

- **UB / UnifiedBus（灵衢）**：华为统一总线网络及语义。
- **HCOMM**：华为通信基础库（`libhcomm.so`），提供 `Hcomm*`（Endpoint/Mem/Channel/Thread）与 `Ra*`（hccp socket）两套 C API。本仓经它建链、注册内存、下发 SQE。
- **hcomm transport（Device 侧）**：hcomm 库在 Device（AI CPU）侧的传输层（`hccp_service` 拉起）。数据面下发原语 `HcommSendOnThread` 为**双边 SEND**——由 A5（`DEV_TYPE_950`）AI CPU 执行，SQE 经 channel 直达 TGT，**无需 Device 侧配套应用**；该原语**仅 A5 芯片支持**（非 A5 返回 NOT_SUPPORT）。

### 1.2 两种部署模式

| 模式 | 环境变量 | 语义 | TP 类型 |
|---|---|---|---|
| offline（默认） | `SPDK_HCOMM_NETWORK_MODE=offline` | HOST 与 Device 同节点，经 HDC；需 `aclrtSetDevice` | CTP |
| peer_online | `SPDK_HCOMM_NETWORK_MODE=peer_online` | host 侧标准网卡，跨节点 | RTP |

两种 profile（`SPDK_HCOMM_PROTOCOL`）均 CONN_RM（单边可靠消息，无 bind）：

- `ubc_tp`（默认）：UB 原生协议，要求 UB 硬件网卡（EID 必须匹配 device UB EID 列表）。
- `uboe`：UB over Ethernet，`UboeIpv4ToEid()` 自动把 IPv4 → EID，直接填 TGT IP 即可。**多数 NPU↔TGT 以太网部署应选 `uboe`**。

---

## 2. 整体架构

### 2.1 数据面（零拷贝 keyed SGL + cid 索引数组）

- **SQ**：`sq_buf[cid]`——cid 索引的 SQE 槽数组（对照 `nvme_rdma` 的 `cmds[]`）。**分配 cid == 分配一个 SQE 槽**，无环/无 doorbell/无满检查。
  - `cmd.cid`（DW0）= cid（TGT 据此定位 CQE 槽；MPTR 回归标准 metadata 语义，当前为 0）。
  - `DPTR = keyed SGL {tokenId, user_buf_addr, length}`——用户缓冲经 `mem_map` 按需注册缓存，**零拷贝**（对照 `nvme_rdma` 的 `mr_map`）。
- **CQE**：`cqe_buf[cid]`——64B cacheline 对齐槽（16B 标准 `spdk_nvme_cpl` + 48B padding），**非环形**，TGT 直接写在 `cid × 64` 偏移；HOST 轮询在途 cid 的 `padding[0]` 有效标记。
- 深度 = `create_qpair` 传入的 `qsize`（单一来源），夹到设备 `sqMaxDepth`、**不取 2 的幂**；cid 池（`free_cids` 栈 + `active_cids` 顺序表）由 transport 自管；req 池 `num_requests` 超有效深度时被收敛到有效深度。
- 下发原语 `HcommSendOnThread(thread, channel, sqe, sqe_size+count*16)`：**双边 SEND**，由 Device 侧 A5 传输层执行，把整条 SQE（96B + 尾部 SGL）直接送达 TGT（channel 对端）。

### 2.2 SQE 线格式（96B 单格式）

> ⚠️ **128B 格式已临时删除**：`nvme_hcomm_sqe.h`、`sqe128_fill`、submit/dptr 的 128B 分支及 `sqe_96b` 格式选择逻辑均已移除（后续不再使用，96B 是唯一线格式）。若将来需要恢复 128B，从 **git 历史记录** 中找回：`lib/nvme/nvme_hcomm_sqe.h`、`test/unit/lib/nvme/nvme_hcomm_sqe.h/` 及 `lib/nvme/nvme_hcomm.c` 中相关 diff（`NVME_HCOMM_MAX_SGL`/`NVME_HCOMM_INI_JFR_TOKEN` 宏已迁入 `nvme_hcomm_sqe_96b.h`，恢复时注意去重）。

| 头文件 | 大小 | 布局 | 当前状态 |
|---|---|---|---|
| `nvme_hcomm_sqe_96b.h` | **96B（唯一格式，槽 352B）** | 标准 64B `spdk_nvme_cmd` 逐字节 + DW16~21 rsvd + 路由字 4B(DW22: ctrlid\|sqid；ctrlid 取 ctrlr->cntlid（ctrlr_construct 初始化 0xFFFF 动态分配，见 §8.1）) + token 4B(DW23) + **尾部 SGL 数组[16]**（偏移 96） | 唯一线格式 |

- 每个 qpair 在 `create_qpair` 时记录 `hqpair->sqe_size`（固定 96）与 `hqpair->sqe_stride`（96+256，含尾部 SGL 区）。
- **多 SGL（feat-hcomm-multi-sge）**：DPTR 描述符 type 区分单/多段——单段 = keyed data block 直写 DPTR（字节流与旧格式完全一致，老 TGT 兼容）；多段 = N 个 keyed 描述符在尾部数组，DPTR 放 `LAST_SEGMENT+OFFSET` 计数描述符（`length = N×16`，**唯一计数信源**，rdma / 标准 NVMf "SGL in capsule" 同款约定，TGT 取 `N = length/16`；原 `nsgl` dword 已删——传输层源码核实为 CONN_RM 可靠消息 + TA/TAACK ARQ，双编码交叉校验无对象），SEND 长度 = base+N×16 变长。段数有效上限 = min(16, Identify 的 ICD/MSDBD)（TGT 未上报回落 16，照 rdma 协商；`sgls.supported` 必须置位）。TGT 契约见 `hcomm_multi_sge_design.md`。
- **无协商**：INI 与 TGT 必须讲同一格式（lockstep 升级）；96B 尾部 DW22 路由字(DW22: ctrlid\|sqid) + DW23 token 供 TGT 定位 jetty。
- DPTR 写标准 NVMe `dptr.sgl1`（标准 NVMe 偏移 24）。

### 2.3 建链（控制面，两协议统一走 HcommChannelCreate）

`ctrlr_connect_qpair` 只启动 worker 线程即返回 0，qpair 保持 CONNECTING，后续全部由 poll 路径驱动（**三阶段异步**，rdma/tcp 同款模式）：

1. **Phase 1（worker 线程，阻塞）**：`nvme_hcomm_qpair_connect` —— `HcommMemReg`/export + `HcommThreadAlloc` + `HcommChannelCreate`（socket 建链 + UB 建链 + 内存交换全包）。UBOE 时预建 CLIENT socket 塞 `desc.socket`（规避 `AicpuTsUboeChannel` 按 IP 字节比较定角色、local<remote 变 SERVER 的库 bug）；UBC_TP 走库内 socket。随后 per-qpair NOF 侧信道握手 `nvme_hcomm_nof_handshake`（帧带 qid/sq_size/cqe 地址/TGT jetty 身份）。结果经 `connect_state` 原子发布（LINK_UP / FAILED）。
2. **Phase 2（poll 线程）**：`nvme_hcomm_qpair_connect_poll` —— LINK_UP 后经 `nvme_fabric_qpair_connect_async/_poll` 发 NVMe-oF fabrics CONNECT（admin CONNECT 在 TGT 创建 controller 并返回 cntlid；IO CONNECT 绑定 qid）。
3. **Phase 3（poll 线程）**：收割 CONNECT CQE —— 成功置 CONNECTED，框架推进（`WAIT_FOR_CONNECT_ADMINQ` / check_enabled）并重发 CONNECTING 期间排队的请求；失败走 `connect_unwind` → `nvme_transport_ctrlr_disconnect_qpair_done()`（abort 排队请求 + 置 DISCONNECTED，满足 reconnect 与两个框架等待者）。

### 2.4 资源归属（改代码前必须理解）

| 对象 | 粒度 | 说明 |
|---|---|---|
| `endpoint`（HcommEndpointCreate） | **ctrlr 级** | 本地身份 + 内存域（未来可进程级；JFC/token 池按 rdmaHandle 缓存） |
| `channel`（=一条 jetty） | **qpair 级** | 对照 `nvme_rdma` 每 qpair 一 QP；adminq 也有独立 channel |
| `thread`/RTSQ | **qpair 级** | 每 qpair `HcommThreadAlloc(engine,1,...)` 一条 |
| `socket_handle`（预建 CLIENT socket） | **ctrlr 级** | UBOE workaround，全 ctrlr 共享，`socket_lock` 保护 |
| `nof_fd_handle`（NOF 侧信道 fd） | **ctrlr 级** | 帧带 qid 复用 |
| `cqe_buf`/`sq_buf`/`req_for_cid`/`free_cids`/`active_cids`/`xlation_for_cid` | **qpair 级** | 数据面，对照 `nvme_rdma_qpair` |
| `mem_map`（注册缓存） | **ctrlr 级** | 全 qpair 共享一个 LRU 注册缓存（内部自带锁，多 reactor 安全；per-cid 引用记录在 qpair 的 `xlation_for_cid`） |
| JFC / TP | 库自动共享 | `RdmaHandleManager::GetJfcHandle` / `TpManager`（CONN_REUSE） |

### 2.5 依赖调用链

```
spdk_hcomm（INI transport，lib/nvme/nvme_hcomm.c）
  └─ 链接 libhcomm.so（$CANN_MERGED/lib64/libhcomm.so）
       ├─ Hcomm* C API（hcomm_res.h：Endpoint/Mem/Channel/Thread）
       └─ Ra* C API（network/hccp*.h）
            ├─ offline：HDC RPC → Device 侧 hccp_service.bin
            └─ peer_online：进程内直调 rs（librs.so）
```

### 2.6 数据面发送链路（SendWithImm / HixlBatchSend）

> 2026-09 对照参考实现（hcomm `staged-sendwithimm-rasocket-oob-userctl-ub-rtp-9.2.0` / hixl
> `staged-sendwithimm-batch-9.1.0`）逐环核实而成。**改数据面或上真机前必读本节。**

#### ① 两个"归属"别混淆（决定"要什么内存"）

| 对象 | 归属 | 说明 |
|---|---|---|
| SQE 数据本体 `sq_buf[cid]` | **HOST 内存**，`HcommMemReg` 注册即可 | UB 硬件经 DMA 读它没问题（与单边 Write/Read 的本地缓冲同一机制） |
| jetty SQ（`sqVa_`）/ RTSQ / 门铃寄存器 | **Device 侧资源** | 只有 AI CPU 能写；HOST 写会撞 `trs_sq_cq_query_uio` 拒绝（早期 host 直发失败的根因） |

**结论**："必须设备内存"说的是后者；**发送源用 HOST 内存 + 注册即可，不要求设备内存**。
判据只有一条：地址必须落在该 channel 的 `locBufferMap` 里（见 ④ 第 6 环）。附加条件：必须注册在
**同一个 endpoint**、且**早于 `HcommChannelCreate`**（`exchangeAllMems` 在建链那一刻快照）；必须 pinned
（`spdk_zmalloc` 的大页天然满足）。

#### ② AICPU 是通用 CPU，"算子"就是普通共享库

- AICPU（AI CPU）= NPU 上的通用 CPU 核；它加载执行的 kernel 是**普通 ELF 共享库**，跑普通 C++。
- hixl：`src/ops/hixl_kernel/` → `add_library(cann_hixl_kernel SHARED hixl_send.cc ...)`；`.ini` 登记
  `opInfo.opKernelLib=AICPUKernel` / `opInfo.kernelSo=libcann_hixl_kernel.so`；打包进
  `opp/built-in/op_impl/aicpu/kernel` + `.../aicpu/config`。
- hcomm：**同一份 `.cc`** 用 `-DCCL_KERNEL_AICPU` 编出设备侧目标 `ccl_kernel` / `ccl_kernel_plf`
  （`cmake/device/CMakeLists.txt:93`），打包成 `aicpu_hcomm.tar.gz`（`TAR_ROOT_DIR aicpu_kernels_device`）。

所以算子内可直接 `HcommProxy::SendWithImmOnThread(...)` → 弱符号 → device 侧 `HcommSendWithImmOnThread`，
**就是一次普通函数调用**，不是特殊机制。

#### ③ 完整链路

```
HOST（SPDK, lib/nvme/nvme_hcomm.c）
  ① spdk_wmb()                            ; 保证 SQE 字节对 DMA 可见
  ② nvme_hcomm_load_batch_send_kernel()   ; 一次性 BinaryLoadFromFile + GetFunction（flag 守卫）
  ③ 填 param{thread, io_batches[]={channel, local_src=sq_buf[cid], len, imm_data=0},
             batch_size=1, status_array, timeout_ms=0, complete_sender_cqe=0}
  ④ nvme_hcomm_launch_batch_send()        ; KernelArgsInit/Append/Finalize + LaunchKernelWithConfig
        ▼
AICPU 核（libcann_hixl_kernel.so）
  ⑤ HixlBatchSend(HixlBatchSendParam*)    ; 普通 C++，逐条循环 + 写 status_array
  ⑥ HcommProxy::SendWithImmOnThread       ; 弱符号解析到 hcomm device 侧
  ⑦ HcommSendWithImmOnThread              ; aicpu_ts_primitives_c_adpt.cc
        BuildLocRmaBufferLite(src,len) → RmaBufferLite{addr,size,tokenId,tokenValue}
        threadPtr->GetStreamLitePtr()  → 本 thread 的 RTSQ
        transportLitePtr->SendWithImm(locRmaBuf, immData, *streamLite)
  ⑧ UbTransportLiteImpl::SendWithImm      ; 取 connVec[0]
  ⑨ UbConnLite::SendWithImm               ; non-staged: FillOneSqeWrite(UDMA_OPC_SEND_WITH_IMM,...) + immData
  ⑩ ProcessOneWqe                         ; 写 jetty SQ（设备内存）+ cache clean/invalidate, pi++
  ⑪ BuildUbDbSendTask → RtsqA5::UbDbSend  ; 门铃作为一条 RTSQ 硬件任务提交
  ⑫ HcommThreadJoin                       ; 等 RTSQ head==tail 排空 = 本地发送完成
        ▼
UB 硬件 → TGT 收到 SQE 字节（对端须已预投递接收）
        ▼
TGT 处理 → 回写 CQE 到 cqe_buf[cid] → SPDK process_completions 轮询 padding[0]
```

#### ④ 逐环核实表（改数据面前对照）

| # | 环节 | 位置 | 结论 |
|---|---|---|---|
| 1 | `sq_buf` 分配（每槽 `sqe_stride` = 96 + 16×16 SGL 尾部区） | `nvme_hcomm.c:2931` | ✅ |
| 2 | `sq_buf` 经 `HcommMemReg` 注册（`COMM_MEM_TYPE_HOST`），size = `sq_depth × sqe_stride` | `:3869` → `:2323` | ✅ |
| 3 | **注册先于 `HcommChannelCreate`**（顺序错了第 6 环会静默拿错 token） | 注册 `:3869` → 建链 `:3934`→`:3784` | ✅ |
| 4 | `exchangeAllMems=true` 把注册内存收进交换列表 | `:3731` / `ub_mem_transport.cc:559` | ✅ |
| 5 | 序列化进 channel uniqueId → 设备侧解析成 `locBufferMap` | `ub_transport_lite_impl.cc:84,190` | ✅ |
| 6 | `BuildLocRmaBufferLite(sqe,len)` 命中 `sq_buf` → 正确 `tokenId/tokenValue` | `ub_transport_lite_impl.cc:281` | ✅ |
| 7 | SPDK AICPU launch 的 arg 约定与 hixl 自己的 `LaunchDeviceKernel`（`hixl_cs_client.cc:817`，服务 `HixlBatchPut/Get`）**同构** | `nvme_hcomm.c:4963`：`KernelArgsAppend(args, <结构体地址>, sizeof(结构体), &ph)` + `block_dim=1` + `LaunchKernelWithConfig(func, dim, stream, cfg, args, nullptr)` | ✅ |
| 8 | `cfg=NULL` 被 runtime 显式容忍；`reserve` 必须 nullptr | `runtime/src/acl/aclrt_impl/kernel.cpp:238` / `:235` | ✅ |
| 9 | 算子 → `SendWithImmOnThread` → `HcommSendWithImmOnThread` | `hixl_send.cc:46` / `aicpu_ts_primitives_c_adpt.cc:467` | ✅ |
| 10 | `SendWithImm` → SQE + immData → `ProcessOneWqe` | `ub_conn_lite.cc:496` | ✅ |
| 11 | 对端寻址靠 `tpn`（建链 import jetty 时得到）；**双边 SEND 不需要远端地址**，`rmtObjId=0` 是设计如此 | `ub_conn_lite.cc:102,910`；`DevUbConnection::SetImportInfo` | ✅ |
| 12 | 门铃 = RTSQ 任务（与 Write/Read/Notify/Wait 同一路径） | `ub_transport_lite_impl.cc:237` → `RtsqA5::UbDbSend` | ✅ |
| 13 | 本地完成：`STARS_POLL` 硬件自动 retire + `ThreadJoin`；故 `complete_sender_cqe` 必须为 **0** | `:5023` / `hixl_send.cc:53`；传 1 会抛 `sender CQ polling requires USER_CTL JFC`（`ub_conn_lite.cc:786`） | ✅ |

> **JFC 模式是建链内部决定的**（`AicpuTsUrmaChannel::BuildConnection` 报给 `DevUbConnection`），调用者碰不到。
> 9.1.0 的 AICPU 通道**硬编码 `STARS_POLL`**（`aicpu_ts_urma_channel.cc:112,119`）；参考 9.2.0 是
> `isStaged_ ? USER_CTL : STARS_POLL`——**staged 那套控制面 9.1.0 没有**（`isStaged_` 不存在、
> `SetStagedRemoteInfo` 无调用者），所以我们**永远走 non-staged + STARS_POLL**，与 `complete_sender_cqe=0` 自洽。
> 门铃两条路是"谁负责收尾 sender CQE"的两面：RTSQ 任务 ↔ `STARS_POLL`；直敲 ↔ `USER_CTL`（须软件收 CQE）。**不能自由组合。**

#### ⑤ 待确认项（真机前逐条对照）

1. 🔴 **设备上下文是 thread-local，submit 线程可能没有**。`aclrtLaunchKernelWithConfig` → `ApiImpl::LaunchKernelV2`
   会做 `Context *curCtx = CurrentContext(); CHECK_CONTEXT_VALID_WITH_RETURN(curCtx, RT_ERROR_CONTEXT_NULL)`
   （`runtime/src/runtime/core/src/api_impl/api_impl.cc:1285`）。而 `CurrentContext()` 先取**线程本地** context/ref，
   都没有时才尝试隐式 `SetDevice(defaultDeviceId)`——**前提是有人调过 `aclrtSetDefaultDeviceId`**（否则
   `hasSetDefaultDevId_` 为 false，直接返回 nullptr + 打 `current ctx is nullptr!`，见 `api_impl.cc:295-343`）。
   本仓只在两处 `aclrtSetDevice`：`ctrlr_construct`（`:3161`）与 connect worker（`:4079`），
   **submit 路径（`nvme_hcomm_send_sqe`）不做任何保证**。故若提 IO 的线程 ≠ 建 ctrlr 的线程
   （多 reactor / 跨线程提 IO），launch 会以 `RT_ERROR_CONTEXT_NULL` 失败。**这一条无法靠静态分析判定，
   取决于应用的线程模型。** 处置方向：在 `ctrlr_create_qpair`（`alloc_io_qpair` 在"将来用该 qpair 的线程"上
   调用）加一次性、**线程本地 flag 守卫**的 `aclrtSetDevice`；**不要动全局 `acl_refcnt`**（那是 acquire/release 配对的）。
2. 🟠 **`stream = NULL`（默认流）**：hixl 既有实现的同类 launch 用真实 stream + `aclrtWaitAndResetNotify`。
   runtime 对 stream 不做非空校验（`GetValidatedObjectImpl` 对 NULL 显式返回成功，`runtime_handle_guard.cc:113`），
   语法语义都允许；代价是**完全 fire-and-forget，不知道内核是否执行失败**——只能靠 `status_array` 与 CQE 超时暴露。
3. 🟡 **`BuildLocRmaBufferLite` 地址没命中时不报错**：打 WARNING 后用 `locBufferMap` **第一条**的 token 顶上
   （`ub_transport_lite_impl.cc:303-307`）。当前顺序（④ 第 3 环）保证命中，但将来注册/地址算错会**静默用错 token**。
4. 🟠 **`imm_data = 0` 与 TGT 契约**：staged 设计本意是 `imm = (controllerId << 16) | qpIndex`
   （`aicpu_ts_urma_channel.cc:774`，由 `HcommChannelGetStagedInfo` 暴露），语义同 RDMA `SEND_WITH_IMM` 的
   `imm_data`——供**接收端**识别来源/路由。我们改把 cid 放 SQE DW0、imm 传 0，**需与 TGT 约定一致**。
5. 🟠 **TGT 须已预投递接收**（`RaRecvWrlist`）才能收双边 SEND，属对端职责。

#### ⑥ 真机 DFX 对照链

启用 `SPDK_LOG_FLAGS=nvme_hcomm SPDK_LOG_LEVEL=debug`，正常一路应看到（顺序即链路）：

```
SPDK   : loaded HixlBatchSend kernel: bin=... func=...                                        # ②（仅首次）
HIXL   : [HixlBatchSend] send batch[0]. thread=... channel=... local_src=... len=... imm=0x0   # ⑤
hcomm  : [UbConnLite::SendWithImm] start, loc size[...], immData[0]                            # ⑨
hcomm  : [UbTransportLiteImpl::SendWithImm] SENT (with imm): slice addr[..] pi[..] immData[0]  # ⑧
hcomm  : [HcommThreadJoin] SUCCESS. RTSQ's head[..] == tail[..]                                # ⑫
HIXL   : [HixlBatchSend] completed. batch_size=1, overall_status=0                             # ⑤
```

| 卡在哪 | 首查 |
|---|---|
| ② `loaded ... kernel` 没出现 | 算子没装（§6.3 第 3 条）；日志报 `aclrtBinaryLoadFromFile ... failed` + `-EIO` |
| ④ launch 失败 | **⑤-1 设备上下文**（`RT_ERROR_CONTEXT_NULL`） |
| ⑨ 之前就错 | `BuildLocRmaBufferLite` 是否命中 `locBufferMap`（**⑤-3**；没命中是静默降级，看 WARNING） |
| ⑫ 超时 | 本地 RTSQ 没排空（`HCCL_E_TIMEOUT`）→ 算子返回 `FAILED` |
| ⑫ 通过但 TGT 没收到 | 对端预投递接收（**⑤-5**）、`imm` 契约（**⑤-4**） |

---

## 3. 仓库与目录结构

| 文件 | 内容 |
|---|---|
| `lib/nvme/nvme_hcomm.c`（约 5320 行） | **核心**：HCOMM transport 全部实现（INI 侧） |
| `lib/nvme/nvme_hcomm_sqe_96b.h` | 96B SQE 线格式（唯一格式；原 128B 头 `nvme_hcomm_sqe.h` 已临时移除，git 历史可恢复） |
| `include/spdk/nvme.h` | `SPDK_NVME_TRANSPORT_NAME_HCOMM "HCOMM"`(:32) / `SPDK_NVME_TRANSPORT_HCOMM=4098`(:442)，一等 transport |

> transport 上层接入：`ctrlr_scan` / `ctrlr_set_reg_*` / `ctrlr_get_reg_*` 复用 `nvme_fabric_ctrlr_*`。
>
> **编译门控**：`nvme_hcomm.c` 由 `lib/nvme/Makefile:22` 的 `C_SRCS-$(CONFIG_HCOMM) += nvme_hcomm.c` 门控，仅当 `./configure --with-hcomm=...`（`CONFIG_HCOMM=y`）才编译；`-lhcomm` 在 `lib/nvme/Makefile:62` 链接。`mk/config.mk`、`include/spdk/config.h`、`build/` 均为 configure 生成且已被 .gitignore——里面会带上本机绝对路径，**各开发者在本机重新 configure 即可，不要提交这些生成文件**。

---

## 4. 关键代码导读（行号 2026-09 核实，随演进漂移，以函数名检索为准）

### 4.1 文件头部必读

`nvme_hcomm.c` 第 1–150 行是**架构大注释**（数据面拓扑图 + 单条 NVMe 读写全流程 + 建链/生命周期），改代码前先读。

**对标参考实现**（写代码时对照着读）：`lib/nvme/nvme_rdma.c`（数据面/建链结构最接近）、`nvme_tcp.c`（fabrics + poll group）、`nvme_fabric.c`（fabrics 通用）、`nvme_transport.c`（ops 派发）、`nvme_pcie.c`（同步数据面对照）。

### 4.2 函数索引

| 主题 | 函数 | 行号 |
|---|---|---|
| env 配置 | `nvme_hcomm_protocol` / `init_network_mode` / `init_global_settings` / `socket_timeout_init` | 325 / 720 / 783 / 760 |
| ACL 设备上下文（引用计数） | `nvme_hcomm_acl_acquire_device` / `acl_release_device` | 629 / 680 |
| 全局工厂（引用计数；先查设备 EID 再 RaCtxInit，否则 rc=128003） | `acquire_global_socket` / `acquire_global_ctx` | 1202 / 1294 |
| 全局 endpoint（引用计数） | `acquire_global_endpoint` / `release_global_endpoint` | 1433 / 1473 |
| NOF 侧信道 | `nof_socket_connect` / `nof_handshake`（per-qpair）；共享建链 helper `ra_socket_connect`（RaSocketBatchConnect + RaGetSockets 有界轮询） | 1696 / 1883 |
| 内存注册 | `register_seg`(=HcommMemReg) / `unregister_seg` / `mem_map_create` / `get_memory_translation` | 2057 / 2079 / 2201 / 2244 |
| 队列分配 | `create_queues` / `destroy_queues` | 2605 / 2732 |
| ctrlr 生命周期 | `ctrlr_construct`（adminq 必须在返回前创建）/ `ctrlr_destruct` / `ctrlr_enable`(no-op) | 2792 / 2961 / 3027 |
| qpair 生命周期 | `ctrlr_create_qpair` / `create_io_qpair` / `delete_io_qpair`（join worker + abort in-flight） | 3091 / 3163 / 3171 |
| channel 建链 | `create_socket` / `qpair_create_channel` / `qpair_connect` / `qpair_disconnect_channels` | 3296 / 3344 / 3461 / 3557 |
| 异步 connect（三阶段，见 §2.3） | `qpair_connect_cleanup` / `connect_drain_fabric` / `connect_worker` / `connect_unwind` / `qpair_connect_poll` / `ctrlr_connect_qpair` / `ctrlr_disconnect_qpair` | 3445 / 3613 / 3639 / 3669 / 3692 / 3741 / 3784 |
| 错误恢复 | `qpair_abort_reqs`（已实现）/ `qpair_reset`（空，与 rdma 一致）/ `get_memory_domains`（空） | 3824 / 3872 / 3817 |
| IO 数据面 | `qpair_submit_request`（cid→sq_buf[cid]+keyed SGL）/ `qpair_process_completions`（轮询 cqe_buf[cid].padding[0]，max_completions=0 表示不限量） | 4344 / 4390 |
| poll group | `poll_group_process_completions`（已实现，遍历 connected_qpairs，负返回值上抛）/ no-op ops（框架基类管理 STAILQ） | 4634 |
| transport 注册 | `hcomm_ops` + `SPDK_NVME_TRANSPORT_REGISTER(hcomm, ...)` | 5277 / 5322 |
| **死代码区** | 手动 UB bringup 路径（JFC/Jetty create→exchange→import→bind），`#if 0` 包裹保留参考 | **#if 0：4720–5272** |

### 4.3 关键数据结构

| 结构 | 行号 | 要点 |
|---|---|---|
| `struct nvme_hcomm_ctrlr` | 862 | per-target：endpoint/endpointDesc、target_addr/port、local_eid、socket_handle、nof_fd_handle、socket_lock、mem_map（**per-ctrlr** 注册缓存，全 qpair 共享）；尾部 10 个死字段 `#if 0` 包裹（fd_handle/jfc_handle/jetty_handle 等，仅被死代码区引用） |
| `struct nvme_hcomm_qpair` | 959 | cid 索引数组（cqe_buf/sq_buf/req_for_cid/free_cids/active_cids/xlation_for_cid）、channel/thread、tgt_info、sq/cqe lmem_handle + tokenid、`sqe_size`/`sqe_stride`、异步 connect 状态机（connect_state 原子 + worker） |
| `struct nvme_hcomm_poll_group` | 1082 | 仅包 base group（no-op ops，框架基类管理 STAILQ） |
| `struct nvme_hcomm_mem_map` | 2159 | 零拷贝注册缓存（64 条目 LRU，引用计数防驱逐；per-ctrlr，内部自带锁） |
| `struct nvme_hcomm_socket_desc` | 3280 | libhcomm 私有 `SocketDesc` 的镜像复制（**ABI 风险点**） |

---

## 5. 编译流程

### 5.1 本机环境（已就绪，可直接编）

| 项 | 本机值 |
|---|---|
| CANN 9.1.0 merged 目录（下称 `$CANN_MERGED`） | `~/Ascend/cann-9.1.0/merged` |
| DPDK | apt `libdpdk-dev`（23.11.4；`pkg-config --modversion libdpdk` 验） |
| hcomm 库 | `$CANN_MERGED/lib64/libhcomm.so` —— **v3 ABI**（`HCOMM_CHANNEL_VERSION = 3U`，有 `channelName`） |

> ⚠️ **本机 `$CANN_MERGED` 的坑已经踩完并固化**：`securec.h` 的 `errno_t`、`hcomm_res_defs.h` 的
> `COMM_ADDR_EID_LEN`、补入的 `acl_base_mdl.h` 都在里面了。**不需要重做 §5.5 那 4 步**；只有换机器/重新解包时才做。
> ABI 门禁：9.1.0 正式版 = v3 ✅；**`9.1.0_beta.3` 是 v1（无 `channelName`）** ❌ 会编不过。

### 5.2 configure（一次性；本机已完成）

`mk/config.mk` 与 `include/spdk/config.h` 已存在且正确（含 `SPDK_CONFIG_HCOMM 1` 与本机路径），**日常不需要重跑**。确需重跑时：

```bash
cd "$REPO"
./configure \
  --with-dpdk --with-shared \
  --with-hcomm="$CANN_MERGED" \
  --with-hccp-incdir="$CANN_MERGED/asc/impl/adv_api/detail/hccl/cc/src/aicpu_kfc/platform/hccp/inc" \
  --disable-tests --disable-unit-tests --disable-examples --disable-apps \
  --without-isal --without-crypto --without-vbdev-compress --without-dpdk-compressdev \
  --without-vhost --without-virtio --without-nvme-cuse --without-raid5f --without-idxd \
  --without-iscsi-initiator --without-rdma --without-fc --without-daos --without-xnvme \
  --without-ublk --without-uring --without-fuse --without-ocf \
  --without-sma --without-golang --without-avahi --without-vtune --without-fio
```

- `--with-hcomm` → `CONFIG_HCOMM_DIR`（merged：`lib64/libhcomm.so` + `include/`）
- `--with-hccp-incdir` → `CONFIG_HCOMM_INC_DIR`（`network/hccp*.h`，不在 hcomm 子包里）
- `--with-shared` **必须**（CANN 只有 `libhcomm.so` 动态库，无静态库）
- 链接项在 `lib/nvme/Makefile:62`：`-lhcomm -lascendcl -lc_sec`（数据面用 `aclrtBinaryLoadFromFile` 加载 HIXL 算子，故**直链 libascendcl，不用 dlopen**）

### 5.3 编译

```bash
cd "$REPO" && make -j$(nproc)
```

产物 `build/lib/libspdk_nvme.a` / `build/lib/libspdk_nvme.so.12.0`。
**本机实测（2026-09）**：全量编译通过，仅有既有的 `-Wformat-truncation`（channelName tag 的 `snprintf`）告警，无 error。

### 5.4 验证（编译通过 ≠ 正确）

```bash
cd "$REPO"
nm build/lib/libspdk_nvme.a | grep nvme_hcomm_ctrlr_construct
readelf -d build/lib/libspdk_nvme.so.12.0 | grep NEEDED   # 期望：libhcomm.so / libascendcl.so / libc_sec.so
nm -D build/lib/libspdk_nvme.so | grep ' U Hcomm'
```

`U Hcomm` 期望**只剩 host 侧 API**：`HcommChannelCreate` / `HcommChannelDestroy` / `HcommEndpointCreate|Destroy` /
`HcommMemExport|Reg|Unreg` / `HcommThreadAlloc|Free`。
**不应再出现 `U HcommSendOnThread`** —— 数据面已改由 `HixlBatchSend` 算子在设备侧驱动（§2.6）。

> **教训（仍然有效）**：SO 链接容忍未定义符号，"编译通过"不等于符号完整——曾经正是这个盲区让
> `HcommSendOnThread` 未定义符号一路带进运行时。**任何 `Hcomm*` 符号都要按此校验**，别只看编译退出码。

### 5.5 换机器时：从 rpm 准备 `$CANN_MERGED`

只解三个必需子包（**不需要整套 1.5G toolkit**）：`cann-hcomm`（`lib64/libhcomm.so` + `include/hcomm` + `include/hccl`）、
`cann-asc-devkit`（`asc/.../hccp/inc/network/hccp*.h`）、`cann-npu-runtime`（`include/securec.h` + `include/acl/`）。

```bash
# <arch> 全程统一 x86_64 或 aarch64，别混搭
rpm2cpio Ascend-cann-toolkit-9.1.0-linux.<arch>.rpm | cpio -id
cd usr/local/Ascend
./Ascend-cann-toolkit_9.1.0_linux-<arch>.run --extract=. --noexec      # 解出 run_package/ 下 31 个子包
cd run_package
./cann-hcomm_9.1.0_linux-<arch>.run --extract=. --noexec
./cann-asc-devkit_9.1.0_linux-<arch>.run --extract=. --noexec
./cann-npu-runtime_9.1.0_linux-<arch>.run --extract=. --noexec
CANN_MERGED=/path/to/your/cann-9.1.0/merged
rm -rf "$CANN_MERGED" && mkdir -p "$CANN_MERGED" && cp -ral <arch>-linux/. "$CANN_MERGED"/
```

**解包后必做 4 处手工改动**（configure 不会处理，缺一编不过）：

1. `$CANN_MERGED/include/securec.h`：`errno_t` 的条件式 `#if` → **无条件 `typedef int errno_t;`**（保留 `#ifndef` 防重定义）。原因：spdk 用 `-std=gnu11`，gcc 默认置 `__STDC_WANT_LIB_EXT1__`，而 glibc 不实现 Annex K —— 两头落空，`errno_t` 未定义。
2. `$CANN_MERGED/include/hcomm/hcomm_res_defs.h`：`static const uint32_t COMM_ADDR_EID_LEN = 16U;` → `enum { COMM_ADDR_EID_LEN = 16U };`。原因：C 里 `static const` 不是编译期常量，不能做文件作用域数组大小。
   > `lib/nvme/Makefile:27-33` 有个"hcompat 自动 sed 修复"，但**源路径写错**（`include/hcomm_res_defs.h` 少了 `hcomm/`），生成空文件、`|| true` 又吞错 —— **目前无效**，仍须手工改（§9 问题 14）。
3. `mk/config.mk`：`CONFIG_IPSEC_MB?=y` → `n`，然后 `rm -f include/spdk/config.h`。原因：crypto 已关，否则 `ipsecbuild/Makefile` 去找缺失的 intel-ipsec-mb submodule。
   > ⚠️ **每次 configure 都会把它写回 `y`** —— 每次跑完都要重新确认 + 删 `config.h` 让它重生成。
4. **补 `acl_base_mdl.h`**：9.1.0 里 `hcomm/hcomm_primitives.h` → `acl/acl_rt.h` → `acl_base.h` → `acl_base_mdl.h`，但该头被 CANN **错打包进 `cann-ge-executor` 子包**（9.1.0 打包 bug）：
   ```bash
   cp <ge-executor>/<arch>-linux/include/acl/acl_base_mdl.h "$CANN_MERGED/include/acl/"
   ```
   > 退化方案：删掉 `hcomm_primitives.h` 的 include 也能编过（`ChannelHandle`/`ThreadHandle` 已由 `hcomm_res_defs.h:52/60` 提供），但**推荐补头文件**，不改源码迁就打包 bug。

### 5.6 构建 HIXL 算子（数据面前置，跨仓库）

SPDK 数据面依赖 `libcann_hixl_kernel.so`（§2.6 ②），由 **hixl 仓库**构建。**本机已实测跑通（2026-09）**。

> 📖 **真机完整逐步指导（含前置检查、分档处置、排障表）见同仓 `hixl_kernel_build_install.md`** ——
> 那篇才是给真机用的，全篇无固定路径。本节只给要点与"为什么需要这些步骤"。

```bash
# ---- 推荐：一键脚本（自动探测 hixl/CANN/cmake → 编译 → 验证 → 安装 → 再验证）----
bash <本仓>/scripts/build_install_hixl.sh
#   常用：--cann <CANN目录> --hixl <hixl仓库> --build-only --install-cmake --help
#   安装默认"最小化拷贝"：只落 kernel/config 两个产物，不动 CANN 目录其它内容；
#   --official-installer 才走官方 .run 装整套 hixl（脚本会拒绝装进 union 软链目录——
#   实测安装器会在目标里建脚手架并覆盖同名软链，把 union 的 include/ge 弄坏）。
#   不熟悉流程的同事直接跑它即可；失败会打印可执行的排错建议。

# ---- 等价的手工步骤（脚本失败时逐步定位用）----
export CANN=<CANN 安装目录>          # 即 ASCEND_HOME_PATH
export WS=<工作区父目录>             # 三个仓库所在目录
export HIXL=$WS/hixl
export SPDK=$WS/spdk_hcomm           # 本仓

bash "$SPDK/scripts/hixl_union.sh" --cann "$CANN"   # ① 自检：说"不需要 union"就跳过，否则按提示补
export ASCEND_HOME_PATH=<上一步末行给出的路径>
cd "$HIXL" && bash build.sh -j"$(nproc)"            # ② 编译
```

产物与验证：

```
build_out/cann-hixl_9.1.0_linux-<arch>.run                          # 整包（标准 CANN 包布局）
build/device_install/lib/libcann_hixl_kernel.so                     # 算子本体
build/device_install/hixl/aicpu_kernel/cann-hixl-compat.tar.gz      # ★ 实际部署件（内含上面那个 .so）
build/device_install/hixl/aicpu_kernel/libcann_hixl_kernel.json     # 算子注册信息
```

```bash
nm -D --defined-only .../libcann_hixl_kernel.so | grep HixlBatchSend   # 期望 T HixlBatchSend
grep -c HixlBatchSend .../libcann_hixl_kernel.json                     # 期望 >=1
nm -D -u .../libcann_hixl_kernel.so | grep Hcomm                       # 期望全是 w（弱符号，设备侧解析）
tar -tzf .../cann-hixl-compat.tar.gz                                   # 期望 aicpu_kernels_device/./libcann_hixl_kernel.so
```

> **落盘的是 `cann-hixl-compat.tar.gz`，不是裸 `.so`**：TSD 扫 `.../aicpu/config/` 的 JSON，再把同级
> `.../aicpu/kernel/cann-hixl-compat.tar.gz` 解到 `/usr/lib64/aicpu_kernels/<vfId>/aicpu_kernels_device/`，
> 按 JSON 的 `kernelSo` 解析。别只拷裸 `.so` 进 `kernel/`（那是错的）。

**为什么需要 `scripts/hixl_union.sh`**：hixl 的 `cann-cmake` 会逐个 `find_package` 十余个 CANN 包。
**完整安装的 CANN 通常直接满足**（脚本 Step A 会明确说"不需要 union"）；只有"手工合并的子集"才需要补：

| 缺件 | 谁要 | 补法 |
|---|---|---|
| `toolkit/toolchain/hcc`（含 **aarch64 交叉编译器**） | `CMakeLists.txt:50`（缺即 `FATAL_ERROR`） | `--bisheng`。**设备侧固定交叉编译**——AICPU 在 NPU 的 ARM 核上跑，与宿主架构无关 |
| `include/graph/types.h`、`pkg_inc/common/checker.h`、`lib64/{libmetadef,libexe_graph,libopp_registry}.so`、`librt2_registry.a` | `Findmetadef` | `--metadef` |
| `include/acl/{acl_op.h,acl_mdl.h,ops/}` | `acl.h` 内 `#include "acl_op.h"` | `--geexec`（9.1.0 打包 bug；`acl_base_mdl.h` 同理，见 §5.5 第 4 条） |
| `devlib/libascend_hal.so` | `Findascend_hal`（找 `devlib` **直下**） | 脚本原地补软链到 `devlib/linux/<arch>/` |
| `share/info/<pkg>/version.info` | 版本门禁（**文件缺失=硬错误**，版本不符仅告警） | 脚本自动补 `Version=<CANN版本>` |
| `cann-cmake` tag `master-021` | `fetch_cann_cmake.cmake` | 联网从 `gitcode.com/cann/cmake.git` 拉；离线用 `--cann_3rd_lib_path` |
| `cmake`（真二进制，非 pip 的 py 包装器） | `build.sh` | 加进 `PATH` |

**部署到真机**：**union 只是"能编出来"的手段，与真机运行无关**。真机只需把
`cann-hixl-compat.tar.gz` + `libcann_hixl_kernel.json` 两个文件落到 `$CANN/opp/built-in/op_impl/aicpu/{kernel,config}/`。
官方 `.run` 别装进 union 目录（会覆盖软链把 union 弄坏，见 `hixl_kernel_build_install.md` §5）。

---

## 6. 运行与使用

### 6.1 用户参数（trid）

| trid 字段 | 取值 | 说明 |
|---|---|---|
| `trtype`/`trstring` | `"HCOMM"` | 选 hcomm transport |
| `traddr` | TGT IP | UBOE 下自动 IPv4→EID |
| `trsvcid` | TGT 端口 | UBOE = TGT NOF 服务端口 |
| `subnqn` | 子系统 NQN | UB 无 discovery target，建议直连 |

RPC 示例：`{"name":"Nvme0","trtype":"HCOMM","traddr":"192.168.1.100","trsvcid":"5555","adrfam":"IPv4","subnqn":"nqn....:ub-target-1"}`

### 6.2 环境变量

| 变量 | 默认 | 作用 |
|---|---|---|
| `SPDK_HCOMM_PROTOCOL` | `ubc_tp` | `uboe` 切 UBOE |
| `SPDK_HCOMM_NETWORK_MODE` | `offline` | offline/peer_online |
| `SPDK_HCOMM_PHY_ID` | 0 | 物理网口（<64） |
| `SPDK_HCOMM_SOCKET_TIMEOUT_MS` | 30000 | socket 建链轮询总超时（RaGetSockets 轮询 / NOF 握手 / channel 状态轮询共用） |
| `SPDK_HCOMM_CHANNEL_PORT` | 60001 | TGT 侧 HCCP channel 建连端口（`HcommChannelCreate` 的 `desc.port`，与库 `DEFAULT_LISTENING_PORT` 同值；原误用 trsvcid/NOF 端口，仅当 TGT 在 NOF 端口同时监听 channel 时才碰巧可用） |
| `SPDK_HCOMM_CONNECT_WATCHDOG_MS` | 300000 | 整个异步建链看门狗（worker 握手 + fabrics CONNECT，自 spawn 起算）；超时后 connect_poll 不再无限 `-EAGAIN`：worker 仍在 WORKING → 标记 abandoned、发布 FAILED、finalize 但**不清理**（资源归可能存活的 worker，由重连清扫 join 后释放）；其它相位 → 正常 unwind（-ETIMEDOUT） |
| `SPDK_HCOMM_LOCAL_IP` | 自动探测 | 覆盖本地 IP/EID 检测 |

硬编码：notifyNum=4；engine=`COMM_ENGINE_AICPU_TS`；UBOE listen port 60001；INI JFR token `0xACFE`；channelName 格式 `nvme_hcomm_<ctrlid>_<qid>`（ctrlid 首次 admin 建链时为 0xFFFF（动态分配请求），tag 随 HCCP 连接传到 TGT 做白名单/qpair 绑定，仅库内 socket 路径 UBC_TP 生效；UBOE 预建共享 socket 的 tag 保持基名 `nvme_hcomm`，qpair 绑定靠 NOF 握手帧）。

### 6.3 运行前置（真机）

1. NPU 驱动/固件 + CANN 环境（libascendcl/libtsdclient/libhcomm 在 `LD_LIBRARY_PATH`）。
   **`ASCEND_HOME_PATH` 必须在运行期环境里设好**——SPDK 用它拼算子的 JSON 路径
   （`getenv("ASCEND_HOME_PATH")` + `/opp/built-in/op_impl/aicpu/config/libcann_hixl_kernel.json`；
   未设时回落 `/usr/local/Ascend/cann`，**常规安装的 CANN 目录通常不是这个，会加载失败**）。
   指向的必须是第 3 条里装了算子的那个 CANN 目录。
2. Device 侧 `hccp_service` 已拉起（`HcommEndpointCreate` 内部会自动拉起）。
3. **HIXL 算子已 build 并装进 CANN opp 路径**（数据面发送依赖）：hixl 仓库（`src/ops/hixl_kernel/`）build 出算子，装成
   `$ASCEND_HOME_PATH/opp/built-in/op_impl/aicpu/kernel/cann-hixl-compat.tar.gz`（内含 `libcann_hixl_kernel.so`）+
   `.../aicpu/config/libcann_hixl_kernel.json`（`libcann_hixl_kernel.ini` 登记的 `opInfo.kernelSo`）。
   SPDK 运行期经 `aclrtBinaryLoadFromFile` 加载该算子（`HixlBatchSend`），再由算子内部调 device 侧
   `HcommSendWithImmOnThread`（hcomm 源码仓 ccl_kernel 提供）。**这是部署期依赖，不是编译/链接依赖**——
   缺失时 send 报 `aclrtBinaryLoadFromFile ... failed` 并返回 -EIO。
4. TGT 监听 `traddr:trsvcid`，解析 SQE（含多 SGL 契约，见 `hcomm_multi_sge_design.md`）并按 cid 回写 CQE。**A5 环境**：数据面发送（HixlBatchSend → `HcommSendWithImmOnThread`）仅在 A5 芯片（`DEV_TYPE_950`）生效。

---

## 7. 代码提交规则

### 7.1 提交信息惯例（从 git log 归纳，保持一致）

| 前缀 | 用途 |
|---|---|
| `nvme/hcomm: ...` | 通用改动 |
| `feat(nvme/hcomm): ...` | 新功能 |
| `fix(nvme/hcomm): ...` | bug 修复 |
| `refactor(nvme/hcomm): ...` | 重构 |
| `docs(nvme/hcomm): ...` | 文档 |
| `test(nvme/hcomm): ...` | 测试相关 |

### 7.2 提交工作流（提交前 → 提交中 → 提交后）

**① 提交前自检（每次 commit 之前尽量过一遍，不设硬性门槛）：**

1. **编译检查（本地能编译则做）**：`cd "$REPO" && make -j$(nproc)` 全量编译通过（无 error；若新增 warning 需说明原因）。**本地环境编译不了时不死磕，跳过即可，由开发者自行想办法**。
2. **符号完整性检查（能编译才做，本项目特有）**：`nm -D build/lib/libspdk_nvme.so | grep ' U Hcomm'` 无**新增**未定义符号（见 §5.4，编译通过 ≠ 符号完整）。
   > 反面教材：`2d2f55c`（96B SQE）丢失两处 `#endif` 导致编译失败，靠 `4db7e1b` 修复——**当时有编译环境却没编**，入库后才暴露。
3. **测试（能测就测）**：
   - **无真机环境**：能编译时 `make clean && make -j$(nproc)` 干净构建回归，确认从干净状态可编译。
   - **有真机环境**（NPU 驱动 + CANN + hccp_service + TGT 齐备，A5 芯片环境）：跑 probe → channel 建链 → IO 读写，用 `SPDK_LOG_FLAGS=nvme_hcomm SPDK_LOG_LEVEL=debug` 观察。单测当前 `--disable-unit-tests`，主要以真机功能验证为准。
4. **`git diff` 自查 + 交协作方审视**：重构/大改先 `git diff` 给别人看，确认后再提交。

**② 提交动作：**

5. **小步 commit**：一个语义一个 commit，可独立回滚，不要混入无关改动。
6. **不提交产物/生成文件**：`*.o` / `*.d` / `build/` / `mk/config.mk` / `include/spdk/config.h` 都在 .gitignore —— 提交前 `git status` 里不该出现它们。
7. **不改源码迁就 CANN 打包 bug**：用「补 merged 头文件」解决，保持 `nvme_hcomm.c` 与 HEAD 一致（`git diff` 干净）。
8. 提交信息按 §7.1 前缀惯例。

**③ 提交后检查（commit 完成后）：**

9. **复验可编译（能编译时）**：对提交后的 HEAD 再跑一次第 1、2 步（或 `make clean && make` 干净构建），确认提交出去的代码确实可编译。
10. **可测时复验功能**：真机环境就绪时，对提交后版本再跑一遍功能测试（建链 + IO）。

> 沟通语言：中文。

### 7.3 代码风格（与 SPDK 保持一致，改代码前必读）

本仓是 SPDK 24.01 的 fork，**所有新增/修改代码必须与 SPDK 上游代码风格一致**。机器格式以仓库根目录 `.astylerc` 为准（`style=kr`、`indent=force-tab=8`、`align-pointer=name`、`max-code-length=100`、`pad-oper`、`pad-header`），人工对照以下规则。

**① 函数**

- **命名**：`snake_case`，模块前缀 + 动词（如 `nvme_hcomm_qpair_submit_request`）；公开 API 用 `spdk_` 前缀。**禁止**驼峰（`submitRequest` ❌）、匈牙利命名。
- **格式**（SPDK 风格，返回类型独占一行）：
  ```c
  static int
  nvme_hcomm_foo(struct nvme_hcomm_ctrlr *hctrlr, uint16_t qid)
  {
  	...
  }
  ```
- 函数上方用块注释说明职责/参数/返回值（仿照 `nvme_rdma.c` 写法），不写逐行流水账。
- 错误返回 `-errno`（负值）；成功返回 0 或正值。

**② 结构体 / 联合体 / 枚举**

- **类型命名**：`struct nvme_hcomm_xxx` / `union nvme_hcomm_xxx` / `enum nvme_hcomm_xxx`（小写 snake_case，模块前缀）。
- **成员命名**：小写 `snake_case`，语义化（`sqe_size`、`jetty_token`、`free_cids`）。**禁止**成员名带类型前缀/后缀（`int32_t nNum` ❌）、**禁止**全大写（`SQE_SIZE` 仅用于宏/枚举常量）。
- **布局注释**：成员按 dword/byte 偏移组织，段注释用小写 `/* dword N */`、`/* dword N-M */`（对照 `spdk_nvme_cmd`），字段行内注释紧随 `\t/* description */`：
  ```c
  struct nvme_hcomm_sqe {
  	/* dword 0 */
  	uint16_t opc	:  8;	/* opcode */
  	uint16_t fuse	:  2;	/* fused operation */
  	uint16_t cid;		/* command identifier */

  	/* dword 1 */
  	uint32_t nsid;		/* namespace identifier */
  };
  ```
- **位域**：与 `spdk_nvme_cmd` 相同的位域风格（`uint16_t opc : 8;`，位域后接完整字段），先列位域再列普通字段。
- **reserved 字段**：命名 `rsvdN`（按偏移递增）或语义化（`rsvd_16_19` 位域）。
- 结构体定义后紧跟 `SPDK_STATIC_ASSERT(sizeof(...) == N, "...")` 钉死布局（本仓 SQE 头已有此惯例，新增线格式结构必须照做）。
- 位域联合体用 `union { struct { ... } bs; uint32_t value; };` 双视图（packed value + field view），`SPDK_STATIC_ASSERT` 钉 4 字节。

**③ 注释**

- 全部 `/* ... */`，**禁用** `//` 行注释（SPDK 约定）。
- 小写开头（专有名词除外：NVMe、HCOMM、UB、TGT、INI、MPTR、DPTR、CQE、SQE、JFR 等）；句号可省。
- 线格式/协议注释描述**线字节序**（LE host 下字段如何落位），供 TGT 侧解析对照。
- `#if 0` 死代码块保留作参考，说明为何隔离（手动 UB bringup 路径，已被 `HcommChannelCreate` 取代）。

**④ 宏 / 常量**

- 宏、枚举常量：全大写 + 下划线（`NVME_HCOMM_SQE_SIZE`、`NVME_HCOMM_INI_JFR_TOKEN`）；枚举成员 `XXX_YYY`。
- 魔数必须命名（如 `0xACFE` → `NVME_HCOMM_INI_JFR_TOKEN`）。

**⑤ 对齐 / 缩进 / 行宽**

- 缩进：**Tab**（=8），续行用 Tab + 空格对齐到括号内。
- 行宽 ≤ 100 字符；超长拆行：函数参数每行一个、`if` 逻辑换行后 `break-after-logical`。
- 指针 `*` 靠变量名（`struct spdk_nvme_cmd *cmd`）。
- 单行条件必须加大括号（`.astylerc` 的 `j` 选项）。

**⑥ 命名风格自查清单**（提交前过一遍）

- [ ] 函数：`模块前缀_snake_case`，返回类型独立行
- [ ] 结构体/成员：小写 snake_case，语义化，无类型前缀
- [ ] 注释：`/* */` 小写，段注释 `/* dword N */` 风格
- [ ] 宏/常量：全大写
- [ ] 无 `//` 注释、无驼峰、无制表符混空格
- [ ] 新增结构体有 `SPDK_STATIC_ASSERT` 钉大小

> 反例教训：`2d2f55c`（96B SQE）曾用 `Dword` 大写段注释 + `ext_flags`/`token_id` 风格不统一的命名，后续重构对齐到 `spdk_nvme_cmd` 风格（位域 DW0 + `/* dword N */` 注释 + `jetty_token`）——新代码起步就按此规范写，避免事后返工。

### 7.4 华为 C 语言编程规范（补充规范）

> **优先级**：§7.3（SPDK 仓库规范，含 `.astylerc` 机器格式）**最高**；本节次之。**两者冲突时一律以 §7.3 / SPDK 上游实践为准。**
> 来源：《华为技术有限公司 C 语言编程规范》（DKBA 2826-2011.5，2011-05，61 页，工作区根目录 `华为技术有限公司c语言编程规范.pdf`）。

**与 SPDK 风格冲突、明确不采用的条目（速查）**

| 华为条目 | 本仓处理 | 理由 |
|---|---|---|
| 规则9.1 缩进 4 空格 | **Tab = 8**（§7.3 / `.astylerc`） | SPDK 机器格式 |
| 规则3.2/3.3 全局/静态变量加 `g_`/`s_` 前缀 | 不强制前缀 | SPDK 语义命名惯例（既有 `g_hcomm` 是历史选择，不是新规则） |
| 建议3.3 标识符不加模块前缀 | **保留模块前缀**（`nvme_hcomm_*`） | SPDK transport 惯例 + §7.3 |
| 规则8.2 文件头列版权/版本号/作者/工号 | SPDX + SPDK 简洁文件头 | 对齐上游 |
| 建议8.2 中英兼有优先中文注释 | **英文注释**（§7.3） | 对齐 SPDK 上游 |

> 其余条目（头文件 / 函数设计 / 命名 / 变量与数据结构 / 宏与常量 / 质量保证 / 表达式 / 安全性 / 注释 / 编译与告警 /
> 可测性 / 程序效率）与 §7.3 大面积重叠，**不在此重复**；需要时直接查上表所指 PDF。
> 其中只有两条是本仓在 §7.3 之外的**自加硬约束**，单独记住即可：
> ① **新增/重写函数 ≤ 50 行**（含行内注释，函数上方 doc 注释不计；既有函数豁免，被触碰过半时顺势达标）；
> ② **任何 `Hcomm*` 接口返回的指针，先用库源码确认所有权再决定是否 free**（见 §9 问题 19 的 double-free 教训）。

---

## 8. 当前项目进展（2026-09）

### 8.1 关键设计决策与结论

> 只记**仍然生效的契约与踩过的坑**；逐次改动请看 `git log`，仍开放的缺口见第 9 节。

**架构 / 资源归属 / 建链**

- transport 一等集成：`"HCOMM"`（`SPDK_NVME_TRANSPORT_HCOMM=4098`）；`ctrlr_scan`/`set_reg`/`get_reg` 复用 `nvme_fabric_ctrlr_*`。
- **channel 归属 qpair**（每 qpair 一条 = 一条 jetty），`tgt_info` 同；ctrlr 只持共享的 `socket_handle`/`nof_fd_handle`；`endpoint` 为 ctrlr 级（本地身份 + 内存域）。ctrlr 构造错误路径统一走 `nvme_ctrlr_destruct` 管线；ACL 全局上下文带引用计数。
- **建链统一走 `HcommChannelCreate`**（UBC_TP / UBOE 同路径），**三阶段异步**：worker 只跑阻塞握手（注册/export/ThreadAlloc/channel/NOF），fabrics CONNECT 由 poll 路径驱动，失败统一 `connect_unwind` → `nvme_transport_ctrlr_disconnect_qpair_done()`。
- **connect 清理单所有权**：worker 只清自己错误路径，**所有退出路径都发布终态**（LINK_UP/FAILED）；发布后资源归下一个 disconnect/unwind —— **绝不在 worker 可能仍在跑时清理**。`connect_qpair` 入口先 join 未退出 worker，再清扫非 CONNECTED qpair 的残留（FABRIC_SENT 残态先 drain）。
- **`socket_lock`** 串行化"共享 socket 创建 + NOF 握手"；connect 失败**不得**误删共享 socket（全 ctrlr 复用）。
- **建链看门狗** `SPDK_HCOMM_CONNECT_WATCHDOG_MS`（默认 300s）：超时且 worker 仍 WORKING → 标记 abandoned + 发布 FAILED + **不清理**；其它相位 → 正常 unwind（`-ETIMEDOUT`）。

**数据面**

- **96B SQE 是唯一线格式**（128B 已移除，可从 git 历史恢复，见 §2.2 注）；每槽 `sqe_stride` = 96 + 16×16（含尾部 SGL 区）。
- **cid 即 SQE 槽下标**：`sq_buf[cid]`/`cqe_buf[cid]` **非环形**，无门铃、无满检查；cid 池（free 栈 + active 顺序表）由 transport 自管。
- **深度单一来源** = `create_qpair` 的 `qsize`，夹到设备 `sqMaxDepth`、**不取 2 的幂**；req 池收敛到有效深度（避免白分配）。
- **多 SGL**：单段折叠回 DPTR（与旧格式字节流一致 → 老 TGT 兼容）；多段在尾部描述符数组，DPTR 放 `LAST_SEGMENT+OFFSET` 计数描述符（`length = N×16` 为**唯一计数信源**）。段数上限 = min(16, ICD, MSDBD)；TGT 契约见 `hcomm_multi_sge_design.md`。
- **路由字 DW22 = ctrlid | sqid**：ctrlid 取 `ctrlr->cntlid`（construct 初始化为 `0xFFFF` 请求动态分配，CONNECT response 存回真值）。
- **发送链路**（HixlBatchSend → AICPU 算子 → 设备侧 `SendWithImm`）见 §2.6 —— **改数据面/上真机前必读**。

**控制面踩坑（都已修，但改代码时别踩回去）**

- `channelName` = `nvme_hcomm_<ctrlid>_<qid>`：tag 随 HCCP 连接传到 TGT，做白名单/socket 池化 key + qpair 绑定（仅 UBC_TP 库内 socket 路径生效；UBOE 预建共享 socket 保持基名，绑定靠 NOF 帧）。
- `desc.port` 由 `SPDK_HCOMM_CHANNEL_PORT` 控制（默认 60001，同库 `DEFAULT_LISTENING_PORT`）——**原误用 trsvcid/NOF 端口**，只有 TGT 恰好在 NOF 端口也监听 channel 时才碰巧可用。
- **offline + IPv6 已解禁**：库对 IPv6 只静默跳过 `RaHdcSocketInit` 的 vnic 表同步（对 rs 只是转发"用你自己的表"，无害），rs socket 层接受 `AF_INET6`；无 v6 设备侧可达性时以 connect 超时显形（保留 NOTICE 面包屑）。UBC_TP channel 对 v6 是全链原生的。
- `detect_local_eid` / `acquire_global_ctx` 的 env fallback（`SPDK_HCOMM_LOCAL_IP`）**必须判空**——原缺判空是 NULL 解引用段错误（本机实证 SIGSEGV）。`acquire_global_ctx` 在 `RaGetDevEidInfoNum` 失败/为空时可把该 env 按 IPv6 填 `attr.ub.eid`（`union HccpEid.raw` 网络序）。
- **`RaGetSockets` 是轮询契约**：`RaSocketBatchConnect` 发起后须按 1ms 重试直到 CONNECTED（helper `nvme_hcomm_ra_socket_connect`，上限 `SPDK_HCOMM_SOCKET_TIMEOUT_MS`）；UBC_TP 控制 socket 与 NOF 侧信道共用。
- **建链 DFX（DEBUG 级）**：每个阻塞调用前有 entering 日志；状态机**只在相位迁移时**打印（poll 频率下逐次打会刷屏，用 `debug_last_phase` 跟踪）；长等待循环**限频 1 条/秒**；数据面（submit/completions/mem_map）**刻意不打**。审计规则 = **只看最后一行日志就能判识当前阶段与阻塞点**。
- 本机环境类修复：`-lc_sec` 必须链接；`xlation_for_cid` 用 `SPDK_MALLOC_SHARE`（DPDK 下 flags=0 分配会失败）；`inet_pton` 目标缓冲不能传 NULL。

### 8.2 进行中 / 待做（见第 9 节问题清单）

- 修正 `lib/nvme/Makefile` 的 hcompat 自动修复（问题 14），消除 §5.5 手工第 2 步。
- 清理 `nvme_hcomm.c` 里残留的 `HcommSendOnThread` 旧注释（数据面已改 `HixlBatchSend`；见 `:74/:119/:122/:692/:1071/:1154/:3863/:4787`）——陈旧注释会误导排查。

---

## 9. 潜在 bug / 已知问题清单（改代码前必读）

> ✅ = 已修（括号内为修复 commit）；⚠️ = 仍开放，需要处理。

### 🔴 阻断级

1. ⚠️ **数据面运行时依赖与前置条件**（原「`HcommSendOnThread` 未定义符号」条目已随架构切换失效：`nvme_hcomm.c` 现已不调用该符号，仅注释残留旧名）。三条硬依赖：① **HIXL 算子已装进 CANN opp 路径**（§6.3 第 3 条；缺失时 `aclrtBinaryLoadFromFile ... failed` + `-EIO`）；② device 侧 `libccl_kernel` 提供 `HcommSendWithImmOnThread`；③ 🔴 **submit 线程必须有 ACL 设备上下文**——`aclrtSetDevice` 是 thread-local，本仓只在 `ctrlr_construct`（`:3161`）与 connect worker（`:4079`）调用，**submit 路径不做保证**，跨线程提 IO 时 `aclrtLaunchKernelWithConfig` 会以 `RT_ERROR_CONTEXT_NULL` 失败。详见 §2.6 ⑤-1。

### 🟠 功能缺口

2. ✅ **队列深度单一来源**（`3379d51` + `cc69438`）：`qsize` 成为深度唯一来源（`create_queues` 不再重读 `ctrlr->opts`，per-qpair 覆盖不再被吞），去掉 pow2 取整，req 池 `num_requests` 收敛到有效深度避免白分配。原「双源 + req/cid 池失配 → 静默降深度」已消除。
3. ✅ **submit_request 数据面风险**（support-multi-sgl 分支）：原实现 `req->payload.contig_or_cb_arg` 被直接当数据缓冲不查 `payload.type`（`d955216` 曾以一律 `-EINVAL` 拒绝 SGL payload 兜底）。现已实现多段 SGL：submit 按 `nvme_payload_type()` 分支，SGL payload 走 `reset_sgl_fn`/`next_sge_fn` 逐段注册（每段独立 tokenId）+ 尾部描述符数组，`get_max_sges()` 协商返回 min(16, ICD, MSDBD) 且走查执行该上限。TGT 需按 `hcomm_multi_sge_design.md` 升级解析器（变长接收 + DPTR 描述符类型分支：单段 keyed / 多段 LAST_SEGMENT 计数）。
4. ⚠️ **`qpair_reset` 空实现**（返回 0，`nvme_hcomm.c:3872`）：reset 语义缺失（与 `nvme_rdma.c` 一致，非 HCOMM 独有）。
5. ⚠️ **`ctrlr_get_memory_domains` 空实现**（返回 0，`nvme_hcomm.c:3817`）：上层拿不到内存域。
6. ⚠️ **`ctrlr_enable` no-op**（`nvme_hcomm.c:3027`）；`ctrlr_ready`/寄存器读写依赖 base（`nvme_fabric_ctrlr_*`），无 transport 自定义逻辑。
7. ✅ **in-flight 收割**（`2ba6c44`）：`delete_io_qpair` 现在先调 `nvme_hcomm_qpair_abort_reqs(qpair, 0)` 再 `disconnect_channels`/`destroy_queues`/`qpair_deinit`，杜绝了「在途请求引用已释放的 cqe_buf/req_for_cid → use-after-free + 完成泄漏」（镜像 `nvme_rdma_ctrlr_delete_io_qpair`）。connect/disconnect 回调链路的真机闭环验证仍待做（遗留项）。

### 🟡 风险 / 债务

8. ⚠️ **`nvme_hcomm_socket_desc` 私有结构镜像**（`nvme_hcomm.c:3280`）：libhcomm 私有 `SocketDesc` 的镜像复制，升级库时 ABI 可能变，**升级 libhcomm 必须复核此结构**。
9. ⚠️ **UBOE 预建 socket workaround** 依赖库内行为（`AicpuTsUboeChannel` 按 IP 字节比较定角色），库升级需回归。
10. ✅ **`RaGetSockets` 单次调用**（`c03b6ca`）：`RaSocketBatchConnect` 异步发起后，共享 helper `nvme_hcomm_ra_socket_connect` 按 1ms 间隔轮询 `RaGetSockets`（`SOCK_EAGAIN` 重试，HCCP 内部超时 status==2 立即终止），上限 `SPDK_HCOMM_SOCKET_TIMEOUT_MS`（默认 30s 安全网）；UBC_TP 控制 socket 与 NOF 侧信道共用。
11. ⚠️ **`exchangeAllMems=true` 全量内存交换**：每个 channel 把 endpoint 全部注册内存全量交换，多 qpair 时 O(N²)。重构目标：`exchangeAllMems=false` + `desc.memHandles={sq,cqe}` 只交换本 qpair 两段。
12. ⚠️ **死代码保留为 `#if 0` 参考**：UBC 手动 bringup 12+ 函数（`#if 0` 区 ~550 行，L4720–5272）保留在文件尾部；`jetty_key_info` 结构体和 ctrlr 10 个死字段也以 `#if 0` 包裹，不参与编译。已被 `HcommChannelCreate` 取代，仅供恢复参考。
13. ⚠️ **SQE 格式无协商**：INI 与 TGT 必须 lockstep 讲同一格式，无 wire format marker。128B 格式已临时移除（见 §2.2 注），当前 96B 单格式下跨格式位序分支问题不复存在；将来若从 git 历史恢复 128B，需重新引入格式选择逻辑并注意路由字位序差异（96B DW22: ctrlid 低16位；128B DW30: sqid 低16位）。
14. ⚠️ **`lib/nvme/Makefile` 的 hcompat 自动修复失效**：`lib/nvme/Makefile:30-31` 的 sed 源路径写错（`include/hcomm_res_defs.h` 少了 `hcomm/` 子目录），生成的是空 `build/hcompat/hcomm_res_defs.h`，`|| true` 又吞掉错误——所以 `COMM_ADDR_EID_LEN` 的 enum 修复目前只能靠手工改 `$CANN_MERGED/include/hcomm/hcomm_res_defs.h`（§5.5 第 2 步）。干净做法：修正 sed 源路径（或直接上游提交 enum 修复）后，删除该手工步骤。
15. ✅ **128B SQE 丢失 CDW10-15**（`e5ac525`）：128B `sqe128_fill` 原先只复制 DW0-3（16B），非读写命令（Fabric 0x7f、admin 等）的 CDW10-15（含 FCTYPE）丢失。已修复：非读写命令时 memcpy CDW10-15（24 字节）到 DW16-21。96B 格式复制完整 64B 无此问题。
16. ✅ **process_completions 缺 qpair 状态检查**（`903c886`，后由 `ba8a58a` 演进）：原实现断开时可能读 stale cqe_buf，先以非 `CONNECTED` 早返回修复；三阶段连接改造后 CONNECTING 期间也需收 CQE（fabrics CONNECT 应答），改为 `active_cids` 门控（只遍历在途 cid）+ `qpair_connect` 复位 `cqe_buf`（防止重连后 stale valid 标志伪造完成）。
17. ✅ **submit_request 冗余 memcpy**（`3de9db6`）：96B 和 128B 路径在 `sqe_fill` 后再 memcpy DW0-3 传播 PSDT 变更。已修复：PSDT 在 fill 前设置，fill 一次性复制，消除 re-copy。
18. ⚠️ **拆除顺序契约：所有 `HcommMemUnreg` 必须先于 `RaCtxDeinit`**（库源码核实，2026-09）：`RaCtxDeinit` → `RsCtxDeinit`（rs_ctx.c:206）→ `RsUbCtxDeinit`（rs_ub.c:1127）→ `RsUbFreeSegCbList` **批量 urma 注销该 ctx 全部注册段**（含 HcommMemReg 注册的 SQ/CQE/payload）+ `RsUrmaDeleteContext`；而 `RaSocketDeinit` 只拆 TCP conn/listen/white 节点（rs.c:1652→RsFreeSocketList），**不碰 UB ctx/注册内存**——"RaSocketDeinit 释放注册内存导致 double free"的论断已证伪。当前 `ctrlr_destruct` 顺序正确（adminq/payload 全部 unreg → `HcommEndpointDestroy` → `RaCtxDeinit`），改拆除顺序时必须保持 unreg 在前。另：库任何路径都不 free 缓冲内存本体（只 urma 注销），缓冲归 INI 侧 `spdk_free`；ctx 销毁后再 `HcommMemUnreg` 走 rs 侧 -ENODEV 报错路径（按 addr 查 segCb 已空），非 double-free。
19. ✅ **`HcommMemExport` 所有权契约（cleanupSIGABRT 根因，2026-09）**：`UbRegedMemMgr::MemoryExport`（urma_mem.cc:98-99）返回的是 **`LocalUbRmaBuffer::Desc`（std::vector\<char\> 成员）的 `data()`——借出指针，归库所有**，不是 malloc 的 blob。INI 侧曾在 CQE 导出与 mem_map payload 路径 `free(desc)`（共 4 处）：第一次 free 释放 vector 内部缓冲（glibc 同堆不立即崩），后续 `HcommMemUnreg` → `~LocalUbRmaBuffer` → vector 析构 `operator delete` 同一指针 → **`free(): double free detected` SIGABRT**——与 cleanupSIGABORT 报告 backtrace（free 直挂 ~LocalUbRmaBuffer 帧）逐帧吻合；日志里 `RaSocketDeinit` 紧邻 `HcommMemUnreg` 只是 NOF 失败路径的顺序拆除（时间相关，非因果）。修复：4 处 `free(desc)` 全部移除，两处补 BORROWED 所有权注释 + `export_tokenid` 文档注明。**教训：任何 Hcomm* 接口返回的指针，先到库源码确认所有权再决定是否 free。**
20. ✅ **send 失败误用 `-EAGAIN` → fabrics CONNECT 假成功（对照 rdma 修复）**：`HcommSendOnThread` 失败时 `send_sqe` 曾返回 `-EAGAIN`。base 层 `nvme_qpair_submit_request`（nvme_qpair.c:1109）会把 transport 的 `-EAGAIN` **吞掉转成"请求进 `queued_req` + 返回 0"**——于是 `nvme_fabric_qpair_connect_async`（用 `reserved_req` 提交 CONNECT）报成功、connect_poll 进 FABRIC_SENT 等 CQE，但 CONNECT 从未发出：重发由完成数驱动（`nvme_qpair_resubmit_requests(qpair, ret)`，无 CQE 即永不重发；ENABLED 转换时的 drain 也到不了），`transport_failure_reason=LOCAL` 仅在 **ENABLED** 状态被 `check_enabled`（nvme_qpair.c:678）消费——只能等看门狗（默认 300s）超时。修复：send 失败改返回 **`-EIO`**（fatal）。契约对照 rdma：其唯一 `-EAGAIN` 是 `nvme_rdma_req_get` req 池耗尽（暂时性），post 失败以**错误完成**传播、绝不 -EAGAIN；hcomm 保留 `reserve_cid` 的 `-EAGAIN`（cid 池耗尽，同语义）。reserved_req 在 base error 路径安全（`_nvme_free_request` 特判不回池，nvme_internal.h:1345）。**教训：transport `-EAGAIN` 的 base 层语义是"排队重试 + 报成功"——只可用于真正暂时性的资源不足，通道级失败必须 fatal。**

---

## 10. 开发规范与注意事项

- **中文沟通**；代码风格严格遵循 **§7.3 代码风格**（与 SPDK 一致，参考同目录 `nvme_rdma.c`/`nvme_tcp.c` 写法，机器格式见 `.astylerc`）。
- 调试日志：`SPDK_LOG_FLAGS=nvme_hcomm SPDK_LOG_LEVEL=debug`。
- 错误码速查：`rc=128003` → RaCtxInit 前缺设备 EID；`rc=128000` → hccp_service 未正确拉起（`--hdcType=18`）。
- 改 `nvme_hcomm.c` 注意 libhcomm ABI 版本（v1 toolkit vs v3 源码仓）；`SocketDesc` 镜像结构升级库时需复核。
- 真机结论多来自 `nids_aiv_ub` 仓对齐验证（代码注释标注出处行号）。
- 每个 qpair = N 条 TCP + N jetty，注意 TGT 连接数与设备 jetty 容量上限。

---

## 11. 术语表

| 术语 | 含义 |
|---|---|
| UB / UnifiedBus（灵衢） | 华为统一总线网络及语义 |
| UBOE / UBC_TP | UB over Ethernet / UB 原生协议（两种 profile，均 CONN_RM） |
| Jetty / JFS / JFR / JFC | UB 事务层队列：全功能 / 专发送(SQ) / 专接收(RQ) / 完成队列(CQ) |
| CONN_RM / RC | 可靠消息（单边，无 bind）/ 可靠连接（需 bind） |
| CTP / RTP / UTP | 域内链路层可靠 / 可路由 / 不可靠（TP=Transport Point，传输层实体） |
| TA / TAACK | 传输层确认重传机制 / 其确认报文 |
| RTSQ | Runtime Task Stream Queue：AI CPU 的硬件任务流（每 thread 一条，绑芯片 SQ；硬件消费非 OS 线程） |
| hcomm transport | hcomm 库在 Device（AI CPU）侧的传输层：执行双边 SEND（`HcommSendOnThread`，仅 A5/950 支持）与 UB 收发，SQE 经 channel 直达 TGT |
| NOF 侧信道 | 独立 Ra\* socket，per-qpair 握手帧传 qid/队列参数 |
| 引擎（CommEngine） | CPU / AICPU(_TS) / AIV / CCU——决定数据面执行位置与 channel 类 |
| EID / INI / TGT | UB 端点标识(16B) / NVMe-oF 发起方 / 目标方 |
| NOF | NVMe over Fabrics |
