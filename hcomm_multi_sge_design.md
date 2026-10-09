# HCOMM 多 SGE（scatter-gather）支持 — 已实施方案（TGT 实施契约）

> **状态：方案 A 已在 INI 侧实施（本分支）。TGT 按本文 §3 线格式契约与 §5 解析规则实施。**
> 采纳日期：2026-09 · 设计预案：2026-09-10（方案 A/B/C 对比见文末附录）
> 变更记录：2026-09 **删除 `nsgl` 计数 dword**——段计数收敛到 DPTR `LAST_SEGMENT`
> 描述符单一信源（rdma / 标准 NVMe-oF "SGL in capsule" 同款约定）；传输层已
> 源码核实为可靠消息（CONN_RM + TA/TAACK ARQ，字节精确、保消息边界），双
> 编码交叉校验无实际对象，随双编码一并删除（见 §3.4 / §5 决策 6）。
> 分支：support-multi-sgl（基于 refactor-hcomm-globals）
> 对照实现：`nvme_rdma.c` 的 `nvme_rdma_build_sgl_request`（:1599）— 本方案与其机制同构

---

## 1. 机制总览（与 RDMA 的同构关系）

RDMA 能携带多段不是因为 NVMe 命令装得下 N 个描述符，而是它的**胶囊**结构
`struct spdk_nvmf_cmd { cmd; sgl[16]; }` 把描述符数组挂在命令后面、随 SEND
一次发送（发送长度 = 64 + N×16，变长；**胶囊存储本身是定长槽**）。

HCOMM 的 SQE 槽与之完全同构化：

| 机制 | RDMA | HCOMM（本方案） |
|---|---|---|
| 描述符存储 | 胶囊尾部 `sgl[16]`，cid 索引定长槽 | SQE 槽尾部 `sgl[16]`，cid 索引定长槽（stride = base + 256B） |
| 发送长度 | `64 + N×16` 变长 | `base + N×16` 变长（`HcommSendOnThread` 带 length 参数） |
| 段计数 | DPTR 放 `LAST_SEGMENT+OFFSET` 描述符（length=数组大小，N=length/16） | **同——DPTR LAST_SEGMENT 是唯一计数信源**（rdma / 标准 NVMe-oF "SGL in capsule" 约定） |
| 单段 | DPTR 直接放 keyed data block | 同（DPTR 描述符 type 区分单/多段，字节流与旧格式**完全一致**） |
| 逐段注册 | 每段独立过 `mr_map` 拿 rkey | 每段独立过 `mem_map` 拿 tokenId（引用计数防驱逐） |
| 段上限 | 16，且被 ICD/MSDBD 协商夹紧 | **16（NVME_HCOMM_MAX_SGL）槽位容量**；有效上限按 Identify 的 ICD/MSDBD 夹紧（TGT 未上报回落 16） |

## 2. 已实施的 HOST 侧改动（本 PR）

1. **线格式**（`nvme_hcomm_sqe.h` / `nvme_hcomm_sqe_96b.h`）：新增尾部
   `sgl[NVME_HCOMM_MAX_SGL]` 描述符数组，全部偏移用
   `SPDK_STATIC_ASSERT` 钉死；两个独立单测（零依赖 gcc 直编）覆盖布局与
   填充行为。段计数**不设专用字段**——由 DPTR `LAST_SEGMENT` 描述符的
   24-bit length 唯一携带（原 `nsgl` dword 方案已删除，见 §3.4）。
2. **submit**（`nvme_hcomm_qpair_submit_request`）：SGL payload 走
   `reset_sgl_fn`/`next_sge_fn` 逐段循环（照抄 rdma），逐段
   `get_memory_translation` 注册拿 tokenId，描述符写入尾部数组；
   **单段 SGL 折叠回 DPTR 路径**（keyed 描述符直写 DPTR，旧 TGT 字节流
   兼容）；多段 DPTR 放 `LAST_SEGMENT+OFFSET` 计数描述符
   （`length = N×16`，唯一计数编码）。CONTIG 路径不变。
   原 `d955216` 的 SGL 拒绝路径移除。
3. **引用计数**：`xlation_for_cid` 从单条 `xlation_ref` 扩为
   `{count, refs[16]}`（每段一条），完成/abort/删除路径统一
   `release_all` 释放——LRU 驱逐绝不会反注册在途 tokenId。
4. **槽位**：`sq_buf` 分配与 `HcommMemReg` 注册按 **stride**
   （96B 格式 352B / 128B 格式 384B）；SEND 长度按段数变长。
5. **`get_max_sges()`** 协商上报：min(16, ICD, MSDBD)（照 rdma
   `:2799-2818`），TGT 未上报回落 16；submit 走查执行同一上限
   （`ctrlr->max_sges`，未协商回落 16，见 §4.5）。

## 3. 线格式契约（TGT 实施依据）

### 3.1 96B 格式（`nvme_hcomm_sqe_96b.h`）

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0–63 | 标准 `spdk_nvme_cmd` | 不变（DPTR 在 cmd.dptr.sgl1，偏移 24） |
| 64–87 | rsvd（6 dword） | 恒零（原 nsgl dword 位置，已回退保留） |
| 88 | 路由字 | 不变（ctrlid 低16 \| sqid 高16） |
| 92 | jetty token | 不变 |
| **96–** | **尾部描述符数组** | `N × 16B`，偏移 96 + i×16，N 见 §3.4 |

槽位容量：96 + 16×16 = **352B**（HOST 侧 sq_buf 槽 stride）。

### 3.2 128B 格式（`nvme_hcomm_sqe.h`）

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0–95 | 现有 128B 布局 | 不变（MPTR/DPTR/NVM 字段） |
| 96–119 | rsvd（6 dword） | 恒零（原 nsgl dword 位置，已回退保留） |
| 120 | 路由字 | 不变（sqid 低16 \| ctrlid 高16） |
| 124 | jetty token | 不变 |
| **128–** | **尾部描述符数组** | `N × 16B`，偏移 128 + i×16，N 见 §3.4 |

槽位容量：128 + 16×16 = **384B**。

### 3.3 描述符线格式（与 DPTR 内现有关键字描述符完全相同）

每段 16B，标准 `struct spdk_nvme_sgl_descriptor` keyed 视图（LE）：

```
+0   address（u64）      —— 该段的 HOST 虚拟地址
+8   length（u24）       —— 该段长度
+11  key（u32）          —— 该段的 tokenId（逐段独立注册）
+15  [7:4]=type=0x4(KEYED_DATA_BLOCK)  [3:0]=subtype=0(ADDRESS)
```

### 3.4 语义（单段/多段由 DPTR 描述符 type 区分，段计数单一信源）

- **单段（含全部 CONTIG payload）**：DPTR（96B: `cmd.dptr.sgl1`；
  128B: `data_ptr[0..15]`）放一个 keyed data block 描述符
  （type=0x4），发送长度 = base（96/128）。**字节流与扩展前完全一致
  ——未升级的 TGT 继续正常工作。**
- **多段（N ≥ 2）**：
  - N 个 keyed 描述符在尾部数组（§3.1/§3.2 偏移）；
  - DPTR 区域放一个 `LAST_SEGMENT + OFFSET` 计数描述符：
    `unkeyed.type=0x3(LAST_SEGMENT), subtype=0x0(OFFSET),
    length = N×16, address = 0`——**这是唯一计数编码**（rdma
    `build_sgl_request` / 标准 NVMe-oF "SGL in capsule" 同款约定），
    TGT 取 `N = length / 16`；
  - 发送长度 = base + N×16。

**为什么不再设第二个计数信源（原 `nsgl` dword 已删除）**：本栈传输层
已源码核实（hcomm 源码树）——`HcommSendOnThread` → `UbConnLite::Send`
走单 WQE SGE 引用（`sqe->length` 原样携带精确字节数，分片上限 256MB
远大于 384B），TP 层硬编码 `CONN_RM` 可靠消息（`tp_mgr.cc`，TA/TAACK
ARQ + 选择性重传），RM 语义保消息边界——传输路径**字节精确、无
padding/截断**。双编码的交叉校验因此无实际校验对象（不一致只剩硬件
CRC 漏检这种极低概率事件，两个编码同时错同样骗过校验）；而 TGT 的
参考实现（照 nvme_rdma.c）本来就只解析 DPTR LAST_SEGMENT。单一信源
让契约与 TGT 实现、与标准 NVMf 语义完全对齐，DW16（96B）/DW24（128B）
回到保留清零。

## 4. 对 TGT 的要求

1. **接收侧接受变长**：单条 SQE 的 SEND 长度在 [base, base+256] 内变化
   （最大 352B / 384B），按实际收到的长度处理，不得假定恒定 base。
2. **解析分支**（伪代码，与 nvme_rdma TGT 侧同构）：
   ```
   dp = DPTR 描述符（96B: sqe+24；128B: sqe+32）
   type = dp[15] >> 4
   if (type == 0x4):                 /* KEYED_DATA_BLOCK */
       单段：dp 即数据描述符（现行为，不变）
   elif (type == 0x3):               /* LAST_SEGMENT */
       if (dp.length % 16 != 0):     fail_cmd()
       n = dp.length / 16
       if (n > 16):                  fail_cmd()  /* 槽位容量 */
       if (recv_len != base + dp.length): fail_cmd()  /* RM 保边界 */
       for i in 0..n-1:
           desc[i] = sqe + tail_offset + i*16   /* 96B:96  128B:128 */
   else:
       fail_cmd()                    /* 未知描述符类型 */

   /* fail_cmd() = 经 MPTR/cid 回错误完成（如 Invalid Field），记录
      type / LS.length / recv_len 三值供诊断；连续出现则断连排查 */
   ```
   （recv_len 精确匹配是防 TGT 自身接收实现 bug 的 sanity 检查——
   传输层契约保证二者恒等，见 §3.4。）
3. **逐段数据搬运**：每段描述符独立携带 tokenId/addr/length，TGT 对每段
   独立发起 keyed RDMA 读写（段与段互不依赖，可并行）。
4. **MPTR/CQE/路由字/token 全部不变**；96B/128B 路由字位序差异仍需按
   sqe_size 分支（本 PR 不改位序，见 §6）。
5. **能力上报（可选，推荐）**：TGT 在 Identify Controller 响应中——
   - `cdata.sgls.supported` 必须置位（否则 HOST base 层不会走 SGL 路径）；
   - `nvmf_specific.ioccsz`（16B 单位，覆盖整条 SQE 含 base）：TGT 能接受的
     最大 SQE 总字节数 ÷ 16，如 96B 格式满配 16 段 = 352B → ioccsz=22。
     若 TGT 按标准 NVMf 语义上报（从标准 64B 命令起算，如 64+16×16=320B
     → ioccsz=20），INI 同样安全：换算公式减去 ≥64 的 sqe_base，结果只会
     更保守（最多少用 2 段），绝不超发；
   - `nvmf_specific.msdbd`：TGT 单命令能处理的最大数据段数（0 = 不限）。
   INI 侧 `get_max_sges` 按三者夹紧，且 **submit 走查执行该上限**（照 rdma
   `:1615` 用 `ctrlr->max_sges`，未协商回落 16）；未上报时按 lockstep
   缺省 16 运行（过渡期语义：只收不报的能力，绝不放大）。

## 5. 决策记录（原设计预案 §7 的决策点）

| # | 决策点 | 结论 |
|---|---|---|
| 1 | TGT 改造 | TGT 按本契约实施（用户确认） |
| 2 | 方案取舍 | **A**（尾部数组 + DPTR LAST_SEGMENT 计数），无额外 RTT |
| 3 | N_max | **16**（对齐 NVME_RDMA_MAX_SGL_DESCRIPTORS）为槽位容量；**有效上限运行时协商**：INI 按 Identify Controller 的 ICD（ioccsz）与 MSDBD（msdbd）夹紧（照 rdma `:2799-2818`），TGT 未上报（ioccsz=0 / 小于 base、msdbd=0）时回落 16 |
| 4 | 过渡策略 | **DPTR 描述符类型即兼容位**：单段命令 keyed 描述符字节流不变，新旧 TGT 可共存 |
| 5 | 路由字位序统一 | **本 PR 不做**（改位序会破坏单段字节兼容性；留待未来 lockstep 升级） |
| 6 | 段计数信源 | **DPTR LAST_SEGMENT.length 单一信源**——原 `nsgl` dword 已删除：传输层源码核实为可靠消息（CONN_RM + TA/TAACK，字节精确保边界），双编码交叉校验无实际对象；TGT 参考实现本就只读 LAST_SEGMENT；单一信源与标准 NVMf "SGL in capsule" 语义对齐，DW16/DW24 回退保留 |

## 6. 已知边界与后续

- **每 qpair 内存**：槽位 stride 增至 352/384B（qsize=1024 → sq_buf
  ≈384KB/qpair）+ per-cid 引用记录 264B——与 rdma 的胶囊+rdma_req 开销
  同阶。
- **mem_map 容量**：64 条目缓存，多段请求每段持一条引用；全忙时走瞬态
  注册（不缓存），正确性不受影响，重度多段负载下注册开销上升——缓存
  扩容/哈希化是独立的后续优化。
- **无协商 → 已补运行时协商**：段数有效上限 = min(槽位容量 16, ICD, MSDBD)，
  ICD/MSDBD 来自 Identify Controller（adminq 标准通路）；TGT 未上报时回落
  16。槽位存储（352/384B）不受协商影响——协商只收使用量，与 rdma 的
  cmds[] 固定 320B 槽同构。若将来接 FABRICS CONNECT 的完整交换，同一
  数据源继续生效，无需再改。
- **段数 >16 的请求**：submit 返回 -EINVAL（同 rdma 的 remaining>0 拒绝），
  上层需自行拆分。
- **单段长度上限 2^24-1**（keyed.length 是 24 位字段）：超限段 HOST 侧
  拒绝（-EINVAL），不会静默截断——与 rdma 的 MAX_KEYED_SGL_LENGTH 防御
  一致（CONTIG 整体与 SGL 逐段两处检查）。
- **端到端完整性**：传输层已核实为可靠消息（CONN_RM + TA/TAACK ARQ，
  字节精确、保消息边界），故不设计数交叉校验；TGT 侧保留 recv_len
  精确匹配 sanity 检查（防 TGT 自身接收实现 bug）。真机若观测到告警，
  可在 SQE 尾部或握手帧引入 CRC（lockstep 升级，独立后续）。

---

## 附录：原方案对比（2026-09-10 预案，供背景）

- **方案 A（采纳）**：定长槽 + 尾部描述符数组 + DPTR LAST_SEGMENT 计数；
  变长发送；单段 DPTR keyed 描述符向后兼容。
  （初版曾在 DW16/DW24 设 `nsgl` 计数 dword 并要求与镜像交叉校验，
  后经传输层源码核实删除——双编码无校验对象，单信源与 rdma 同构。）
- **方案 B（否决）**：SQE 线格式零改动，DPTR 描述符指向 HOST 侧描述符池，
  TGT 两阶段（先读描述符表再搬数据）——多一次串行 RTT，小 IO 多段场景
  延迟占比高。
- **方案 C（否决）**：数据随 SQE 带内发送——bounce 拷贝回归 + 全部流量
  挤 AICPU SEND 带宽，违背 zero-copy 设计。
