/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Intel Corporation. All rights reserved.
 */

/*
 * NVMe over HCOMM transport - Initiator side
 *
 * Architecture:
 *   Global shared (struct nvme_hcomm_globals g_hcomm, ref-counted):
 *     socket_handle    = RaSocketInit factory (shared by all ctrlrs)
 *     ctx_handle       = RaCtxInit factory (shared by all ctrlrs, used for device cap query)
 *     endpoint         = HcommEndpointCreate singleton (brings up the HCCP stack)
 *     acl_refcnt       = ACL device context refcount, offline mode (aclrtSetDevice)
 *
 *   Per-ctrlr (struct nvme_hcomm_ctrlr):
 *     endpoint         = copy of the global endpoint singleton handle
 *     target_addr/port = from transport_id
 *     socket_handle    = pre-created CLIENT socket (UBOE workaround, shared by all qpairs)
 *     nof_fd_handle    = NOF side-channel socket fd (shared by all qpairs)
 *     socket_lock      = mutex protecting socket_handle / nof_fd_handle creation
 *                        and NOF handshake I/O against concurrent IO qpair connect
 *     mem_map          = memory registration cache (zero-copy), shared by all qpairs
 *
 *   Per-qpair (struct nvme_hcomm_qpair):
 *     channel          = HcommChannelCreate handle (per-qpair, like rdma_qp/cm_id)
 *     thread           = HcommThreadAlloc handle
 *     cqe_buf/sq_buf   = cid-indexed NVMe queue arrays
 *
 * Connection lifecycle:
 *   ctrlr_construct   -> init settings + acquire acl/endpoint/ctx + create adminq
 *   ctrlr_enable      -> no-op
 *   create_io_qpair   -> allocate cqe/sq arrays + HcommThreadAlloc
 *   connect_qpair    -> (per-qpair, three-phase async) a worker thread runs
 *                        HcommChannelCreate + NOF handshake, then the poll
 *                        path drives the NVMe-oF fabrics CONNECT
 *   disconnect_qpair -> (per-qpair) HcommChannelDestroy
 *   delete_io_qpair  -> join worker + abort in-flight + HcommThreadFree
 *                        + free cqe/sq arrays
 *   ctrlr_destruct   -> delete adminq + release sockets + release endpoint/ctx/acl
 *
 * NOTE: The manual UB bringup path (JFC/Jetty create → exchange → import → bind)
 *       has been superseded by HcommChannelCreate. The old functions are
 *       preserved in a DEAD CODE area at the end of this file (#if 0).
 */

/*
 * ======= INI-SIDE DATA PLANE ARCHITECTURE (HOST + Device AI CPU) =======
 *
 * Topology:
 *
 *   ┌─────────────────────────────────────────────────────────────────────┐
 *   │ INI Side (this node)                     TGT Side (remote NVMe)     │
 *   │                                                                     │
 *   │  ┌──────────┐        ┌───────────┐        ┌──────────────────────┐  │
 *   │  │  HOST    │◄──────►│  Device   │◄─UB────│      TGT             │  │
 *   │  │ (普通CPU)│ HCOMM  │ (AI CPU)  │ Jetty │  (Target NVMe SSD)   │  │
 *   │  │          │        │           │        │                      │  │
 *   │  │ cqe_buf  │        │  hcomm    │        │  UB网口在Device侧    │  │
 *   │  │ (poll)   │        │ transport │        │  IO收发在AI CPU完成  │  │
 *   │  └──────────┘        └───────────┘        └──────────────────────┘  │
 *   │       │▲                    │▲                                      │
 *   │       ││                    ││                                      │
 *   │  ┌────┴┴────────────────────┴┴──────────────────────────────┐       │
 *   │  │            Shared Memory (HOST ↔ Device)                 │       │
 *   │  │  ┌────────────────┐  ┌────────────────────────────────┐  │       │
 *   │  │  │  cqe_buf       │  │  sq_buf                        │  │       │
 *   │  │  │ (phys-contig)  │  │  (cid-indexed)                 │  │       │
 *   │  │  │  CQE slots     │  │  spdk_nvme_cmd entries         │  │       │
 *   │  │  │  indexed by cid│  │  Read/Write/Admin CMDs         │  │       │
 *   │  │  └────────────────┘  └────────────────────────────────┘  │       │
 *   │  └──────────────────────────────────────────────────────────┘       │
 *   │                                                                     │
 *   │  NOTE: UB network interface is on the Device (AI CPU) side.         │
 *   │  HOST cannot directly access the UB fabric.  HcommSendOnThread      │
 *   │  (A5 only) hands the SQE to the Device-side hcomm transport,        │
 *   │  which delivers it to the TGT (the channel peer) via the            │
 *   │  bilateral SEND - no Device-side helper application exists.         │
 *   └─────────────────────────────────────────────────────────────────────┘
 *
 * Two shared memory regions (HOST ⇄ Device):
 *
 *   cqe_buf (CQE completion array) — completed NVMe CQ entries:
 *     - SHARED memory between HOST and Device (DMA/IO-coherent)
 *     - REQUIRE physically contiguous + cacheline-aligned memory
 *     - cid-indexed array (NOT a ring): TGT writes at cqe_buf[cid]
 *     - Each slot is struct nvme_hcomm_cqe_slot (64B: 16B spdk_nvme_cpl
 *       + 48B padding; padding[0] is the valid flag)
 *     - HOST polls cqe_buf[cid].padding[0] for each in-flight cid
 *
 *   sq_buf (Submission Queue) — NVMe submission entries:
 *     - SHARED memory between HOST and Device
 *     - cid-indexed array (mirrors nvme_rdma's cmds[rdma_req->id]):
 *       allocating a cid == allocating an SQE slot (unified, no
 *       separate ring / doorbell / full-check)
 *     - Each slot is hqpair->sqe_stride bytes (96B base SQE + tail
 *       SGL area)
 *     - cmd.cid (DW0) = cid (TGT uses it to locate cqe_buf[cid])
 *     - DPTR = Keyed SGL {key=tokenId, address=user_buf, length}
 *       (zero-copy, mirrors nvme_rdma's keyed SGL with rkey)
 *
 *   User payload buffers — zero-copy (no bounce buffer):
 *     - Registered on-the-fly via mem_map (cached HcommMemReg + HcommMemExport)
 *     - TGT accesses user buffers directly via UB RDMA using tokenId
 *     - exchangeAllMems=true: all registered memory auto-exchanged with peer
 *
 * Flow for a single NVMe read/write (single-phase protocol):
 *
 *   ┌────────────────────────────────────────────────────────────────────┐
 *   │  CMD Submission  (HOST → Device → TGT)                             │
 *   │                                                                    │
 *   │  [1] HOST (SPDK): submit_request() is called                       │
 *   │       └─ Reserve cid: req_for_cid[cid] = req                       │
 *   │       └─ Build SQE at sq_buf[cid]: cmd.cid (DW0)=cid               │
 *   │       └─ DPTR = Keyed SGL {key=tokenId, address=user_buf, length}  │
 *   │          (multi-SGL: + tail descriptor array, DPTR carries a        │
 *   │           LAST_SEGMENT count mirror; SEND length grows to           │
 *   │           sqe_size + count*16)                                      │
 *   │       └─ No bounce-copy (zero-copy, TGT accesses user_buf directly)│
 *   │       └─ spdk_wmb(); HcommSendOnThread(thread, channel, sqe, size)│
 *   │                                                                    │
 *   │  [2] Device hcomm transport (per-command, from submit_request):    │
 *   │       └─ HcommSendOnThread enqueues the bilateral SEND on the      │
 *   │          A5 task stream: the transport reads the SQE from          │
 *   │          sq_buf[cid] (variable length: 96B SQE + tail SGL          │
 *   │          descriptors) and delivers it to the TGT (channel peer)    │
 *   └────────────────────────────────────────────────────────────────────┘
 *
 *   ┌────────────────────────────────────────────────────────────────────┐
 *   │  Data Transfer & Completion  (TGT → HOST)                           │
 *   │                                                                    │
 *   │  [3] TGT processes the SQE:                                        │
 *   │       └─ Reads DPTR keyed SGL (tokenId + user_buf addr + length)   │
 *   │       └─ Reads cmd.cid (DW0) to locate cqe_buf[cid]                │
 *   │       └─ For READ:  TGT reads from NVMe SSD, RDMA-writes to        │
 *   │          user_buf directly (zero-copy, fire & forget)              │
 *   │       └─ For WRITE: TGT RDMA-reads from user_buf directly,         │
 *   │          writes to NVMe SSD (zero-copy)                            │
 *   │                                                                    │
 *   │  [4] TGT writes CQE directly into cqe_buf[cid]:                    │
 *   │       └─ RDMA-write 16B spdk_nvme_cpl into cqe_buf[cid].cpl      │
 *   │       └─ Set cqe_buf[cid].padding[0]=1 (valid flag, with wmb)     │
 *   │                                                                    │
 *   │  [5] HOST polls and completes (process_completions):               │
 *   │       └─ Scan active_cids[] for in-flight cids                     │
 *   │       └─ Check cqe_buf[cid].padding[0] (valid flag)               │
 *   │       └─ assert(cqe_buf[cid].cpl.cid == cid)                       │
 *   │       └─ No bounce-copy (data already in user_buf)                 │
 *   │       └─ nvme_complete_request(req, &cqe_buf[cid].cpl)            │
 *   │       └─ memset cqe_buf[cid]; free cid (after memset)             │
 *   └────────────────────────────────────────────────────────────────────┘
 */

#include "spdk/stdinc.h"

#include "acl/acl_rt.h"      /* direct libascendcl linkage (no dlopen/dlsym) */

#include "spdk/assert.h"
#include "spdk/log.h"
#include "spdk/queue.h"
#include "spdk/nvme.h"
#include "spdk/nvmf_spec.h"
#include "spdk/string.h"
#include "spdk/likely.h"
#include "spdk/config.h"
#include "spdk/thread.h"

#include "nvme_internal.h"

/* HCOMM 96B SQE wire format (standard 64B NVMe cmd + reserved 24B + routing/token dwords) */
#include "nvme_hcomm_sqe_96b.h"

/* HCOMM transport API headers */
#include "hcomm/hcomm_res.h"
#include "hcomm/hcomm_res_defs.h"
#include "hcomm/hcomm_primitives.h"
#include "network/hccp.h"
#include "network/hccp_ctx.h"
#include "network/hccp_tp.h"

#define NVME_HCOMM_DEFAULT_NOTIFY_NUM	4
#define NVME_HCOMM_DEFAULT_ENGINE	COMM_ENGINE_AICPU_TS
/* Floor for draining an in-flight fabrics CONNECT at disconnect when no
 * fabrics_connect_timeout_us is configured (the fabric layer itself
 * never times out when the opt is 0). */
#define NVME_HCOMM_CONNECT_DRAIN_TIMEOUT_US	(5ULL * 1000 * 1000)

/* Keyed SGL descriptors carry a 24-bit length field (spdk_nvme_sgl_descriptor
 * keyed.length) - a larger single segment would silently truncate the high
 * bits and the TGT would move the wrong byte count.  Reject instead (same
 * guard as nvme_rdma's NVME_RDMA_MAX_KEYED_SGL_LENGTH). */
#define NVME_HCOMM_MAX_KEYED_SGL_LENGTH	((1u << 24u) - 1)

/* Safety-net upper bound (ms) for RaGetSockets polling.  RaSocketBatchConnect
 * is ASYNC — it may return SOCK_EAGAIN while the HCCP state machine completes
 * the whitelist handshake.  RaGetSockets is then polled until status ==
 * CONNECTED.  This timeout is NOT the real connection timeout — the HCCP
 * state machine has its own internal timeout (not configurable via API).
 * This value just needs to be >= the HCCP internal timeout so we don't give
 * up prematurely.  30s is a generous safety net. */
#define NVME_HCOMM_CONNECT_TIMEOUT_MS	30000U

/* Default for the whole-connect watchdog (worker handshake + fabrics
 * CONNECT, from the spawn).  Generous on purpose: the handshake is
 * bounded by library-internal timeouts and the fabrics phase by
 * fabrics_connect_timeout_us (120s default), so this only fires when
 * something is genuinely stuck.  Distinct from the socket timeout -
 * do not confuse the two. */
#define NVME_HCOMM_CONNECT_WATCHDOG_MS	300000U

/* Default TGT-side HCCP listening port for HcommChannelCreate's desc.port -
 * same value as the library's DEFAULT_LISTENING_PORT (aicpu channel classes),
 * so an unconfigured TGT and an unconfigured INI agree. */
#define NVME_HCOMM_DEFAULT_CHANNEL_PORT	60001U
/* Poll interval for RaGetSockets and channel-status loops (microseconds). */
#define NVME_HCOMM_POLL_INTERVAL_US	1000
/* SocketInfoT.status: 0 not connected / 1 connected / 2 timeout / 3 connecting. */
#define NVME_HCOMM_SOCKET_CONNECTED	1
#define NVME_HCOMM_SOCKET_TIMEOUT	2

/*
 * ========== Process-global state (single struct) ==========
 *
 * Everything shared across controllers lives in ONE struct, mirroring the
 * base driver's g_spdk_nvme_driver pattern (nvme.c).  The HCOMM library
 * makes this shape unavoidable:
 *   - RaSocketInit / RaCtxInit are heavyweight factories with internal
 *     resource limits (RS_MAX_SOCKET_NUM = 16); for 1000+ target support
 *     they must be shared among all controllers;
 *   - HcommEndpointCreate brings up the whole HCCP stack (rs subprocess,
 *     JFC/TP pools cached per rdmaHandle) — process-wide by construction;
 *   - the deployment settings below select HOW those singletons are
 *     created (ops tables, nicPosition, UB profile), so they cannot vary
 *     per ctrlr while the singletons are shared.
 *
 * Locking:
 *   - lock guards the one-time settings resolution, the refcounts and the
 *     singleton create/destroy paths.  It is deliberately held across the
 *     blocking library calls (RaSocketInit / RaCtxInit / aclrtSetDevice):
 *     those library resources are not documented re-entrant, and
 *     these paths run only in ctrlr construct/destruct — never the IO
 *     path (submit/process_completions touch no global state).
 *   - endpoint_lock guards the endpoint singleton; it never nests with
 *     lock.
 *   - the deployment settings fields are read WITHOUT the lock once
 *     initialised: they are write-once-stable, and every reader operates
 *     on a ctrlr whose construct already acquired lock (release/acquire
 *     edge) — see nvme_hcomm_init_global_settings().
 */
/*
 * HixlBatchSend AICPU kernel launch ABI.
 *
 * The bilateral SEND_WITH_IMM primitive (HcommSendWithImmOnThread) is a
 * device-side API: the host cannot drive the Device jetty/RTSQ directly, so
 * it launches the HixlBatchSend AICPU kernel (libcann_hixl_kernel.so, installed
 * under the CANN opp path), which runs on the AI CPU and calls
 * HcommSendWithImmOnThread for each batch.
 *
 * The structs below mirror hixl_send.h (the AICPU launch ABI).  Keep the
 * sizes in sync with that header when the kernel ABI changes.
 */
struct nvme_hcomm_send_io_batch {
	uint64_t	channel;	/* ChannelHandle */
	void		*local_src;	/* SQE address (shared mem, device-visible) */
	uint64_t	len;		/* SQE length */
	uint32_t	imm_data;	/* SEND_WITH_IMM immediate */
	uint32_t	reserved;
};
SPDK_STATIC_ASSERT(sizeof(struct nvme_hcomm_send_io_batch) == 32, "io_batch ABI");

struct nvme_hcomm_batch_send_param {
	uint64_t	thread;		/* ThreadHandle */
	struct nvme_hcomm_send_io_batch *io_batches;
	uint64_t	batch_size;
	uint32_t	*status_array;
	uint32_t	timeout_ms;
	void		*stats;
	uint32_t	complete_sender_cqe;
};
SPDK_STATIC_ASSERT(sizeof(struct nvme_hcomm_batch_send_param) == 56, "batch_send_param ABI");

/* AICPU kernel JSON under the CANN opp path (libcann_hixl_kernel). */
#define NVME_HCOMM_HIXL_KERNEL_JSON_SUFFIX \
	"/opp/built-in/op_impl/aicpu/config/libcann_hixl_kernel.json"

struct nvme_hcomm_globals {
	/* ---- deployment settings: resolved once from env, then stable ---- */

	/* RaGetSockets polling / RaSocketSend/Recv retry bound in ms — NOT
	 * the real connection timeout (HCCP has its own internal one).
	 * SPDK_HCOMM_SOCKET_TIMEOUT_MS overrides (default:
	 * NVME_HCOMM_CONNECT_TIMEOUT_MS). */
	uint32_t		socket_timeout_ms;
	bool			socket_timeout_inited;

	/* TGT-side HCCP listening port for the channel socket connection
	 * (HcommChannelCreate desc.port) — a separate service from the
	 * trsvcid/NOF port.  Matches the library's own default for an unset
	 * desc.port (DEFAULT_LISTENING_PORT 60001).  Historically desc.port
	 * carried ctrlr->target_port, which only works when the TGT listens
	 * for channel connections on the NOF port too.
	 * SPDK_HCOMM_CHANNEL_PORT overrides (default 60001). */
	uint16_t		channel_port;
	bool			channel_port_inited;

	/* Whole-connect watchdog in ms (worker handshake + fabrics CONNECT,
	 * measured from the worker spawn) - the connect_poll state machine
	 * returns -EAGAIN forever without it (stuck worker, or a fabric
	 * layer that never times out when fabrics_connect_timeout_us == 0).
	 * SPDK_HCOMM_CONNECT_WATCHDOG_MS overrides (default:
	 * NVME_HCOMM_CONNECT_WATCHDOG_MS). */
	uint32_t		connect_watchdog_ms;
	bool			connect_watchdog_inited;

	/* HCCP operating environment, SPDK_HCOMM_NETWORK_MODE: "offline"
	 * (default; HOST<->NPU same node via HDC, CTP) or "peer_online"
	 * (cross-node routable Ethernet, RTP).  Drives
	 * RaSocketInit/RaCtxInit ops-table selection, nicPosition and the
	 * UB transport profile; both profiles use CONN_RM, only the TP
	 * type differs. */
	int			network_mode;
	bool			network_mode_inited;

	/* Physical NPU device id hosting the NIC (offline mode),
	 * SPDK_HCOMM_PHY_ID (default 0) — the target NPU is not necessarily
	 * phy 0. */
	unsigned int		phy_id;

	/* UB transport protocol, SPDK_HCOMM_PROTOCOL: "uboe" selects UBOE
	 * (UB over Ethernet — UboeIpv4ToEid converts the target IPv4 to an
	 * EID directly, so most NPU<->TGT deployments over Ethernet should
	 * use it); any other value or unset (default) selects UBC_TP (UB
	 * hardware NICs, EID must match the device EID list).
	 * Lazy-resolved by nvme_hcomm_protocol(). */
	CommProtocol		protocol;
	bool			protocol_inited;

	/* ---- RaSocket factory (ref-counted; first construct inits, last
	 *      destruct deinits) ---- */
	void			*socket_handle;
	int			socket_refcnt;

	/* ---- RaCtx factory (ref-counted) + device capability cache ---- */

	/* sqMaxDepth / rqMaxDepth from RaGetDevBaseAttr; clamps per-qpair
	 * depths (nvme_hcomm_qpair_sq_size).  Cleared on last release so a
	 * later acquire re-queries — the process may switch phy_id / mode
	 * mid-lifetime.  RaGetDevBaseAttr is a cheap memcpy (no IPC). */
	void			*ctx_handle;
	int			ctx_refcnt;
	struct DevBaseAttr	dev_attr;
	bool			dev_attr_inited;

	/* ---- ACL device context, offline mode only (ref-counted) ----
	 * aclrtSetDevice must run before HcommEndpointCreate: the library
	 * reads the thread-local device context and never sets the device
	 * itself (EndpointDesc.loc.device.devPhyId is ignored — overwritten
	 * internally by hrtGetDevicePhyIdByIndex).  A per-ctrlr reset used
	 * to tear the context out from under surviving controllers, hence
	 * the refcount. */
	unsigned int		acl_refcnt;
	int32_t			acl_logic_dev_id;

	/* HixlBatchSend AICPU kernel, loaded once (guarded by lock). */
	aclrtBinHandle		batch_send_bin;
	aclrtFuncHandle		batch_send_func;
	bool			batch_send_loaded;

	/* ---- endpoint singleton (ref-counted) ----
	 * Identity depends only on the local NPU (commAddr via
	 * HrtRaGetDevEidInfoList), so all controllers share it.  The HCCP
	 * service (aclrtSetDevice + TsdProcessOpen + RaInit, via
	 * HccpHdcManager::Init + HrtRaInit with enableHdcAsync=true) is
	 * brought up internally by HcommEndpointCreate — no explicit
	 * bringup needed here. */
	EndpointHandle		endpoint;
	int			endpoint_refcnt;
	EndpointDesc		endpoint_desc;

	/* ---- locks (never nested with each other) ---- */
	pthread_mutex_t		lock;
	pthread_mutex_t		endpoint_lock;
};

static struct nvme_hcomm_globals g_hcomm = {
	.socket_timeout_ms = NVME_HCOMM_CONNECT_TIMEOUT_MS,
	.channel_port = NVME_HCOMM_DEFAULT_CHANNEL_PORT,
	.connect_watchdog_ms = NVME_HCOMM_CONNECT_WATCHDOG_MS,
	.network_mode = NETWORK_OFFLINE,
	.protocol = COMM_PROTOCOL_UBC_TP,
	.lock = PTHREAD_MUTEX_INITIALIZER,
	.endpoint_lock = PTHREAD_MUTEX_INITIALIZER,
};

/* HCOMM protocol selector — returns the lazy env-resolved value (see the
 * protocol field above).  The first caller is nvme_hcomm_init_global_settings
 * (under g_hcomm.lock), so the one-time write is serialized; every later
 * caller operates on a ctrlr that already passed through construct and
 * only reads the stable value. */
static CommProtocol
nvme_hcomm_protocol(void)
{
	if (!g_hcomm.protocol_inited) {
		const char *env = getenv("SPDK_HCOMM_PROTOCOL");
		if (env != NULL && strcmp(env, "uboe") == 0) {
			g_hcomm.protocol = COMM_PROTOCOL_UBOE;
			SPDK_NOTICELOG("HCOMM protocol = UBOE (from env)\n");
		} else {
			g_hcomm.protocol = COMM_PROTOCOL_UBC_TP;
			if (env != NULL && env[0] != '\0') {
				SPDK_WARNLOG("Unknown SPDK_HCOMM_PROTOCOL='%s', using UBC_TP\n", env);
			}
		}
		g_hcomm.protocol_inited = true;
	}
	return g_hcomm.protocol;
}

/*
 * ========== Connection negotiation frame (nvmf_cm_conn_req) ==========
 *
 * Single request/response frame exchanged over the connected socket (INI sends
 * first) to swap the per-endpoint jetty identity {EID, uasid, jetty_id,
 * jetty_token, tp_handle, psn} needed for EXP jetty import under CONN_RM, plus
 * the NVMe queue parameters (ctrl_id / qid / sq_size / rq_size) and the INI
 * CQE-ring physical address (ini_cqe_addr) the target needs to DMA completions
 * back to.
 *
 * Under CONN_RM (unilateral, no bind, no opaque QpKey swap), the importer
 * rebuilds the 64B QpKey from the discrete {eid, uasid, jetty_id, trans_mode}
 * identity published here.
 * Reference: nids_aiv_ub include/host_device/ub_protocol.h (kv_cm_conn_ub_info)
 * and include/host/ub/ub_resource.h (RemoteJettyDescriptor / ImportRemoteJetty).
 *
 * Handshake (INI = client, runs in connect_qpair after RaSocketBatchConnect):
 *   1. send   nvmf_cm_conn_req   (INI jetty id + NVMe params + ini_cqe_addr)
 *   2. recv   nvmf_cm_conn_resp  (TGT jetty id + active TP + status)
 *   3. close fd
 *
 * The HCCP whitelist (256B tag) + "a5a5a\0" ack is consumed internally by
 * the HCCP state machine during RaSocketBatchConnect; the business layer
 * speaks CM frames directly (reference: nids_aiv_ub ub_oob_rasocket.cpp).
 */
/* nvmf_cm_hdr opcode / trans_type / status. */
enum {
	NVME_HCOMM_CM_OP_REQ = 0,
	NVME_HCOMM_CM_OP_RESP,
	NVME_HCOMM_CM_OP_DISCONN,
	NVME_HCOMM_CM_OP_MAX
};
#define NVME_HCOMM_CM_TRANS_HCOMM   0
#define NVME_HCOMM_CM_STATUS_OK     0

/* nvmf_cm_hdr.version. */
#define NVME_HCOMM_CM_VERSION_V3    3

/*
 * INI-side JFR token (published in req.ub_info.jetty_token so the TGT can
 * present it when importing the INI jetty).  Mirrors the TGT-side fixed
 * JFR token (nids_aiv_ub kJfrToken = 0xACFE); the INI reads the TGT's token
 * back from resp.ub_info.jetty_token and uses that for its own import.
 */
/* NVME_HCOMM_INI_JFR_TOKEN defined in nvme_hcomm_sqe_96b.h */

/*
 * CM frame header.
 *
 *   version    : frame format version (NVME_HCOMM_CM_VERSION_V3).
 *   opcode     : NVME_HCOMM_CM_OP_{REQ,RESP} — drives which sub-blocks to parse.
 *   trans_type : transport/protocol family (HCOMM=0); extensible for future
 *                transports.
 *   status     : 0 = OK; non-zero on resp means TGT rejected the req.
 *   msg_len    : total frame length including this header (LE).
 */
#pragma pack(push, 1)
struct nvmf_cm_hdr {
	uint8_t  version;
	uint8_t  opcode;
	uint8_t  trans_type;
	uint8_t  status;
	uint32_t msg_len;
};
#pragma pack(pop)
SPDK_STATIC_ASSERT(sizeof(struct nvmf_cm_hdr) == 8, "nvmf_cm_hdr must be 8 bytes");

/*
 * UB connection-manager info: the per-endpoint jetty identity + active-TP
 * fields exchanged inside nvmf_cm_conn_info.ub_info.  All fields are the
 * discrete form needed to rebuild the 64B QpKey under CONN_RM (no opaque key
 * blob is carried).
 *
 *   tp_handle   : sender-local TP handle.  req: 0 (peer EID unknown yet);
 *                resp: TGT local tpHandle → INI uses as peerTpHandle.
 *   psn         : sender tx PSN.  Peer takes it as its rx PSN.  Each side
 *                generates its own tx PSN (random) — only this one value is
 *                published; the sender's rx PSN comes from the peer's tx PSN.
 *   jetty_type  : urma_jetty (=1, TARGET_TYPE_JETTY).
 *   is_shared   : whether the jetty is shared (0 = private; reserved for a
 *                future shareTp path — currently always 0).
 *   eid[16]     : HccpEid of the publishing endpoint.
 *   uasid       : QpCreateInfo.ub.uasid (urma_jetty_id_t.uasid).
 *   jetty_id    : QpCreateInfo.ub.id (urma_jetty_id_t.id).
 *   jetty_token : the publisher's JFR token_value (kJfrToken-style); the peer
 *                presents this token when importing the publisher's jetty.
 *                (i.e. resp.jetty_token = TGT JFR token, consumed by INI
 *                import; req.jetty_token = INI JFR token, consumed by TGT
 *                import — symmetric.)
 */
#pragma pack(push, 1)
struct ub_cm_info {
	uint64_t tp_handle;
	uint32_t psn;
	uint8_t  jetty_type;
	uint8_t  is_shared;
	uint16_t rsv1;
	uint8_t  eid[16];
	uint32_t uasid;
	uint32_t jetty_id;
	uint32_t jetty_token;
	uint32_t rsvd32;
};
#pragma pack(pop)
SPDK_STATIC_ASSERT(sizeof(struct ub_cm_info) == 48, "ub_cm_info must be 48 bytes");

/*
 * Connection info: NVMe queue parameters (outer) + UB jetty identity (union,
 * extensible for future transport families) + the INI CQE-ring physical
 * address.
 *
 *   ctrl_id      : NVMe controller id (cntlid).  0 at connect_qpair time —
 *                 the controller is assigned cntlid by the target after a
 *                 successful Fabrics Connect, so the admin-qpair req carries
 *                 0 and the IO-qpair reqs carry the now-known cntlid.
 *   qid          : NVMe queue id (admin = 0, IO = qpair->qid).
 *   sq_size      : submission queue depth (NVMe SQ depth).
 *   rq_size      : receive (completion) queue depth (NVMe CQ depth).
 *   data_slot_size : unused (zero-copy, always 0; kept for ABI compat).
 *   ub_info      : UB jetty identity + active-TP fields (see ub_cm_info).
 *   ini_cqe_addr : INI CQE ring physical address (spdk_vtophys of the
 *                 physically-contiguous cqe_buf), so the TGT can DMA
 *                 NVMe completions back into it.
 *   ini_cqe_tokenid : INI CQE ring tokenId from HcommMemExport — TGT
 *                 uses it to locate the imported remote mapping.
 */
#pragma pack(push, 1)
struct nvmf_cm_conn_info {
	uint16_t ctrl_id;
	uint16_t qid;
	uint32_t sq_size;
	uint32_t rq_size;
	uint32_t data_slot_size;
	union {
		struct ub_cm_info ub_info;
	};
	uint64_t ini_cqe_addr;
	uint64_t ini_cqe_tokenid;
};
#pragma pack(pop)
SPDK_STATIC_ASSERT(sizeof(struct nvmf_cm_conn_info) == 80,
		   "nvmf_cm_conn_info must be 80 bytes");

/* Full request frame.  resp shares this shape; differs only in hdr.opcode /
 * hdr.status / ub_info content. */
#pragma pack(push, 1)
struct nvmf_cm_conn_req {
	struct nvmf_cm_hdr       hdr;
	struct nvmf_cm_conn_info conn_info;
};
#pragma pack(pop)
SPDK_STATIC_ASSERT(sizeof(struct nvmf_cm_conn_req) == 88,
		   "nvmf_cm_conn_req must be 88 bytes");

struct nvmf_cm_conn_resp {
	struct nvmf_cm_hdr       hdr;
	struct nvmf_cm_conn_info conn_info;
};
SPDK_STATIC_ASSERT(sizeof(struct nvmf_cm_conn_resp) == 88,
		   "nvmf_cm_conn_resp must be 88 bytes");

/*
 * CONN_RM jetty QpKey: under CONN_RM the peer only publishes the jetty identity
 * {EID, uasid, jetty_id, transport mode}, and the importer rebuilds the 64B
 * opaque QpKey from those discrete fields.  Layout:
 * eid@0(16) uasid@16(4) id@20(4) transMode@24(4) → 32B used in the 64B key.
 * Transport mode: RM=0x1 / RC=0x2 (equal to CONN_RM / CONN_RC).
 *
 * Preserved for reference; only referenced by the quarantined #if 0 dead code
 * area at the end of this file.
 */
#if 0
#pragma pack(push, 8)
struct nvme_hcomm_jetty_key_info {
	union {
		uint8_t  raw[16];     /* HccpEid (16B) */
		uint64_t _align8[2];  /* forces 8-byte alignment → struct pads to 32B */
	} eid;
	uint32_t uasid;
	uint32_t id;
	uint32_t trans_mode;    /* transport mode: RM=1, RC=2 */
};
#pragma pack(pop)
SPDK_STATIC_ASSERT(sizeof(struct nvme_hcomm_jetty_key_info) == 32, "jetty_key_info must be 32 bytes");
#endif

/*
 * TGT-side connection info — the INI's view of everything it must obtain from
 * the target's nvmf_cm_conn_resp.  Filled by nvme_hcomm_nof_handshake() from
 * resp.conn_info.ub_info and stored per-qpair in hqpair->tgt_info.
 *
 * NOTE: HcommChannelCreate handles UB jetty import internally, so these
 * fields are currently WRITE-ONLY — nof_handshake populates them but the
 * live code path never reads them back.  They are retained for
 * diagnostic/debugging purposes and for a future path that may need the
 * TGT jetty identity outside of HcommChannelCreate.  If no such path
 * emerges, this struct and its fill code can be removed.
 *
 * Fields group into three roles:
 *   identity   — TGT jetty identity {eid, uasid, jetty_id}
 *   path       — TGT active-TP parameters {tp_handle, psn}
 *   credential — TGT JFR token the INI presents on import {jetty_token}
 *
 * Note: under the UBC/CTP deployment the driver derives peerTpHandle internally
 * from (localEid, peerEid), so tp_handle may legitimately arrive as 0; under
 * UBoE/RTP it must be a real TGT-side value (0 ⇒ EXP import likely fails).
 */
#pragma pack(push, 1)
struct tgt_conn_info {
	/* ---- TGT identity (rebuild QpKey via nvme_hcomm_fill_jetty_key) ---- */
	uint8_t  eid[16];        /* resp.ub_info.eid   — also peerEid for RaCtxGetTpInfoList */
	uint32_t uasid;          /* resp.ub_info.uasid — QpCreateInfo.ub.uasid */
	uint32_t jetty_id;       /* resp.ub_info.jetty_id — QpCreateInfo.ub.id */

	/* ---- TGT active-TP path (fed into expImportCfg) ---- */
	uint64_t tp_handle;      /* resp.ub_info.tp_handle → expImportCfg.peerTpHandle */
	uint32_t psn;            /* resp.ub_info.psn (TGT tx) → expImportCfg.rxPsn */

	/* ---- TGT access credential ---- */
	uint32_t jetty_token;    /* resp.ub_info.jetty_token → importInfo.in.ub.tokenValue
				  * (TGT JFR token_value; INI presents it on import) */
};
#pragma pack(pop)
SPDK_STATIC_ASSERT(sizeof(struct tgt_conn_info) == 40, "tgt_conn_info must be 40 bytes");

/*
 * ======= Shared Queue Data Structures (HOST ⇄ Device AI CPU) =======
 *
 * SQE: 96B wire format (see nvme_hcomm_sqe_96b.h) - cid-indexed array
 * (sq_buf[cid], mirroring
 * nvme_rdma's cmds[cid]).  DPTR carries a keyed SGL {tokenId,
 * user_buf_addr, length}; cmd.cid (DW0) carries cid (TGT uses it to
 * CQE slot).  Multi-SGL payloads add keyed descriptors in the SQE
 * tail array (the DPTR LAST_SEGMENT mirror is the single count
 * source; SEND length grows to base+16*count); a single-segment
 * payload keeps the legacy DPTR-only layout.  Allocating a cid ==
 * allocating an SQE slot (unified, like RDMA's free_reqs / cmds[]),
 * so there is no separate ring / doorbell / full-check.
 *
 * CQE: 64B cacheline-aligned slot wrapping the standard 16B
 * `spdk_nvme_cpl`.  Indexed by cid (cqe_buf[cid]), NOT a ring —
 * TGT writes directly at offset cid × 64.  Physically contiguous
 * + cacheline aligned for safe DMA.
 */

/* CQE slot: standard NVMe completion (16B) padded to one cacheline (64B). */
struct nvme_hcomm_cqe_slot {
	struct spdk_nvme_cpl	cpl;		/* 16B standard NVMe CQE */
	uint8_t			padding[48];	/* pad to 64B cacheline */
} __attribute__((aligned(64)));
SPDK_STATIC_ASSERT(sizeof(struct nvme_hcomm_cqe_slot) == 64,
		   "cqe_slot must be 64 bytes (cacheline)");

/*
 * Number of entries per queue (cqe/sq) — these are per-qpair cid-indexed
 * arrays shared between HOST and the Device AI CPU.  They are NOT hardcoded:
 * at runtime they are derived from
 *     spdk_min(opts.{io,admin}_queue_size, g_hcomm.dev_attr.sqMaxDepth)
 * (no power-of-two rounding — cid-indexed flat arrays, no ring wrap).
 * See nvme_hcomm_qpair_sq_size() in the queue-lifecycle section.  The same
 * value is also sent to the TGT as nvmf_cm_conn_req.sq_size / rq_size so the
 * INI's array capacity, the Jetty's per-SQE slot count, and the TGT's per-
 * qpair recv-buffer count are all consistent.
 */

/*
 * ========== Per-qpair UB memory registration ==========
 *
 * Each qpair registers two fixed HOST memory regions on the endpoint via
 * HcommMemReg at connect time (deferred from create_qpair to keep the
 * latter non-blocking) so the TGT / Device can access them:
 *
 *   SQ  (sq_buf) : cid-indexed NVMe cmd array; HOST builds each SQE
 *                   at sq_buf[cid] and pushes it to the TGT via the
 *                   bilateral SEND (HcommSendOnThread).
 *   CQE (cqe_buf) : TGT RDMA-writes NVMe completions at
 *                   cqe_buf[cid]; HOST polls padding[0] per cid.
 *
 * Per-request user payload buffers are registered on-the-fly via mem_map
 * (zero-copy, mirror nvme_rdma's spdk_rdma_mem_map) — no bounce buffer.
 * The CQE tokenId is published to the TGT in the nvmf_cm_conn_req frame.
 */

static int
nvme_hcomm_acl_acquire_device(void)
{
	int32_t logic_id;
	int rc;

	/* Ref-counted, guarded by g_hcomm.lock (same pattern as
	 * nvme_hcomm_acquire_global_ctx): the first ctrlr calls aclrtSetDevice;
	 * later ctrlrs just bump the count, and only the last
	 * nvme_hcomm_acl_release_device() resets the device.  A per-ctrlr reset
	 * used to tear the thread-local ACL context out from under surviving
	 * controllers. */
	pthread_mutex_lock(&g_hcomm.lock);
	if (g_hcomm.acl_refcnt > 0) {
		g_hcomm.acl_refcnt++;
		pthread_mutex_unlock(&g_hcomm.lock);
		return 0;
	}

	logic_id = (int32_t)g_hcomm.phy_id;

	SPDK_DEBUGLOG(nvme_hcomm, "calling aclrtSetDevice(%d) (may block on device init)\n",
		      logic_id);
	rc = aclrtSetDevice(logic_id);
	if (rc != 0) {
		SPDK_ERRLOG("aclrtSetDevice(%d) failed rc=%d\n", logic_id, rc);
		pthread_mutex_unlock(&g_hcomm.lock);
		return -1;
	}
	g_hcomm.acl_refcnt = 1;
	g_hcomm.acl_logic_dev_id = logic_id;
	pthread_mutex_unlock(&g_hcomm.lock);
	SPDK_DEBUGLOG(nvme_hcomm, "aclrtSetDevice(%d) ok\n", logic_id);
	return 0;
}

/*
 * Load the HixlBatchSend AICPU kernel once and cache its func handle.
 * Must be called after nvme_hcomm_acl_acquire_device() (which sets the ACL
 * device context).  Guarded by g_hcomm.lock.
 */
static int
nvme_hcomm_load_batch_send_kernel(void)
{
	const char *ascend_home;
	char json_path[512];
	aclrtBinaryLoadOption option = {};
	aclrtBinaryLoadOptions options = {};

	pthread_mutex_lock(&g_hcomm.lock);
	if (g_hcomm.batch_send_loaded) {
		pthread_mutex_unlock(&g_hcomm.lock);
		return 0;
	}

	ascend_home = getenv("ASCEND_HOME_PATH");
	snprintf(json_path, sizeof(json_path), "%s%s",
		 ascend_home != NULL ? ascend_home : "/usr/local/Ascend/cann",
		 NVME_HCOMM_HIXL_KERNEL_JSON_SUFFIX);

	option.type = ACL_RT_BINARY_LOAD_OPT_CPU_KERNEL_MODE;
	option.value.cpuKernelMode = 0;
	options.options = &option;
	options.numOpt = 1;

	if (aclrtBinaryLoadFromFile(json_path, &options, &g_hcomm.batch_send_bin) != ACL_SUCCESS) {
		SPDK_ERRLOG("aclrtBinaryLoadFromFile(%s) failed\n", json_path);
		pthread_mutex_unlock(&g_hcomm.lock);
		return -1;
	}
	if (aclrtBinaryGetFunction(g_hcomm.batch_send_bin, "HixlBatchSend",
				   &g_hcomm.batch_send_func) != ACL_SUCCESS) {
		SPDK_ERRLOG("aclrtBinaryGetFunction(HixlBatchSend) failed\n");
		pthread_mutex_unlock(&g_hcomm.lock);
		return -1;
	}

	g_hcomm.batch_send_loaded = true;
	pthread_mutex_unlock(&g_hcomm.lock);
	SPDK_DEBUGLOG(nvme_hcomm, "loaded HixlBatchSend kernel: bin=%p func=%p\n",
		      g_hcomm.batch_send_bin, g_hcomm.batch_send_func);
	return 0;
}

static void
nvme_hcomm_acl_release_device(void)
{
	pthread_mutex_lock(&g_hcomm.lock);
	/* Underflow guard: peer_online mode never acquires (ctrlr_construct
	 * only calls acquire for NETWORK_OFFLINE), so release is a no-op
	 * there — same defensive shape as nvme_hcomm_release_global_ctx. */
	if (g_hcomm.acl_refcnt == 0) {
		pthread_mutex_unlock(&g_hcomm.lock);
		return;
	}
	g_hcomm.acl_refcnt--;
	if (g_hcomm.acl_refcnt > 0) {
		pthread_mutex_unlock(&g_hcomm.lock);
		return;
	}

	/* Last reference: reset the device. */
	SPDK_DEBUGLOG(nvme_hcomm, "calling aclrtResetDevice(%d) (last ref, may block)\n",
		      g_hcomm.acl_logic_dev_id);
	aclrtResetDevice(g_hcomm.acl_logic_dev_id);
	pthread_mutex_unlock(&g_hcomm.lock);
	SPDK_DEBUGLOG(nvme_hcomm, "aclrtResetDevice done (last ref)\n");
}

/* ra.h (rdma_agent/inc, not includable by SPDK due to deep private deps))
 * defines RA_MAX_PHY_ID_NUM = 64; mirror it for the env bounds check. */
#ifndef RA_MAX_PHY_ID_NUM
#define RA_MAX_PHY_ID_NUM 64
#endif

static void
nvme_hcomm_init_network_mode(void)
{
	const char *env;
	char *end = NULL;
	unsigned long phy;

	if (g_hcomm.network_mode_inited) {
		return;
	}
	g_hcomm.network_mode_inited = true;

	env = getenv("SPDK_HCOMM_NETWORK_MODE");
	if (env != NULL) {
		if (strcmp(env, "peer_online") == 0) {
			g_hcomm.network_mode = NETWORK_PEER_ONLINE;
		} else if (strcmp(env, "offline") == 0) {
			g_hcomm.network_mode = NETWORK_OFFLINE;
		} else {
			SPDK_WARNLOG("Unknown SPDK_HCOMM_NETWORK_MODE='%s', using default offline\n",
				     env);
			g_hcomm.network_mode = NETWORK_OFFLINE;
		}
	}

	env = getenv("SPDK_HCOMM_PHY_ID");
	if (env != NULL && env[0] != '\0') {
		phy = strtoul(env, &end, 0);
		if (end != env && phy < RA_MAX_PHY_ID_NUM) {
			g_hcomm.phy_id = (unsigned int)phy;
		} else {
			SPDK_WARNLOG("Invalid SPDK_HCOMM_PHY_ID='%s', using default 0\n", env);
		}
	}

	SPDK_NOTICELOG("HCOMM network mode: %s (%d), phyId=%u\n",
		       (g_hcomm.network_mode == NETWORK_PEER_ONLINE) ? "peer_online" : "offline",
		       g_hcomm.network_mode, g_hcomm.phy_id);
}

static void
nvme_hcomm_socket_timeout_init(void)
{
	if (g_hcomm.socket_timeout_inited) {
		return;
	}
	g_hcomm.socket_timeout_inited = true;

	const char *env = getenv("SPDK_HCOMM_SOCKET_TIMEOUT_MS");
	if (env != NULL && env[0] != '\0') {
		char *end = NULL;
		unsigned long v = strtoul(env, &end, 0);
		if (end != env && v >= 1 && v <= 600000) {
			g_hcomm.socket_timeout_ms = (uint32_t)v;
			SPDK_NOTICELOG("HCOMM socket timeout = %u ms (from env)\n",
				       g_hcomm.socket_timeout_ms);
		} else {
			SPDK_WARNLOG("Invalid SPDK_HCOMM_SOCKET_TIMEOUT_MS='%s', using default %u\n",
				     env, g_hcomm.socket_timeout_ms);
		}
	}
}

static void
nvme_hcomm_channel_port_init(void)
{
	if (g_hcomm.channel_port_inited) {
		return;
	}
	g_hcomm.channel_port_inited = true;

	const char *env = getenv("SPDK_HCOMM_CHANNEL_PORT");
	if (env != NULL && env[0] != '\0') {
		char *end = NULL;
		unsigned long v = strtoul(env, &end, 0);
		if (end != env && v >= 1 && v <= 65535) {
			g_hcomm.channel_port = (uint16_t)v;
			SPDK_NOTICELOG("HCOMM channel port = %u (from env)\n",
				       g_hcomm.channel_port);
		} else {
			SPDK_WARNLOG("Invalid SPDK_HCOMM_CHANNEL_PORT='%s', using default %u\n",
				     env, g_hcomm.channel_port);
		}
	}
}

static void
nvme_hcomm_connect_watchdog_init(void)
{
	if (g_hcomm.connect_watchdog_inited) {
		return;
	}
	g_hcomm.connect_watchdog_inited = true;

	const char *env = getenv("SPDK_HCOMM_CONNECT_WATCHDOG_MS");
	if (env != NULL && env[0] != '\0') {
		char *end = NULL;
		unsigned long v = strtoul(env, &end, 0);
		if (end != env && v >= 1 && v <= 3600000) {
			g_hcomm.connect_watchdog_ms = (uint32_t)v;
			SPDK_NOTICELOG("HCOMM connect watchdog = %u ms (from env)\n",
				       g_hcomm.connect_watchdog_ms);
		} else {
			SPDK_WARNLOG("Invalid SPDK_HCOMM_CONNECT_WATCHDOG_MS='%s', using default %u\n",
				     env, g_hcomm.connect_watchdog_ms);
		}
	}
}

static void
nvme_hcomm_init_global_settings(void)
{
	/* Serialize the one-time env resolution.  ctrlr_construct calls this
	 * as its first statement and SPDK allows concurrent connects from
	 * multiple threads, so the unguarded check-then-set inside the init
	 * helpers would race (e.g. nvme_hcomm_acl_acquire_device reads
	 * g_hcomm.phy_id under this same mutex).  The cross-validation below also
	 * triggers nvme_hcomm_protocol()'s lazy env parse, so the one-time
	 * write to g_hcomm.protocol happens under the lock too; every later
	 * caller of nvme_hcomm_protocol() operates on a ctrlr that already
	 * passed through here, so it only reads the stable value. */
	pthread_mutex_lock(&g_hcomm.lock);

	nvme_hcomm_init_network_mode();
	nvme_hcomm_socket_timeout_init();
	nvme_hcomm_channel_port_init();
	nvme_hcomm_connect_watchdog_init();

	/* Cross-validate protocol vs network mode.  The two env vars are
	 * strongly coupled but independently configurable, which leads to
	 * silent misconfiguration:
	 *   ubc_tp + peer_online → CTP over RTP path, endpoint locType mismatch
	 *   uboe   + offline     → RTP over HDC path, no routable transport
	 * Warn loudly so the operator notices before the connect hangs. */
	if (nvme_hcomm_protocol() == COMM_PROTOCOL_UBOE &&
	    g_hcomm.network_mode != NETWORK_PEER_ONLINE) {
		SPDK_WARNLOG("SPDK_HCOMM_PROTOCOL=uboe but NETWORK_MODE is "
			     "offline — UBOE expects peer_online (RTP).  "
			     "Connection will likely fail.\n");
	}
	if (nvme_hcomm_protocol() == COMM_PROTOCOL_UBC_TP &&
	    g_hcomm.network_mode == NETWORK_PEER_ONLINE) {
		SPDK_WARNLOG("SPDK_HCOMM_PROTOCOL=ubc_tp but NETWORK_MODE is "
			     "peer_online — UBC_TP expects offline (CTP).  "
			     "Connection will likely fail.\n");
	}

	pthread_mutex_unlock(&g_hcomm.lock);
}

/* Per-mode UB transport profile helpers.
 * conn_mode wire value: 0=Rc / 1=Rm; under the nvmf_cm frame this is always 1
 * (CONN_RM) for both profiles — see nvme_hcomm_conn_mode() below. */
static inline uint8_t
nvme_hcomm_conn_mode(void)
{
	/* Both profiles use CONN_RM (Reliable Message): UBC/offline hardware
	 * rejects CONN_RC+CTP (umq_ub.c returns EINVAL) and UBoE/peer defaults
	 * to RM, so the wire conn_mode is always 1 (Rm).  Only the TP type
	 * (CTP vs RTP) varies by mode — see nvme_hcomm_tp_type().
	 * Reference: nids_aiv_ub EffectiveJettyRm()/NormalizeTpTypeForProfile(). */
	return 1u;
}

static inline enum TransportModeT
nvme_hcomm_trans_mode(void)
{
	/* Always CONN_RM — see nvme_hcomm_conn_mode() rationale. */
	return CONN_RM;
}

/* urma_tp_type_t: 0=RTP, 1=CTP.  UBoE/peer uses RTP; UBC/offline uses CTP
 * (link-layer-reliable, single-supernode). */
static inline uint32_t
nvme_hcomm_tp_type(void)
{
	return (g_hcomm.network_mode == NETWORK_PEER_ONLINE) ? 0u : 1u;
}

static inline bool
nvme_hcomm_use_rtp(void)
{
	return g_hcomm.network_mode == NETWORK_PEER_ONLINE;
}

/* ========== Per-controller (per-target) structure ==========
 *
 * Mirrors nvme_rdma_ctrlr: holds only process-level / per-target resources
 * shared by all qpairs of this controller.  Per-connection resources
 * (channel, PSN, etc.) live in nvme_hcomm_qpair.
 */
struct nvme_hcomm_ctrlr {
	struct spdk_nvme_ctrlr			ctrlr;
	EndpointHandle				endpoint;
	EndpointDesc				endpointDesc;
	bool					endpoint_created;

	/* Target address / port from transport_id */
	char					target_addr[SPDK_NVMF_TRADDR_MAX_LEN + 1];
	uint16_t				target_port;

	/* Local HccpEid (16B) — INI NIC identity, shared by all qpairs of
	 * this ctrlr (same NPU NIC → same EID).  Published in
	 * nvmf_cm_conn_req.conn_info.ub_info.eid. */
	uint8_t				local_eid[16];

	/* ---- Per-target socket resources (shared by all qpairs) ----
	 * One ctrlr = one socket.  The pre-created CLIENT socket (UBOE
	 * workaround) and the NOF side-channel fd are shared by all qpairs;
	 * each qpair's channel reuses them. */
	void					*socket_handle;  /* pre-created CLIENT socket (SocketCreate) */
	void					*nof_fd_handle;  /* NOF side-channel fdHandle (RaGetSockets) */

	/* Protects socket_handle / nof_fd_handle creation and NOF handshake
	 * I/O against concurrent IO qpair connect (each qpair runs its own
	 * worker thread — see nvme_hcomm_connect_worker).  The first qpair
	 * to connect creates the shared socket + NOF fd; subsequent qpairs
	 * find them already set.  The lock serialises the check-then-create
	 * race and the request-response NOF handshake on the shared fd. */
	pthread_mutex_t				socket_lock;

	/* ---- Dead fields (referenced only by quarantined #if 0 code) ----
	 * Retained for reference; only referenced by the DEAD CODE PRESERVATION
	 * AREA at the end of this file.  Wrapped in #if 0 so they do not
	 * participate in compilation. */
#if 0
	void					*fd_handle;		/* legacy HCCP fdHandle */
	void					*jfc_handle;
	void					*jetty_handle;
	void					*rem_jetty_handle;
	struct QpCreateInfo			local_qp_info;
	struct HccpTpInfo			tp_info;
	uint8_t				remote_conn_mode;
	bool					jfc_created;
	bool					jetty_created;
	bool					remote_imported;
#endif

	/* ---- Per-ctrlr memory registration cache (zero-copy data path) ----
	 * Shared by all qpairs of this ctrlr.  User payload buffers are
	 * registered with HcommMemReg on the endpoint; tokenIds are exchanged
	 * via exchangeAllMems and are valid across channels of this endpoint.
	 * Protected by its own internal mutex (qpairs may be on different
	 * reactor threads). */
	struct nvme_hcomm_mem_map		*mem_map;
};

/* ========== Per-queue (per-qpair) structure ==========
 *
 * Holds the INI-side shared-memory data plane (HOST <-> Device AI CPU):
 *   cqe_buf   — TGT writes NVMe CQEs at cqe_buf[cid]; HOST polls the
 *                valid flag (padding[0]) to complete.
 *   sq_buf    — cid-indexed array of standard 64B spdk_nvme_cmd entries
 *                (mirrors nvme_rdma's cmds[rdma_req->id]); HOST builds
 *                the SQE at sq_buf[cid] and sends it to the TGT via
 *                the bilateral SEND (HcommSendOnThread).
 *   req_for_cid[] — cid-indexed array tracking in-flight CMDs (set in
 *                submit_request, cleared in process_completions).
 *
 * These are PER-QPAIR resources (each qpair gets its own set), mirroring
 * nvme_rdma_qpair which carries its own cq/sq per qpair.
 */
/* Async connect lifecycle: worker-thread handshake + poll-path fabrics
 * CONNECT.  Phase transitions (writing thread in brackets):
 *
 *   IDLE        -> WORKING       [connect caller, before pthread_create]
 *   WORKING     -> LINK_UP       [worker: channel + NOF handshake ok]
 *   WORKING     -> FAILED        [worker: handshake failed; the worker has
 *                                 already unwound its resources]
 *   LINK_UP     -> FABRIC_SENT   [poll thread: fabrics CONNECT submitted]
 *   FABRIC_SENT -> IDLE          [poll thread: CONNECT reply ok, qpair set
 *                                 CONNECTED (cntlid stored by fabric layer)]
 *   any         -> IDLE          [poll thread / disconnect: failure unwind]
 *
 * The worker thread only runs the blocking hcomm-level setup (channel
 * create + NOF handshake) and never touches NVMe/qpair state - its
 * result is published through the release store of connect_state (which
 * also publishes channel_created / tgt_info).  All NVMe-layer traffic,
 * including the fabrics CONNECT submit and its CQE reap, happens on the
 * polling thread (rdma/tcp pattern: transports drive fabrics CONNECT
 * from their poll path, never from a side thread). */
enum nvme_hcomm_connect_state {
	NVME_HCOMM_CONN_IDLE,          /* no connect in flight / reset */
	NVME_HCOMM_CONN_WORKING,       /* worker running the blocking handshake */
	NVME_HCOMM_CONN_LINK_UP,       /* handshake ok; fabrics CONNECT not sent */
	NVME_HCOMM_CONN_FABRIC_SENT,   /* fabrics CONNECT submitted, awaiting CQE */
	NVME_HCOMM_CONN_FAILED,        /* terminal: handshake failed (see result) */
};

struct nvme_hcomm_qpair {
	struct spdk_nvme_qpair			qpair;
	struct nvme_hcomm_ctrlr			*hctrlr;    /* back-pointer to parent ctrlr */
	ThreadHandle				thread;
	bool					thread_allocated;

	/* ---- Per-qpair channel (HcommChannelCreate) ----
	 * One qpair = one channel (mirrors nvme_rdma_qpair's rdma_qp/cm_id).
	 * Each qpair creates and destroys its own channel; the ctrlr's
	 * shared socket_handle is reused via the channel desc. */
	ChannelHandle				channel;
	bool					channel_created;

	/* HixlBatchSend AICPU kernel launch state (DMA-visible so the AI CPU
	 * operator can read/write it).  Single batch, one SQE per send. */
	struct nvme_hcomm_send_io_batch	*batch;
	uint32_t			*send_status;

	/* INI's own tx PSN for this qpair's channel.  Generated per-qpair
	 * in nvme_hcomm_nof_handshake, published in req.ub_info.psn so the
	 * TGT aligns its rx PSN.  Per-qpair because each channel has its
	 * own PSN sequence. */
	uint32_t				local_tx_psn;

	/* ---- TGT identity (parsed from this qpair's NOF handshake resp) ----
	 * Per-qpair: each qpair's handshake negotiates its own TGT jetty
	 * identity, access token, active-TP path and rx PSN.  The same
	 * physical TGT may share a jetty across multiple qpairs (multi-
	 * channel scenario), but the negotiated metadata is recorded per
	 * qpair so each channel's import/dispatch path is self-contained.
	 * local_eid (INI NIC identity) stays on hctrlr — shared by all
	 * qpairs of this ctrlr.  Filled only in nvme_hcomm_nof_handshake. */
	struct tgt_conn_info			tgt_info;

	/* ---- CQE array: cid-indexed, cacheline-aligned, phys-contiguous ----
	 * TGT writes completions at cqe_buf[cid] (offset = cid × 64).
	 * HOST polls cqe_buf[cid] for each in-flight cid.
	 * No ring header — the array starts at cqe_buf[0] = cid 0. */
	struct nvme_hcomm_cqe_slot		*cqe_buf;      /* DMA memory, 64B-aligned */
	void					*cqe_phys;      /* physical addr (for TGT DMA) */
	uint32_t				cqe_depth;

	/* ---- SQ: cid-indexed array of SQE slots ----
	 * Each slot is hqpair->sqe_stride bytes: the base SQE (96) plus the
	 * tail SGL descriptor area (NVME_HCOMM_MAX_SGL * 16B) so any cid
	 * can host a max-size multi-SGL SQE.  Mirrors nvme_rdma's cmds[rdma_req->id]
	 * (its capsule is likewise 64B cmd + sgl[16]).  Allocating a cid ==
	 * allocating an SQE slot (unified, no separate ring / doorbell /
	 * full-check).  HOST builds the SQE at sq_buf[cid] and pushes it to
	 * the TGT via the bilateral SEND (HcommSendOnThread); the SEND
	 * length is variable (sqe_size + count*16), the slot stride is fixed. */
	void					*sq_buf;
	uint32_t				sq_depth;
	uint8_t					sqe_size;	/* base SQE size (96) */
	uint16_t				sqe_stride;	/* sqe_size + tail SGL area */

	/* cid → request tracking ----
	 * req_for_cid[cid] points to the in-flight nvme_request for that cid.
	 * Set in submit_request, cleared in process_completions.  NULL = free. */
	struct nvme_request			**req_for_cid;

	/* 空闲 cid 池 ----
	 * cid 由 transport 层自己分配（标准 SPDK 模式，同 rdma/tcp）：
	 * 值域 [0, cqe_depth)，保证 cid 直接索引 cqe_buf / sq_buf /
	 * req_for_cid 不越界，且同一时刻每个 in-flight 请求的
	 * cid 唯一。alloc cid 即分配一个 SQE 槽（与 RDMA 的 free_reqs 一致）。
	 * free_cids[cid] = 下一个空闲 cid（单链表栈）；free_cid_head 为栈顶；
	 * UINT32_MAX 表示空栈 / 栈底。alloc/free 均 O(1)。 */
	uint32_t				*free_cids;
	uint32_t				free_cid_head;

	/* in-flight cid 数组（swap-remove 顺序表）----
	 * process_completions 只遍历 active_cids[0..num_active)，轮询开销
	 * O(in-flight) 而非 O(cqe_depth)。提交时 append 尾部；完成时
	 * swap-remove（末元素换到被删位置）。
	 * 与 free_cids 池互补：active 记录"在用"，free 池记录"空闲"。
	 * num_active 恒 ≤ cqe_depth（cid 池大小），数组不会溢出。 */
	uint32_t				*active_cids;
	uint32_t				num_active;

	/* ---- Per-cid mem_map reference records ----
	 * After a successful submit, record the registration references
	 * this request holds (contiguous: one; SGL payload: one per
	 * segment, as a cache entry index or a transient handle).  They
	 * are released on completion / abort / delete, guaranteeing that
	 * an LRU eviction can never unregister a tokenId still referenced
	 * by an in-flight SQE. */
	struct nvme_hcomm_xlation_refs		*xlation_for_cid;

	/* ---- Per-qpair UB-registered memory regions ----
	 * SQ array + CQE array registered once at create_qpair so TGT/Device
	 * can access them via UB RDMA.  Per-request user payload buffers are
	 * registered on-the-fly via mem_map (zero-copy, mirror RDMA's mr_map). */
	void					*sq_lmem_handle;
	void					*cqe_lmem_handle;
	uint32_t				cqe_tokenid;    /* from HcommMemExport */

	bool					queues_created;  /* CQE/SQ allocated */

	/* ---- Async connect state (worker + poll-path fabrics CONNECT) ----
	 * connect_state is written by the worker thread and by the polling
	 * thread - all accesses go through the atomic helpers below (the
	 * worker's release store also publishes its resource writes).
	 * connect_result carries the worker's rc, read by the poll thread
	 * only after FAILED is observed.  The worker is joinable and reaped
	 * by delete_io_qpair if the qpair is torn down first. */
	enum nvme_hcomm_connect_state		connect_state;
	int					connect_result;
	bool					worker_started;  /* connect_worker needs join */
	pthread_t				connect_worker;
	bool					in_connect_poll; /* recursion guard, rdma/tcp */

	/* ---- connect DFX (DEBUG-level tracing) ----
	 * connect_start_ticks is written by ctrlr_connect_qpair before the
	 * worker spawn (the pthread_create edge orders it) and read by both
	 * threads afterwards; debug_last_phase is touched only by the poll
	 * thread - connect_poll runs at poll rate, so phase changes are
	 * logged, not every call. */
	uint64_t				connect_start_ticks;
	enum nvme_hcomm_connect_state		debug_last_phase;

	/* ---- connect watchdog ----
	 * connect_deadline is set by ctrlr_connect_qpair (poll thread) and
	 * only read by connect_poll; connect_abandoned marks a watchdog
	 * expiry with the worker still inside the library - the handshake
	 * resources then stay with the worker and the reconnect sweep
	 * (join first) releases them. */
	uint64_t				connect_deadline;
	bool					connect_abandoned;
};

static inline void
nvme_hcomm_conn_state_store(struct nvme_hcomm_qpair *hqpair,
			    enum nvme_hcomm_connect_state state)
{
	__atomic_store_n(&hqpair->connect_state, state, __ATOMIC_RELEASE);
}

static inline enum nvme_hcomm_connect_state
nvme_hcomm_conn_state_load(struct nvme_hcomm_qpair *hqpair)
{
	return __atomic_load_n(&hqpair->connect_state, __ATOMIC_ACQUIRE);
}

/* Elapsed whole milliseconds since connect_start_ticks (connect DFX). */
static inline uint32_t
nvme_hcomm_connect_elapsed_ms(const struct nvme_hcomm_qpair *hqpair)
{
	return (uint32_t)((spdk_get_ticks() - hqpair->connect_start_ticks) * 1000 /
			  spdk_get_ticks_hz());
}

/* ========== Poll group ========== */
struct nvme_hcomm_poll_group {
	struct spdk_nvme_transport_poll_group	group;
};

static inline struct nvme_hcomm_qpair *
nvme_hcomm_qpair(struct spdk_nvme_qpair *qpair)
{
	return SPDK_CONTAINEROF(qpair, struct nvme_hcomm_qpair, qpair);
}

static inline struct nvme_hcomm_ctrlr *
nvme_hcomm_ctrlr(struct spdk_nvme_ctrlr *ctrlr)
{
	assert(ctrlr->trid.trtype == SPDK_NVME_TRANSPORT_HCOMM);
	return SPDK_CONTAINEROF(ctrlr, struct nvme_hcomm_ctrlr, ctrlr);
}

static inline struct nvme_hcomm_poll_group *
nvme_hcomm_poll_group(struct spdk_nvme_transport_poll_group *group)
{
	return SPDK_CONTAINEROF(group, struct nvme_hcomm_poll_group, group);
}

/*
 * ========== Local IP detection via HCCP RaGetIfaddrs ==========
 *
 * Uses the HCCP libsocket API (RaGetIfaddrs) to enumerate local network
 * interfaces instead of POSIX socket/connect.  Selects the first
 * non-loopback interface matching the target's address family.
 */
static int
nvme_hcomm_get_local_socket_ip(const char *remote_addr, struct rdev *rdevInfo)
{
	struct RaGetIfattr ifattr;
	struct InterfaceInfo ifaddrs[16];
	unsigned int num = 6;
	int family;
	unsigned int i;
	int rc;

	/* Allow explicit override of local IP via env var.  Useful when
	 * RaGetIfaddrs fails (e.g. device HCCP rs doesn't support the
	 * GET_IFADDRS opcode version query, returning 128303 = -EINVAL)
	 * and the fallback (use remote_addr as local) would pick the wrong
	 * NIC.  Format: SPDK_HCOMM_LOCAL_IP=10.10.N.N */
	const char *local_ip_env = getenv("SPDK_HCOMM_LOCAL_IP");
	if (local_ip_env != NULL && local_ip_env[0] != '\0') {
		if (inet_pton(AF_INET, local_ip_env, &rdevInfo->localIp.addr) == 1) {
			rdevInfo->family = AF_INET;
			SPDK_NOTICELOG("Using SPDK_HCOMM_LOCAL_IP=%s as local IP (overrides RaGetIfaddrs)\n",
				       local_ip_env);
			return 0;
		} else if (inet_pton(AF_INET6, local_ip_env, &rdevInfo->localIp.addr6) == 1) {
			rdevInfo->family = AF_INET6;
			SPDK_NOTICELOG("Using SPDK_HCOMM_LOCAL_IP=%s as local IP (overrides RaGetIfaddrs)\n",
				       local_ip_env);
			return 0;
		}
		SPDK_WARNLOG("SPDK_HCOMM_LOCAL_IP='%s' not a valid IP, falling through to RaGetIfaddrs\n",
			     local_ip_env);
	}

	if (inet_pton(AF_INET, remote_addr, &rdevInfo->localIp.addr) == 1) {
		family = AF_INET;
	} else if (inet_pton(AF_INET6, remote_addr, &rdevInfo->localIp.addr6) == 1) {
		family = AF_INET6;
	} else {
		return -EINVAL;
	}
	rdevInfo->family = family;

	memset(&ifattr, 0, sizeof(ifattr));
	ifattr.phyId = g_hcomm.phy_id;
	ifattr.nicPosition = (unsigned int)g_hcomm.network_mode;
	ifattr.isAll = true;

	rc = RaGetIfaddrs(&ifattr, ifaddrs, &num);
	if (rc != 0) {
		SPDK_WARNLOG("RaGetIfaddrs failed: ret=%d, using remote addr as local\n", rc);
		/* Fallback: use remote addr as local (same subnet assumption) */
		return 1;
	}

	for (i = 0; i < num; i++) {
		if (ifaddrs[i].family != family) {
			continue;
		}
		/* Skip loopback */
		if (family == AF_INET) {
			struct in_addr loopback;
			inet_pton(AF_INET, "127.0.0.1", &loopback);
			if (memcmp(&ifaddrs[i].ifaddr.ip.addr, &loopback, sizeof(loopback)) == 0) {
				continue;
			}
		}
		/* Found a matching non-loopback interface */
		rdevInfo->family = ifaddrs[i].family;
		rdevInfo->localIp = ifaddrs[i].ifaddr.ip;
		SPDK_DEBUGLOG(nvme_hcomm, "Local interface %s selected for %s\n",
			      ifaddrs[i].ifname, remote_addr);
		return 0;
	}

	SPDK_WARNLOG("No matching interface found for %s family, using remote addr\n", remote_addr);
	return 1;
}

/*
 * ========== Global shared resource management ==========
 *
 * acquire_global_socket()  / release_global_socket()
 * acquire_global_ctx()     / release_global_ctx()
 *
 * HCCP service (aclrtSetDevice + TsdProcessOpen + RaInit) is brought up
 * internally by HcommEndpointCreate — no explicit bringup here.
 * Called from ctrlr_construct / ctrlr_destruct.
 */


static int
nvme_hcomm_acquire_global_socket(struct nvme_hcomm_ctrlr *hctrlr)
{
	struct rdev rdevInfo = {};
	int rc;

	pthread_mutex_lock(&g_hcomm.lock);
	if (g_hcomm.socket_refcnt > 0) {
		/* Already initialised - just bump refcount */
		g_hcomm.socket_refcnt++;
		SPDK_DEBUGLOG(nvme_hcomm, "Global socket already init, refcnt=%d\n",
			      g_hcomm.socket_refcnt);
		pthread_mutex_unlock(&g_hcomm.lock);
		return 0;
	}

	pthread_mutex_unlock(&g_hcomm.lock);

	rc = nvme_hcomm_get_local_socket_ip(hctrlr->target_addr, &rdevInfo);
	if (rc != 0) {
		/* Degraded detection: localIp keeps the REMOTE address as a
		 * placeholder.  The rs side binds it as the connect source
		 * (IPv6 always, IPv4 when not a vnic pseudo-address), so the
		 * connect then fails with rs_socket_bind_client failed - set
		 * SPDK_HCOMM_LOCAL_IP to the device-side address to recover. */
		SPDK_WARNLOG("HCCP local IP detection for %s degraded (rc=%d): "
			     "local ip falls back to the remote address; set "
			     "SPDK_HCOMM_LOCAL_IP if the connect fails to bind\n",
			     hctrlr->target_addr, rc);
	}

	rdevInfo.phyId = g_hcomm.phy_id;

	pthread_mutex_lock(&g_hcomm.lock);
	if (g_hcomm.socket_refcnt > 0) {
		/* Race: another thread init'd before us */
		g_hcomm.socket_refcnt++;
		pthread_mutex_unlock(&g_hcomm.lock);
		return 0;
	}

	/* HDC/offline + IPv6 is NOT rejected by the library: RaSocketInit(V1)
	 * runs the HDC vnic-table sync only for IPv4 vnic pseudo-addresses
	 * (family == AF_INET && s_addr < RA_VNIC_MAX, ra_host.c:264) and
	 * silently skips it for IPv6, returning a live handle; the rs socket
	 * layer accepts AF_INET6 (rs_socket.c:1781) and the device NIC's IPv6
	 * path carries the connection (A5-verified with the former reject
	 * removed: the TGT still receives the correctly tagged connects).
	 * Notice, not reject: environments without device-side IPv6
	 * reachability fail later as a connect timeout - this log is the
	 * breadcrumb. */
	if (g_hcomm.network_mode == NETWORK_OFFLINE && rdevInfo.family == AF_INET6) {
		SPDK_NOTICELOG("HDC/offline with IPv6 target %s: vnic-table sync "
			       "skipped by the library; relying on device NIC "
			       "IPv6 reachability\n", hctrlr->target_addr);
	}
	SPDK_DEBUGLOG(nvme_hcomm, "calling RaSocketInit (mode=%d phyId=%u family=%d)\n",
		      (int)g_hcomm.network_mode, g_hcomm.phy_id, rdevInfo.family);
	if (rdevInfo.family == AF_INET6) {
		struct SocketInitInfoT socketInit = {};
		socketInit.rdevInfo = rdevInfo;
		socketInit.scopeId = 0;   /* non-link-local; not configurable yet */
		rc = RaSocketInitV1(g_hcomm.network_mode, socketInit, &g_hcomm.socket_handle);
	} else {
		rc = RaSocketInit(g_hcomm.network_mode, rdevInfo, &g_hcomm.socket_handle);
	}
	if (rc != 0 || g_hcomm.socket_handle == NULL) {
		SPDK_ERRLOG("RaSocketInit (global) failed: ret=%d\n", rc);
		g_hcomm.socket_handle = NULL;
		pthread_mutex_unlock(&g_hcomm.lock);
		return -1;
	}
	g_hcomm.socket_refcnt = 1;
	pthread_mutex_unlock(&g_hcomm.lock);

	SPDK_DEBUGLOG(nvme_hcomm, "Global socket init success, handle=%p, refcnt=%d\n",
		      g_hcomm.socket_handle, g_hcomm.socket_refcnt);
	return 0;
}

static void
nvme_hcomm_release_global_socket(void)
{
	pthread_mutex_lock(&g_hcomm.lock);
	if (g_hcomm.socket_refcnt <= 0) {
		pthread_mutex_unlock(&g_hcomm.lock);
		return;
	}
	g_hcomm.socket_refcnt--;
	if (g_hcomm.socket_refcnt > 0) {
		SPDK_DEBUGLOG(nvme_hcomm, "Global socket refcnt dropped to %d, keeping handle\n",
			      g_hcomm.socket_refcnt);
		pthread_mutex_unlock(&g_hcomm.lock);
		return;
	}

	/* Last user - deinit */
	if (g_hcomm.socket_handle != NULL) {
		SPDK_DEBUGLOG(nvme_hcomm, "calling RaSocketDeinit (last ref, handle=%p)\n",
			      g_hcomm.socket_handle);
		int rc = RaSocketDeinit(g_hcomm.socket_handle);
		if (rc != 0) {
			SPDK_WARNLOG("RaSocketDeinit (global) failed: ret=%d\n", rc);
		}
		g_hcomm.socket_handle = NULL;
	}
	pthread_mutex_unlock(&g_hcomm.lock);
	SPDK_DEBUGLOG(nvme_hcomm, "Global socket deinit done\n");
}

static int
nvme_hcomm_acquire_global_ctx(void)
{
	struct CtxInitCfg cfg;
	struct CtxInitAttr attr;
	struct RaInfo eid_q;
	unsigned int eid_num = 0;
	struct HccpDevEidInfo *eid_list = NULL;
	int rc;

	pthread_mutex_lock(&g_hcomm.lock);
	if (g_hcomm.ctx_refcnt > 0) {
		g_hcomm.ctx_refcnt++;
		pthread_mutex_unlock(&g_hcomm.lock);
		return 0;
	}
	pthread_mutex_unlock(&g_hcomm.lock);

	memset(&cfg, 0, sizeof(cfg));
	cfg.mode = g_hcomm.network_mode;

	memset(&attr, 0, sizeof(attr));
	attr.phyId = g_hcomm.phy_id;

	/* Query the device's first EID and fill attr.ub.eid + eidIndex BEFORE
	 * RaCtxInit.  Without this, RaHdcCtxInit fails with ret=-22 (-EINVAL),
	 * decoded as rc=128003 ("hccp message process failed").  The device-side
	 * HCCP rs process rejects the CTX_INIT request when the localEid is
	 * all-zero.  Mirrors nids_aiv_ub ub_v2_resource.cpp:377-415 (A5 verified).
	 *
	 * peer_online mode also queries here, but eid_list may legitimately
	 * be empty (HOST has no device EID); in that case attr.ub.eid stays
	 * zero and RaCtxInit's PEER path may handle it differently.  OFFLINE
	 * (HDC) mode requires a real device EID.
	 *
	 * When the query fails or returns nothing, fall back to
	 * SPDK_HCOMM_LOCAL_IP parsed as an IPv6 address (same trick as the
	 * tail fallback in nvme_hcomm_detect_local_eid). */
	memset(&eid_q, 0, sizeof(eid_q));
	eid_q.mode = g_hcomm.network_mode;
	eid_q.phyId = g_hcomm.phy_id;
	SPDK_DEBUGLOG(nvme_hcomm, "querying device EID count (RaGetDevEidInfoNum, mode=%d phyId=%u)\n",
		      (int)g_hcomm.network_mode, g_hcomm.phy_id);
	rc = RaGetDevEidInfoNum(eid_q, &eid_num);
	if (rc != 0 || eid_num == 0) {
		const char *local_ip_env;
		struct in6_addr loc_addr;

		SPDK_ERRLOG("RaGetDevEidInfoNum failed: ret=%d eid_num=%u (phyId=%u)\n",
			    rc, eid_num, g_hcomm.phy_id);
		/* Fallback: the 16 IPv6 bytes land in union HccpEid.raw
		 * (network order) verbatim.  eidIndex has no env counterpart
		 * and stays 0. */
		local_ip_env = getenv("SPDK_HCOMM_LOCAL_IP");
		if (local_ip_env == NULL ||
		    inet_pton(AF_INET6, local_ip_env, &loc_addr) != 1) {
			SPDK_ERRLOG("No usable SPDK_HCOMM_LOCAL_IP fallback either "
				    "(env %s); RaCtxInit will fail with -EINVAL "
				    "without a device EID.\n",
				    local_ip_env == NULL ? "unset" : "not an IPv6 address");
			/* Fall through anyway — caller will see RaCtxInit's own error. */
		} else {
			memcpy(&attr.ub.eid, &loc_addr, sizeof(attr.ub.eid));
			SPDK_WARNLOG("Using SPDK_HCOMM_LOCAL_IP=%s as device EID "
				     "fallback (eidIndex stays 0)\n", local_ip_env);
		}
	} else {
		eid_list = calloc(eid_num, sizeof(*eid_list));
		if (eid_list == NULL) {
			SPDK_ERRLOG("calloc eid_list (%u entries) failed\n", eid_num);
			return -ENOMEM;
		}
		SPDK_DEBUGLOG(nvme_hcomm, "fetching %u device EID entries (RaGetDevEidInfoList)\n",
			      eid_num);
		rc = RaGetDevEidInfoList(eid_q, eid_list, &eid_num);
		if (rc != 0 || eid_num == 0) {
			SPDK_ERRLOG("RaGetDevEidInfoList failed: ret=%d got=%u (phyId=%u)\n",
				    rc, eid_num, g_hcomm.phy_id);
		} else {
			attr.ub.eidIndex = eid_list[0].eidIndex;
			memcpy(&attr.ub.eid, &eid_list[0].eid, sizeof(attr.ub.eid));
			SPDK_DEBUGLOG(nvme_hcomm, "Got device EID: eidIndex=%u "
				      "eid[0..3]=0x%02x%02x%02x%02x\n",
				      eid_list[0].eidIndex,
				      eid_list[0].eid.raw[0], eid_list[0].eid.raw[1],
				      eid_list[0].eid.raw[2], eid_list[0].eid.raw[3]);
		}
		free(eid_list);
	}

	pthread_mutex_lock(&g_hcomm.lock);
	if (g_hcomm.ctx_refcnt > 0) {
		g_hcomm.ctx_refcnt++;
		pthread_mutex_unlock(&g_hcomm.lock);
		return 0;
	}

	SPDK_DEBUGLOG(nvme_hcomm, "calling RaCtxInit (mode=%d phyId=%u eidIndex=%u)\n",
		      (int)cfg.mode, attr.phyId, attr.ub.eidIndex);
	rc = RaCtxInit(&cfg, &attr, &g_hcomm.ctx_handle);
	if (rc != 0 || g_hcomm.ctx_handle == NULL) {
		SPDK_ERRLOG("RaCtxInit (global) failed: ret=%d\n", rc);
		g_hcomm.ctx_handle = NULL;
		pthread_mutex_unlock(&g_hcomm.lock);
		return -1;
	}
	g_hcomm.ctx_refcnt = 1;

	/* Query device caps (sqMaxDepth / rqMaxDepth — the only fields we actually
	 * use for queue-depth clamping) — these are read from the cached
	 * ctxHandle->devAttr (a single memcpy inside RaGetDevBaseAttr), so this is
	 * cheap.  Other DevBaseAttr fields (sqMaxSge / rqMaxSge / maxJfsRsge /
	 * maxJfsInlineLen / maxWriteSize / maxMsgSize) exist in newer hcomm
	 * versions but are NOT referenced here to keep this code buildable
	 * against older hcomm releases that predate them.  Add back selectively
	 * when min-version bumps. */
	memset(&g_hcomm.dev_attr, 0, sizeof(g_hcomm.dev_attr));
	if (RaGetDevBaseAttr(g_hcomm.ctx_handle, &g_hcomm.dev_attr) == 0) {
		g_hcomm.dev_attr_inited = true;
		SPDK_NOTICELOG("HCOMM device caps: sqMaxDepth=%u rqMaxDepth=%u\n",
				g_hcomm.dev_attr.sqMaxDepth, g_hcomm.dev_attr.rqMaxDepth);
	} else {
		g_hcomm.dev_attr_inited = false;
		SPDK_WARNLOG("RaGetDevBaseAttr failed; queue depths will not be "
			     "clamped by device caps\n");
	}

	pthread_mutex_unlock(&g_hcomm.lock);

	SPDK_DEBUGLOG(nvme_hcomm, "Global UB context init success, handle=%p, refcnt=%d\n",
		      g_hcomm.ctx_handle, g_hcomm.ctx_refcnt);
	return 0;
}

static void
nvme_hcomm_release_global_ctx(void)
{
	pthread_mutex_lock(&g_hcomm.lock);
	if (g_hcomm.ctx_refcnt <= 0) {
		pthread_mutex_unlock(&g_hcomm.lock);
		return;
	}
	g_hcomm.ctx_refcnt--;
	if (g_hcomm.ctx_refcnt > 0) {
		pthread_mutex_unlock(&g_hcomm.lock);
		return;
	}

	if (g_hcomm.ctx_handle != NULL) {
		SPDK_DEBUGLOG(nvme_hcomm, "calling RaCtxDeinit (last ref, handle=%p)\n",
			      g_hcomm.ctx_handle);
		int rc = RaCtxDeinit(g_hcomm.ctx_handle);
		if (rc != 0) {
			SPDK_WARNLOG("RaCtxDeinit (global) failed: ret=%d\n", rc);
		}
		g_hcomm.ctx_handle = NULL;
	}
	/* Drop the cached device caps — the next acquire_global_ctx may run
	 * against a different device (different g_hcomm.phy_id / network mode), so
	 * the depths must be re-derived from a fresh RaGetDevBaseAttr query. */
	memset(&g_hcomm.dev_attr, 0, sizeof(g_hcomm.dev_attr));
	g_hcomm.dev_attr_inited = false;
	pthread_mutex_unlock(&g_hcomm.lock);
	SPDK_DEBUGLOG(nvme_hcomm, "Global UB context deinit done\n");
}

static int nvme_hcomm_build_local_endpoint_desc(struct nvme_hcomm_ctrlr *hctrlr);

static int
nvme_hcomm_acquire_global_endpoint(struct nvme_hcomm_ctrlr *hctrlr)
{
	int rc = 0;

	pthread_mutex_lock(&g_hcomm.endpoint_lock);
	if (g_hcomm.endpoint_refcnt == 0) {
		/* First caller builds the endpoint description.  The desc
		 * uses SPDK_HCOMM_LOCAL_IP (if set) or the first caller's
		 * target_addr to resolve the local EID; subsequent callers
		 * reuse this desc.  This works correctly as long as all
		 * targets are on subnets reachable from the same device NIC
		 * (the normal case). */
		SPDK_DEBUGLOG(nvme_hcomm, "building local endpoint desc (first ctrlr, target=%s)\n",
			      hctrlr->target_addr);
		if (nvme_hcomm_build_local_endpoint_desc(hctrlr) != 0) {
			rc = -1;
			goto out;
		}
		memcpy(&g_hcomm.endpoint_desc, &hctrlr->endpointDesc,
		       sizeof(g_hcomm.endpoint_desc));
		SPDK_DEBUGLOG(nvme_hcomm, "calling HcommEndpointCreate (first ctrlr; "
			      "may spawn hccp_service on the device, can take seconds)\n");
		HcommResult hret = HcommEndpointCreate(&g_hcomm.endpoint_desc, &g_hcomm.endpoint);
		if (hret != 0) {
			SPDK_ERRLOG("HcommEndpointCreate failed: ret=0x%x\n", hret);
			rc = -1;
			goto out;
		}
	}
	/* Copy the global desc into this ctrlr so downstream code
	 * (channel-create, socket-build) can still reference
	 * hctrlr->endpointDesc as before. */
	memcpy(&hctrlr->endpointDesc, &g_hcomm.endpoint_desc, sizeof(hctrlr->endpointDesc));
	hctrlr->endpoint = g_hcomm.endpoint;
	hctrlr->endpoint_created = true;
	g_hcomm.endpoint_refcnt++;
	SPDK_DEBUGLOG(nvme_hcomm, "acquire_global_endpoint: refcnt=%d\n", g_hcomm.endpoint_refcnt);

out:
	pthread_mutex_unlock(&g_hcomm.endpoint_lock);
	return rc;
}

static void
nvme_hcomm_release_global_endpoint(struct nvme_hcomm_ctrlr *hctrlr)
{
	pthread_mutex_lock(&g_hcomm.endpoint_lock);
	assert(g_hcomm.endpoint_refcnt > 0);
	if (--g_hcomm.endpoint_refcnt == 0) {
		SPDK_DEBUGLOG(nvme_hcomm, "calling HcommEndpointDestroy (last ref, handle=%p)\n",
			      g_hcomm.endpoint);
		HcommEndpointDestroy(g_hcomm.endpoint);
		g_hcomm.endpoint = NULL;
		SPDK_DEBUGLOG(nvme_hcomm, "release_global_endpoint: endpoint destroyed\n");
	}
	hctrlr->endpoint_created = false;
	hctrlr->endpoint = NULL;
	pthread_mutex_unlock(&g_hcomm.endpoint_lock);
}

/*
 * ========== Per-target socket connection ==========
 *
 * Shared by the UBC_TP control-plane path and the NOF side-channel.
 * Both use the global g_hcomm.socket_handle (ref-counted RaSocketInit) and
 * RaSocketBatchConnect + RaGetSockets(1=CLIENT) to create an explicit
 * CLIENT socket to the TGT — role is never determined by IP comparison.
 *
 * RaSocketBatchConnect is ASYNC: it initiates the connection and may
 * return SOCK_EAGAIN (128201), which means "initiated, not yet
 * established".  After calling it, RaGetSockets must be POLLED in a
 * loop until status == NVME_HCOMM_SOCKET_CONNECTED, with a timeout
 * (default 5s, mirroring nids_aiv_ub ub_oob_rasocket.cpp).  Calling
 * RaGetSockets only once is a bug — the HCCP state machine may still
 * be completing the whitelist handshake (256B tag → a5a5a).
 */

/* Remove a whitelist entry previously added by nvme_hcomm_ra_socket_connect.
 * Builds the same tag ("%s_%s_%u" of prefix/addr/port) so the entry is
 * matched exactly.  Used both on the normal disconnect path and on
 * connect failure paths (a failed attempt must not leave a stale entry
 * in the rs subprocess). */
static void
nvme_hcomm_whitelist_del(void *socketHandle, const char *target_addr,
			 uint32_t port, const char *tag_prefix)
{
	/* Partial fill below (connLimit/remoteIp/tag only) — the "= {}"
	 * zeroes the struct's remaining members.  tag is snprintf'd before
	 * its first read. */
	struct SocketWlistInfoT wlist = {};
	char tag[SOCK_CONN_TAG_SIZE];
	int rc;

	snprintf(tag, sizeof(tag), "%s_%s_%u", tag_prefix, target_addr, port);
	tag[sizeof(tag) - 1] = '\0';

	wlist.connLimit = 1;
	if (inet_pton(AF_INET, target_addr, &wlist.remoteIp.addr) == 1) {
		/* IPv4 */
	} else if (inet_pton(AF_INET6, target_addr, &wlist.remoteIp.addr6) == 1) {
		/* IPv6 */
	} else {
		return;
	}
	snprintf(wlist.tag, sizeof(wlist.tag), "%s", tag);
	rc = RaSocketWhiteListDel(socketHandle, &wlist, 1);
	if (rc != 0) {
		SPDK_WARNLOG("RaSocketWhiteListDel (tag=%s) failed: ret=%d (non-fatal)\n",
			     tag, rc);
	}
}

/* Shared connect helper: RaSocketBatchConnect (async) + poll RaGetSockets.
 * Caller owns the returned fdHandle.  tag_prefix distinguishes the
 * UBC_TP control socket ("SPDK_HCOMM") from the NOF side-channel ("NOF"). */
static int
nvme_hcomm_ra_socket_connect(void *socketHandle, const char *target_addr,
			     uint32_t port, const char *tag_prefix,
			     void **out_fd_handle)
{
	/* All four are zeroed / fully overwritten before first use below
	 * (memset for the structs — sockInfo re-zeroed every poll
	 * iteration — and snprintf for the tag), so plain declarations
	 * suffice; initialisers here would be redundant. */
	struct SocketConnectInfoT connInfo;
	struct SocketInfoT sockInfo;
	struct SocketWlistInfoT wlist;
	char tag[SOCK_CONN_TAG_SIZE];
	uint32_t elapsed_ms = 0;
	unsigned int connected = 0;
	int rc;

	if (socketHandle == NULL || target_addr == NULL || port == 0) {
		SPDK_ERRLOG("input variable not initialised\n");
		return -1;
	}

	snprintf(tag, sizeof(tag), "%s_%s_%u", tag_prefix, target_addr, port);
	tag[sizeof(tag) - 1] = '\0';

	/* Add whitelist entry so the rs subprocess accepts the connection
	 * from this IP and responds with the "a5a5a" valid flag. */
	memset(&wlist, 0, sizeof(wlist));
	wlist.connLimit = 1;
	if (inet_pton(AF_INET, target_addr, &wlist.remoteIp.addr) == 1) {
		/* IPv4 */
	} else if (inet_pton(AF_INET6, target_addr, &wlist.remoteIp.addr6) == 1) {
		/* IPv6 */
	} else {
		SPDK_ERRLOG("Cannot parse target addr '%s'\n", target_addr);
		return -1;
	}
	snprintf(wlist.tag, sizeof(wlist.tag), "%s", tag);
	SPDK_DEBUGLOG(nvme_hcomm, "adding socket whitelist entry (tag=%s, target=%s:%u)\n",
		      tag, target_addr, port);
	rc = RaSocketWhiteListAdd(socketHandle, &wlist, 1);
	if (rc != 0) {
		SPDK_WARNLOG("RaSocketWhiteListAdd failed: ret=%d (non-fatal)\n", rc);
	}

	memset(&connInfo, 0, sizeof(connInfo));
	connInfo.socketHandle = socketHandle;
	if (inet_pton(AF_INET, target_addr, &connInfo.remoteIp.addr) == 1) {
		/* IPv4 */
	} else if (inet_pton(AF_INET6, target_addr, &connInfo.remoteIp.addr6) == 1) {
		/* IPv6 */
	} else {
		SPDK_ERRLOG("Cannot parse target addr '%s'\n", target_addr);
		nvme_hcomm_whitelist_del(socketHandle, target_addr, port, tag_prefix);
		return -1;
	}
	connInfo.port = port;
	snprintf(tag, sizeof(tag), "%s_%s_%u", tag_prefix, target_addr, port);
	tag[sizeof(tag) - 1] = '\0';
	snprintf(connInfo.tag, sizeof(connInfo.tag), "%s", tag);

	/* Step 1: RaSocketBatchConnect — ASYNC initiator.
	 * Returns 0 or SOCK_EAGAIN on success (connection initiated);
	 * any other non-zero is a real error (don't retry). */
	rc = RaSocketBatchConnect(&connInfo, 1);
	if (rc != 0 && rc != SOCK_EAGAIN) {
		SPDK_ERRLOG("RaSocketBatchConnect to %s:%u failed: ret=%d\n",
			    target_addr, port, rc);
		nvme_hcomm_whitelist_del(socketHandle, target_addr, port, tag_prefix);
		return -1;
	}
	SPDK_DEBUGLOG(nvme_hcomm, "RaSocketBatchConnect issued to %s:%u (tag=%s), "
		      "polling RaGetSockets...\n", target_addr, port, tag);

	/* Step 2: Poll RaGetSockets until CONNECTED, HCCP timeout, or safety
	 * net.  SOCK_EAGAIN means "not ready yet" (retry); other non-zero is a
	 * real error.  status==2 (HCCP_SOCKET_TIMEOUT) means the HCCP state
	 * machine gave up internally — stop immediately, no point waiting for
	 * the safety-net timeout. */
	while (elapsed_ms < g_hcomm.socket_timeout_ms) {
		connected = 0;

		memset(&sockInfo, 0, sizeof(sockInfo));
		sockInfo.socketHandle = socketHandle;
		sockInfo.remoteIp = connInfo.remoteIp;
		snprintf(sockInfo.tag, sizeof(sockInfo.tag), "%s", tag);

		rc = RaGetSockets(1 /*CLIENT*/, &sockInfo, 1, &connected);
		if (rc == 0 && connected == 1 && sockInfo.fdHandle != NULL &&
		    sockInfo.status == NVME_HCOMM_SOCKET_CONNECTED) {
			*out_fd_handle = sockInfo.fdHandle;
			SPDK_DEBUGLOG(nvme_hcomm, "Socket connected to %s:%u (tag=%s), "
				      "fdHandle=%p (after %ums)\n",
				      target_addr, port, tag, *out_fd_handle, elapsed_ms);
			return 0;
		}
		if (rc != 0 && rc != SOCK_EAGAIN) {
			SPDK_ERRLOG("RaGetSockets to %s:%u failed: rc=%d "
				    "(connected=%u status=%d)\n",
				    target_addr, port, rc, connected, sockInfo.status);
			nvme_hcomm_whitelist_del(socketHandle, target_addr, port, tag_prefix);
			return -1;
		}
		/* HCCP internal timeout — stop polling immediately. */
		if (sockInfo.status == NVME_HCOMM_SOCKET_TIMEOUT) {
			SPDK_ERRLOG("RaSocket connect to %s:%u failed: HCCP timeout "
				    "(after %ums)\n", target_addr, port, elapsed_ms);
			nvme_hcomm_whitelist_del(socketHandle, target_addr, port, tag_prefix);
			return -1;
		}
		/* status == 3 (connecting) or SOCK_EAGAIN → keep polling.
		 * Progress breadcrumb once per second so a stuck HCCP state
		 * machine is visible from the last log line. */
		if (elapsed_ms > 0 && (elapsed_ms % 1000) == 0) {
			SPDK_DEBUGLOG(nvme_hcomm, "RaSocket connect to %s:%u still pending: "
				      "elapsed=%ums status=%d\n",
				      target_addr, port, elapsed_ms, sockInfo.status);
		}
		usleep(NVME_HCOMM_POLL_INTERVAL_US);
		elapsed_ms++;
	}

	SPDK_ERRLOG("RaSocket connect to %s:%u safety-net timeout after %ums "
		    "(last status=%d connected=%u)\n",
		    target_addr, port, g_hcomm.socket_timeout_ms,
		    sockInfo.status, connected);
	nvme_hcomm_whitelist_del(socketHandle, target_addr, port, tag_prefix);
	return -1;
}

/* Shared disconnect helper: RaSocketBatchClose. */
static void
nvme_hcomm_ra_socket_disconnect(void *socketHandle, void *fdHandle)
{
	if (fdHandle != NULL) {
		struct SocketCloseInfoT closeInfo = {};
		closeInfo.socketHandle = socketHandle;
		closeInfo.fdHandle = fdHandle;
		int rc = RaSocketBatchClose(&closeInfo, 1);
		if (rc != 0) {
			SPDK_WARNLOG("RaSocketBatchClose fdHandle=%p failed: ret=%d\n", fdHandle, rc);
		}
	}
}


/*
 * ========== NOF side-channel socket ==========
 *
 * Independent CLIENT socket to the TGT for NOF protocol exchange and
 * other data, running alongside the UBOE channel's own socket.
 *
 * Reuses g_hcomm.socket_handle (ref-counted RaSocketInit) — the UBOE channel
 * uses the C++ SocketMgr path, not g_hcomm.socket_handle, so there is no
 * conflict.  The NOF connect calls nvme_hcomm_acquire_global_socket to
 * bump the refcount (init-on-first); the NOF disconnect / ctrlr_destruct
 * calls nvme_hcomm_release_global_socket to balance it.
 *
 * role=1 (CLIENT) is passed explicitly to RaGetSockets via the shared
 * helper, so this side-channel always connects out regardless of IP
 * ordering — not affected by the UBOE channel's IP-comparison bug.
 */
static int
nvme_hcomm_nof_socket_connect(struct spdk_nvme_ctrlr *ctrlr, uint16_t port)
{
	struct nvme_hcomm_ctrlr *hctrlr = nvme_hcomm_ctrlr(ctrlr);

	if (hctrlr->nof_fd_handle != NULL) {
		SPDK_WARNLOG("NOF socket already connected for %s:%u\n",
			     hctrlr->target_addr, port);
		return 0;
	}
	if (port == 0) {
		SPDK_ERRLOG("NOF socket port cannot be 0\n");
		return -1;
	}
	if (!hctrlr->endpoint_created) {
		SPDK_ERRLOG("Endpoint not created, cannot open NOF socket\n");
		return -1;
	}

	/* Acquire g_hcomm.socket_handle (ref-counted RaSocketInit).  Under UBOE
	 * this is the first acquire; under UBC_TP it bumps the existing
	 * refcount from ctrlr_construct.  The held refcount is also what makes
	 * the unlocked g_hcomm.socket_handle reads below safe: it pins the
	 * handle (the last release that would NULL it cannot run) and its
	 * lock/unlock orders this read after the init write. */
	if (nvme_hcomm_acquire_global_socket(hctrlr) != 0) {
		return -1;
	}

	if (nvme_hcomm_ra_socket_connect(g_hcomm.socket_handle, hctrlr->target_addr,
					 port, "NOF_HCOMM",
					 &hctrlr->nof_fd_handle) != 0) {
		nvme_hcomm_release_global_socket();
		return -1;
	}

	SPDK_NOTICELOG("NOF socket connected to %s:%u, fdHandle=%p\n",
		       hctrlr->target_addr, port, hctrlr->nof_fd_handle);
	return 0;
}

static void
nvme_hcomm_nof_socket_disconnect(struct spdk_nvme_ctrlr *ctrlr)
{
	struct nvme_hcomm_ctrlr *hctrlr = nvme_hcomm_ctrlr(ctrlr);

	if (hctrlr->nof_fd_handle != NULL) {
		/* A non-NULL nof_fd_handle implies the socket refcount our
		 * connect acquired is still held (it is released only below),
		 * so g_hcomm.socket_handle is pinned and the reads below are
		 * safe without the lock. */
		/* Remove the whitelist entry added during connect, so the
		 * rs subprocess does not accumulate stale entries across
		 * reconnect cycles. */
		nvme_hcomm_whitelist_del(g_hcomm.socket_handle, hctrlr->target_addr,
					 hctrlr->target_port, "NOF_HCOMM");

		nvme_hcomm_ra_socket_disconnect(g_hcomm.socket_handle, hctrlr->nof_fd_handle);
		hctrlr->nof_fd_handle = NULL;
		/* Balance the acquire from connect — only release if we
		 * actually held a socket, otherwise we'd wrongly decrement
		 * a refcount owned by another ctrlr. */
		nvme_hcomm_release_global_socket();
		SPDK_NOTICELOG("NOF socket disconnected\n");
	}
}

/*
 * ========== Socket data exchange via HCCP RaSocketSend / RaSocketRecv ==========
 */
static int
nvme_hcomm_socket_send(const void *fdHandle, const void *data, size_t len)
{
	unsigned long long sentSize = 0;
	unsigned long long totalSent = 0;
	size_t remaining = len;
	bool eagainLogged = false;
	uint64_t deadline;
	int rc;

	if (len == 0) {
		return 0;
	}

	SPDK_DEBUGLOG(nvme_hcomm, "sending %zu bytes on NOF socket (fdHandle=%p)\n",
		      len, fdHandle);

	deadline = spdk_get_ticks() + g_hcomm.socket_timeout_ms *
		   (spdk_get_ticks_hz() / 1000);

	while (remaining > 0) {
		sentSize = 0;
		rc = RaSocketSend(fdHandle, (const char *)data + totalSent,
				  remaining, &sentSize);
		if (rc == SOCK_EAGAIN) {
			if (!eagainLogged) {
				/* One-shot breadcrumb: the socket is busy and we
				 * are now inside the bounded retry loop. */
				eagainLogged = true;
				SPDK_DEBUGLOG(nvme_hcomm, "RaSocketSend: socket busy (EAGAIN), "
					      "retrying with %ums deadline (remaining=%zu)\n",
					      g_hcomm.socket_timeout_ms, remaining);
			}
			if (spdk_get_ticks() >= deadline) {
				SPDK_ERRLOG("RaSocketSend timeout (%ums), remaining=%zu\n",
					    g_hcomm.socket_timeout_ms, remaining);
				return -1;
			}
			usleep(NVME_HCOMM_POLL_INTERVAL_US);
			continue;
		}
		if (rc == SOCK_ESOCKCLOSED) {
			SPDK_ERRLOG("RaSocketSend: connection closed by peer\n");
			return -1;
		}
		if (rc != 0) {
			SPDK_ERRLOG("RaSocketSend failed: ret=%d, remaining=%zu\n",
				    rc, remaining);
			return -1;
		}
		if (sentSize == 0) {
			SPDK_ERRLOG("RaSocketSend sent 0 bytes, connection may be closed\n");
			return -1;
		}
		if (sentSize > remaining) {
			SPDK_ERRLOG("RaSocketSend sent %llu > remaining %zu\n",
				    sentSize, remaining);
			return -1;
		}
		totalSent += sentSize;
		remaining -= (size_t)sentSize;
	}
	return 0;
}

static int
nvme_hcomm_socket_recv(const void *fdHandle, void *data, size_t len)
{
	unsigned long long recvSize = 0;
	unsigned long long totalRecv = 0;
	size_t remaining = len;
	bool eagainLogged = false;
	uint64_t deadline;
	int rc;

	if (len == 0) {
		return 0;
	}

	SPDK_DEBUGLOG(nvme_hcomm, "receiving %zu bytes on NOF socket (fdHandle=%p, "
		      "blocking poll up to %ums)\n", len, fdHandle,
		      g_hcomm.socket_timeout_ms);

	deadline = spdk_get_ticks() + g_hcomm.socket_timeout_ms *
		   (spdk_get_ticks_hz() / 1000);

	while (remaining > 0) {
		recvSize = 0;
		rc = RaSocketRecv(fdHandle, (char *)data + totalRecv,
				  remaining, &recvSize);
		if (rc == SOCK_EAGAIN) {
			if (!eagainLogged) {
				/* One-shot breadcrumb: no data yet, we are
				 * inside the bounded retry loop waiting for
				 * the TGT's frame. */
				eagainLogged = true;
				SPDK_DEBUGLOG(nvme_hcomm, "RaSocketRecv: no data yet (EAGAIN), "
					      "retrying with %ums deadline (remaining=%zu)\n",
					      g_hcomm.socket_timeout_ms, remaining);
			}
			if (spdk_get_ticks() >= deadline) {
				SPDK_ERRLOG("RaSocketRecv timeout (%ums), remaining=%zu\n",
					    g_hcomm.socket_timeout_ms, remaining);
				return -1;
			}
			usleep(NVME_HCOMM_POLL_INTERVAL_US);
			continue;
		}
		if (rc == SOCK_ESOCKCLOSED) {
			SPDK_ERRLOG("RaSocketRecv: connection closed by peer\n");
			return -1;
		}
		if (rc != 0) {
			SPDK_ERRLOG("RaSocketRecv failed: ret=%d, remaining=%zu\n",
				    rc, remaining);
			return -1;
		}
		if (recvSize == 0) {
			SPDK_ERRLOG("RaSocketRecv got 0 bytes, connection may be closed\n");
			return -1;
		}
		if (recvSize > remaining) {
			SPDK_ERRLOG("RaSocketRecv got %llu > remaining %zu\n",
				    recvSize, remaining);
			return -1;
		}
		totalRecv += recvSize;
		remaining -= (size_t)recvSize;
	}
	return 0;
}

/*
 * ========== NOF handshake (per-qpair exchange after socket connect) ==========
 *
 * Called once per qpair (admin + each IO).  Exchanges per-qpair info:
 *   INI → TGT: ini_cqe_addr (CQE ring phys addr), ini_cqe_tokenid,
 *             sq_size, qid, data_slot_size
 *   TGT → INI: ctrl_id, jetty identity
 *
 * The NOF socket is ctrlr-level (connected once); the handshake runs
 * per-qpair on the same socket.  Uses blocking send/recv.
 */
static int
nvme_hcomm_nof_handshake(struct nvme_hcomm_qpair *hqpair)
{
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	struct nvmf_cm_conn_req req;
	struct nvmf_cm_conn_resp resp;
	int rc;

	/* Build INI → TGT request. */
	memset(&req, 0, sizeof(req));
	req.hdr.version   = NVME_HCOMM_CM_VERSION_V3;
	req.hdr.opcode    = NVME_HCOMM_CM_OP_REQ;
	req.hdr.trans_type = NVME_HCOMM_CM_TRANS_HCOMM;
	req.hdr.status    = NVME_HCOMM_CM_STATUS_OK;
	req.hdr.msg_len   = sizeof(req);

	req.conn_info.ctrl_id  = hctrlr->ctrlr.cntlid;
	req.conn_info.qid      = hqpair->qpair.id;
	req.conn_info.sq_size  = hqpair->sq_depth;
	req.conn_info.rq_size  = hqpair->cqe_depth;
	req.conn_info.data_slot_size = 0;  /* unused (zero-copy, no bounce buffer) */

	/* INI per-qpair CQE ring address + tokenId for TGT to RDMA-write completions. */
	req.conn_info.ini_cqe_addr     = (uint64_t)(uintptr_t)hqpair->cqe_phys;
	req.conn_info.ini_cqe_tokenid  = hqpair->cqe_tokenid;

	/* INI jetty identity.  PSN is per-qpair (each channel has its own
	 * PSN sequence).  local_eid is per-ctrlr (shared NIC identity).
	 * uasid/jetty_id come from HcommChannelCreate's internal QpCreate —
	 * we don't have direct access, so they stay 0 (the TGT resolves
	 * identity via the channel's internal state, not these fields). */
	/* Randomise the initial tx PSN (RDMA-style hygiene).  rand() is not
	 * thread-safe across concurrent connect workers — seed a local
	 * rand_r from the tick counter instead. */
	{
		unsigned int seed = (unsigned int)spdk_get_ticks();

		hqpair->local_tx_psn = rand_r(&seed);
	}
	req.conn_info.ub_info.tp_handle   = 0;
	req.conn_info.ub_info.psn          = hqpair->local_tx_psn;
	req.conn_info.ub_info.jetty_type   = 1;
	req.conn_info.ub_info.is_shared     = 0;
	memcpy(req.conn_info.ub_info.eid, hctrlr->local_eid, 16);
	req.conn_info.ub_info.uasid        = 0;
	req.conn_info.ub_info.jetty_id      = 0;
	req.conn_info.ub_info.jetty_token  = NVME_HCOMM_INI_JFR_TOKEN;

	SPDK_NOTICELOG("NOF handshake qid=%u: cqe_addr=0x%" PRIx64 " cqe_tok=%" PRIu64 "\n",
		       hqpair->qpair.id, req.conn_info.ini_cqe_addr,
		       req.conn_info.ini_cqe_tokenid);

	/* INI sends first (CLIENT) — blocking.
	 *
	 * The nof_fd_handle is shared by all qpairs (ctrlr-level resource).
	 * The socket_lock serialises the send+recv pair so concurrent qpair
	 * handshakes don't interleave request/response frames on the shared
	 * side-channel socket. */
	pthread_mutex_lock(&hctrlr->socket_lock);
	rc = nvme_hcomm_socket_send(hctrlr->nof_fd_handle, &req, sizeof(req));
	if (rc != 0) {
		pthread_mutex_unlock(&hctrlr->socket_lock);
		SPDK_ERRLOG("NOF handshake send failed\n");
		return -1;
	}
	SPDK_DEBUGLOG(nvme_hcomm, "NOF handshake qid=%u: request sent (%zu B), "
		      "waiting for TGT response\n", hqpair->qpair.id, sizeof(req));

	/* Receive TGT reply — blocking. */
	memset(&resp, 0, sizeof(resp));
	rc = nvme_hcomm_socket_recv(hctrlr->nof_fd_handle, &resp, sizeof(resp));
	pthread_mutex_unlock(&hctrlr->socket_lock);
	if (rc != 0) {
		SPDK_ERRLOG("NOF handshake recv failed\n");
		return -1;
	}

	if (resp.hdr.opcode != NVME_HCOMM_CM_OP_RESP) {
		SPDK_ERRLOG("NOF handshake bad opcode: %u\n", resp.hdr.opcode);
		return -1;
	}
	if (resp.hdr.status != NVME_HCOMM_CM_STATUS_OK) {
		SPDK_ERRLOG("NOF handshake rejected: status=%u\n", resp.hdr.status);
		return -1;
	}

	/* Store TGT info.
	 *
	 * tgt_info is per-qpair: each channel's handshake negotiates its
	 * own TGT jetty identity / token / TP path / rx PSN.  Writing it
	 * to hctrlr would let later qpairs clobber earlier ones (and
	 * break the multi-qpair-shares-one-jetty scenario), so the live
	 * path always uses hqpair->tgt_info.
	 *
	 * cntlid is controller-wide (NVMe spec: assigned by admin
	 * qpair's connect).  Only the admin qpair (qid == 0) writes
	 * hctrlr->ctrlr.cntlid; IO qpairs receive the same value in
	 * their resp but MUST NOT overwrite the controller-wide field
	 * (it could race with admin-qpair-initiated updates and confuse
	 * SPDK state checks elsewhere that read hctrlr->ctrlr.cntlid). */
	if (hqpair->qpair.id == 0) {
		hctrlr->ctrlr.cntlid = resp.conn_info.ctrl_id;
	}
	hqpair->tgt_info.uasid     = resp.conn_info.ub_info.uasid;
	hqpair->tgt_info.jetty_id  = resp.conn_info.ub_info.jetty_id;
	hqpair->tgt_info.tp_handle  = resp.conn_info.ub_info.tp_handle;
	hqpair->tgt_info.psn        = resp.conn_info.ub_info.psn;
	hqpair->tgt_info.jetty_token = resp.conn_info.ub_info.jetty_token;
	memcpy(hqpair->tgt_info.eid, resp.conn_info.ub_info.eid, 16);

	SPDK_NOTICELOG("NOF handshake qid=%u success: ctrl_id=%u\n",
		       hqpair->qpair.id, resp.conn_info.ctrl_id);
	return 0;
}

/*
 * Effective per-qpair NVMe sq_size / rq_size — used in three places:
 *   - hqpair->cqe_depth   (Device→HOST CQE array, per-qpair)
 *   - hqpair->sq_depth  (HOST→Device SQ array, per-qpair)
 *   - nvmf_cm_conn_req.conn_info.sq_size / rq_size  (told to the TGT)
 *
 * The input `qpair_sq_size` is the NVMe queue depth (qsize) requested for
 * this qpair (opts.admin_queue_size for qid=0, opts.io_queue_size for
 * qid>0), NOT the software free_req pool size.  It is
 * clamped by the device's sqMaxDepth (the per-Jetty UB SQ cap, cached in
 * g_hcomm.dev_attr) so the INI never tells the TGT it can sustain more in-flight
 * CMDs than the underlying Jetty sqDepth allows — otherwise the TGT would
 * over-allocate recv buffers that the INI can never fill.
 *
 * The value is returned as-is (no power-of-two round-down): the cid-indexed
 * arrays (cqe_buf, sq_buf, free_cids, active_cids) are flat and directly
 * indexed by cid — no ring wraparound, doorbell, or modulo — so the INI side
 * has no power-of-two requirement.  This assumes the TGT does not require a
 * power-of-two sq_size (e.g. a ring-based TGT that masks with
 * `idx & (depth - 1)`).  The sq_size/rq_size sent in the NOF handshake
 * (nvmf_cm_conn_req.conn_info.sq_size/rq_size) carries this exact value, so
 * both sides must agree.  If the TGT ever needs a power-of-two depth, re-add
 * `1u << spdk_u32log2(v)` here.
 *
 * When g_hcomm.dev_attr is not inited (RaGetDevBaseAttr failed), no sqMaxDepth
 * clamp applies — the value matches SPDK's opts unmodified, so the transport
 * still comes up.  The caller already rejects
 * qsize < SPDK_NVME_QUEUE_MIN_ENTRIES (= 2), so in practice the result is
 * >= 2 (barring a broken sqMaxDepth < 2).
 */
static uint32_t
nvme_hcomm_qpair_sq_size(uint32_t qpair_sq_size)
{
	uint32_t v = qpair_sq_size;

	if (g_hcomm.dev_attr_inited && g_hcomm.dev_attr.sqMaxDepth > 0) {
		if (v > g_hcomm.dev_attr.sqMaxDepth) {
			SPDK_WARNLOG("requested queue depth %u exceeds device sqMaxDepth %u, clamping\n",
				     v, g_hcomm.dev_attr.sqMaxDepth);
		}
		v = spdk_min(v, g_hcomm.dev_attr.sqMaxDepth);
	}
	return v;
}

/* Return a pointer to the cid-th SQE slot in the qpair's sq_buf array.
 * Each slot is hqpair->sqe_stride bytes (base SQE + tail SGL area). */
static inline void *
nvme_hcomm_qpair_sqe(struct nvme_hcomm_qpair *hqpair, uint32_t cid)
{
	return (uint8_t *)hqpair->sq_buf + cid * hqpair->sqe_stride;
}

/*

 * ========== Per-qpair memory registration helper (HcommMemReg) ==========
 *
 * Wraps HcommMemReg for a HOST buffer on the ctrlr's endpoint.  Returns
 * the HcommMemHandle (for later unregister + HcommMemExport).  On failure
 * the handle is NULLed.
 */
static int
nvme_hcomm_register_seg(EndpointHandle ep, void *buf, size_t size,
			HcommMemHandle *out_handle)
{
	CommMem mem;
	HcommResult rc;

	mem.type = COMM_MEM_TYPE_HOST;
	mem.addr = buf;
	mem.size = (uint64_t)size;

	*out_handle = NULL;
	rc = HcommMemReg(ep, NULL, &mem, out_handle);
	if (rc != 0 || *out_handle == NULL) {
		SPDK_ERRLOG("HcommMemReg (buf=%p size=%zu) failed: ret=0x%x\n",
			    buf, size, rc);
		*out_handle = NULL;
		return -1;
	}
	return 0;
}

static void
nvme_hcomm_unregister_seg(EndpointHandle ep, HcommMemHandle handle)
{
	if (handle != NULL) {
		HcommMemUnreg(ep, handle);
	}
}

/*
 * Extract tokenId from a HcommMemExport descriptor blob.
 *
 * The blob is a BinaryStream-serialized ExchangeUbBufferDto + EndpointDesc.
 * BinaryStream writes POD types as raw bytes (no alignment, no length prefix);
 * std::string as size_t(8B) length + data.  The ExchangeUbBufferDto fields
 * in order: addr(u64) size(u64) memType(4B) memInfo(string) tokenValue(u32)
 * tokenId(u32) keySize(u32) key[64] segVa(u64) notifyId(u32).
 *
 * OWNERSHIP: the desc pointer is Desc.data() of the library's registered
 * buffer (urma_mem.cc UbRegedMemMgr::MemoryExport) - borrowed, never free
 * it (freeing double-frees inside ~LocalUbRmaBuffer when the buffer is
 * destroyed; verified by the cleanupSIGABORT report).
 *
 * Returns the tokenId, or 0 on parse failure.
 */
static uint32_t
nvme_hcomm_export_tokenid(const void *desc, uint32_t desc_len)
{
	const uint8_t *p = (const uint8_t *)desc;
	const uint8_t *end = p + desc_len;
	size_t meminfo_len;
	uint32_t tokenid;

	/* Skip addr(8) + size(8) + memType(4) = 20 bytes */
	if (p + 20 > end) return 0;
	p += 20;

	/* Read memInfo string length (size_t = 8 bytes) */
	if (p + sizeof(size_t) > end) return 0;
	memcpy(&meminfo_len, p, sizeof(size_t));
	p += sizeof(size_t);

	/* Skip memInfo data — compare in the subtraction domain: a corrupt
	 * length must not wrap the pointer arithmetic past `end`. */
	if (meminfo_len > (size_t)(end - p)) return 0;
	p += meminfo_len;

	/* Skip tokenValue (4 bytes) */
	if (p + 4 > end) return 0;
	p += 4;

	/* Read tokenId (4 bytes) */
	if (p + 4 > end) return 0;
	memcpy(&tokenid, p, 4);
	return tokenid;
}

/*
 * ========== Per-ctrlr memory registration cache (mirror spdk_rdma_mem_map) ==========
 *
 * Zero-copy data path: user payload buffers are registered with HcommMemReg +
 * HcommMemExport on first use, and the {memHandle, tokenId} result is cached
 * so subsequent I/O to the same buffer avoids re-registration.  Mirrors
 * nvme_rdma's spdk_rdma_mem_map / spdk_rdma_get_translation pattern.
 *
 * The cache is shared across all qpairs of the same controller and protected
 * by an internal mutex (qpairs may be on different reactor threads).
 * Lookup is a linear scan over a fixed-size array; when the array is full,
 * the least-recently-used entry is evicted.  The fixme.hcomm keyed-SGL model
 * (exchangeAllMems=true on the UBOE channel) means all HcommMemReg'd memory
 * is auto-exchanged with the peer — the TGT can access any registered region
 * by tokenId without explicit HcommMemImport per request, so a per-ctrlr
 * cache works naturally.
 */

#define NVME_HCOMM_MEM_MAP_ENTRIES	64

struct nvme_hcomm_mem_map_entry {
	void		*base;		/* base address of registered region */
	uint64_t	size;		/* region size */
	HcommMemHandle	handle;		/* from HcommMemReg */
	uint32_t	tokenId;	/* from HcommMemExport */
	uint32_t	refcnt;		/* in-flight requests using this entry */
	bool		in_use;
	uint64_t	last_used;	/* LRU timestamp (clock at last use) */
};

struct nvme_hcomm_mem_map {
	EndpointHandle			endpoint;
	struct nvme_hcomm_mem_map_entry	entries[NVME_HCOMM_MEM_MAP_ENTRIES];
	pthread_mutex_t			lock;	/* shared across qpairs of one ctrlr */
	uint64_t			clock;	/* monotonically increasing LRU clock */
};

/* Translation result (mirror of nvme_rdma_memory_translation_ctx).  The
 * entry_idx/handle fields identify the reference the map holds on behalf
 * of the in-flight request — release it via nvme_hcomm_mem_map_release
 * when the request completes (aborts, fails, or is reaped). */
#define NVME_HCOMM_MEM_MAP_INVALID_IDX	UINT32_MAX

struct nvme_hcomm_mem_translation {
	void		*addr;		/* translated address (= input for HOST) */
	uint32_t	length;		/* translated length */
	uint32_t	tokenId;	/* from HcommMemExport */
	uint32_t	entry_idx;	/* cache entry holding the reference,
				 * or NVME_HCOMM_MEM_MAP_INVALID_IDX */
	HcommMemHandle	handle;		/* transient registration handle
					 * (only when entry_idx is INVALID) */
};

/* Per-cid record of the mem_map reference held while a request is in
 * flight.  Stored on hqpair so the completion / abort / delete paths
 * can release exactly what the submit path acquired. */
struct nvme_hcomm_xlation_ref {
	bool		held;		/* a reference is held */
	uint32_t	entry_idx;	/* cache entry, or INVALID for transient */
	HcommMemHandle	handle;		/* transient registration handle */
};

/* Per-cid multi-segment extension of nvme_hcomm_xlation_ref: an SGL
 * payload acquires one reference per segment (each segment is
 * registered / cache-held on its own).  count == 0 means no reference
 * is held (contiguous payloads use count == 1). */
struct nvme_hcomm_xlation_refs {
	uint32_t			count;
	struct nvme_hcomm_xlation_ref	refs[NVME_HCOMM_MAX_SGL];
};

static struct nvme_hcomm_mem_map *
nvme_hcomm_mem_map_create(EndpointHandle endpoint)
{
	struct nvme_hcomm_mem_map *map = calloc(1, sizeof(*map));
	if (map != NULL) {
		map->endpoint = endpoint;
		pthread_mutex_init(&map->lock, NULL);
	}
	return map;
}

static void
nvme_hcomm_mem_map_destroy(struct nvme_hcomm_mem_map *map)
{
	uint32_t i;

	if (map == NULL) {
		return;
	}
	pthread_mutex_lock(&map->lock);
	for (i = 0; i < NVME_HCOMM_MEM_MAP_ENTRIES; i++) {
		if (map->entries[i].in_use) {
			nvme_hcomm_unregister_seg(map->endpoint,
						  map->entries[i].handle);
			map->entries[i].in_use = false;
		}
	}
	pthread_mutex_unlock(&map->lock);
	pthread_mutex_destroy(&map->lock);
	free(map);
}

/*
 * Translate a host address + length to {tokenId, addr}.  If the range
 * falls within a cached registered region, return the cached tokenId
 * (O(n) scan, n ≤ NVME_HCOMM_MEM_MAP_ENTRIES).  Otherwise register +
 * export the buffer and cache the result.  Returns 0 on success.
 *
 * The caller must release the returned reference via
 * nvme_hcomm_mem_map_release() when the request that uses this
 * translation completes (or fails).  The reference is taken atomically
 * with the lookup, so the entry cannot be evicted in between.
 */
static int
nvme_hcomm_get_memory_translation(struct nvme_hcomm_mem_map *map,
				  void *addr, uint32_t length,
				  struct nvme_hcomm_mem_translation *translation)
{
	uintptr_t req_start = (uintptr_t)addr;
	uintptr_t req_end = req_start + length;
	uint32_t i;

	/* Reject a range that wraps the address space — the containment
	 * check below must not be bypassed by an overflowed req_end. */
	if (length == 0 || req_end < req_start) {
		return -1;
	}

	pthread_mutex_lock(&map->lock);

	/* Cache lookup: find a registered region that fully contains [addr, addr+len). */
	for (i = 0; i < NVME_HCOMM_MEM_MAP_ENTRIES; i++) {
		struct nvme_hcomm_mem_map_entry *e = &map->entries[i];
		if (!e->in_use) {
			continue;
		}
		uintptr_t base = (uintptr_t)e->base;
		uintptr_t end = base + e->size;
		if (end < base) {
			continue;	/* corrupt entry, skip */
		}
		if (req_start >= base && req_end <= end) {
			e->refcnt++;	/* hold for the in-flight request */
			e->last_used = ++map->clock;
			translation->addr = addr;
			translation->length = length;
			translation->tokenId = e->tokenId;
			translation->entry_idx = i;
			translation->handle = NULL;
			pthread_mutex_unlock(&map->lock);
			return 0;
		}
	}

	/* Cache miss: register + export the user buffer. */
	{
		HcommMemHandle handle = NULL;
		void *desc = NULL;
		uint32_t desc_len = 0;
		HcommResult hrc;
		uint32_t tokenId;

		if (nvme_hcomm_register_seg(map->endpoint, addr, length,
					    &handle) != 0) {
			pthread_mutex_unlock(&map->lock);
			return -1;
		}
		hrc = HcommMemExport(map->endpoint, handle, &desc, &desc_len);
		if (hrc != 0 || desc == NULL) {
			SPDK_ERRLOG("HcommMemExport (user buf=%p len=%u) failed: ret=0x%x\n",
				    addr, length, hrc);
			nvme_hcomm_unregister_seg(map->endpoint, handle);
			pthread_mutex_unlock(&map->lock);
			return -1;
		}
		/* desc is BORROWED: Desc.data() of the library's registered
		 * buffer (urma_mem.cc MemoryExport) - parse and forget, the
		 * library-owned vector frees it at buffer destruction. */
		tokenId = nvme_hcomm_export_tokenid(desc, desc_len);
		if (tokenId == 0) {
			SPDK_ERRLOG("Failed to extract tokenId from export desc\n");
			nvme_hcomm_unregister_seg(map->endpoint, handle);
			pthread_mutex_unlock(&map->lock);
			return -1;
		}

		/* Cache the result.  Victim selection: first empty slot,
		 * else the LRU entry with NO in-flight references.  An
		 * entry with refcnt > 0 must never be evicted — an
		 * outstanding SQE still carries its tokenId and the TGT
		 * will use it to RDMA the payload (mirrors the refcount
		 * discipline of spdk_rdma_mem_map). */
		{
			uint32_t victim = UINT32_MAX;
			uint64_t victim_age = UINT64_MAX;

			for (i = 0; i < NVME_HCOMM_MEM_MAP_ENTRIES; i++) {
				if (!map->entries[i].in_use) {
					victim = i;
					break;
				}
				if (map->entries[i].refcnt == 0 &&
				    map->entries[i].last_used < victim_age) {
					victim_age = map->entries[i].last_used;
					victim = i;
				}
			}

			if (victim == UINT32_MAX) {
				/* Every entry is in use AND referenced: use
				 * this registration transiently — the
				 * request gets its token now and the
				 * release path unregisters it. */
				translation->addr = addr;
				translation->length = length;
				translation->tokenId = tokenId;
				translation->entry_idx = NVME_HCOMM_MEM_MAP_INVALID_IDX;
				translation->handle = handle;
				pthread_mutex_unlock(&map->lock);
				return 0;
			}

			if (map->entries[victim].in_use) {
				/* Evict the oldest unreferenced entry. */
				nvme_hcomm_unregister_seg(map->endpoint,
							  map->entries[victim].handle);
			}
			map->entries[victim].base = addr;
			map->entries[victim].size = length;
			map->entries[victim].handle = handle;
			map->entries[victim].tokenId = tokenId;
			map->entries[victim].refcnt = 1;
			map->entries[victim].last_used = ++map->clock;
			map->entries[victim].in_use = true;

			translation->addr = addr;
			translation->length = length;
			translation->tokenId = tokenId;
			translation->entry_idx = victim;
			translation->handle = NULL;
		}

		pthread_mutex_unlock(&map->lock);
		return 0;
	}
}

/*
 * Release a reference previously returned by
 * nvme_hcomm_get_memory_translation.  Cached entries just drop their
 * refcount (the registration stays for reuse); transient registrations
 * are unregistered here.  Clears the record so a double release is a
 * no-op.
 */
static void
nvme_hcomm_mem_map_release(struct nvme_hcomm_mem_map *map,
			   struct nvme_hcomm_xlation_ref *ref)
{
	if (map == NULL || !ref->held) {
		return;
	}

	pthread_mutex_lock(&map->lock);
	if (ref->entry_idx != NVME_HCOMM_MEM_MAP_INVALID_IDX) {
		struct nvme_hcomm_mem_map_entry *e = &map->entries[ref->entry_idx];

		assert(e->in_use && e->refcnt > 0);
		if (e->in_use && e->refcnt > 0) {
			e->refcnt--;
		}
	} else if (ref->handle != NULL) {
		nvme_hcomm_unregister_seg(map->endpoint, ref->handle);
	}
	pthread_mutex_unlock(&map->lock);

	ref->held = false;
	ref->entry_idx = NVME_HCOMM_MEM_MAP_INVALID_IDX;
	ref->handle = NULL;
}

/* Release every reference recorded in a per-cid multi-segment record
 * (contiguous: 1; SGL payload: one per segment), then reset it so a
 * double release is a no-op. */
static void
nvme_hcomm_mem_map_release_all(struct nvme_hcomm_mem_map *map,
			       struct nvme_hcomm_xlation_refs *refs)
{
	uint32_t i;

	for (i = 0; i < refs->count; i++) {
		nvme_hcomm_mem_map_release(map, &refs->refs[i]);
	}
	refs->count = 0;
}

/*
 * ========== Endpoint descriptor helpers ==========
 */
static int
nvme_hcomm_build_local_endpoint_desc(struct nvme_hcomm_ctrlr *hctrlr)
{
	EndpointDesc *ep = &hctrlr->endpointDesc;

	/* Zero the entire struct — no 0xFF garbage from EndpointDescInit. */
	memset(ep, 0, sizeof(*ep));

	ep->protocol = nvme_hcomm_protocol();
	ep->loc.locType = ENDPOINT_LOC_TYPE_HOST;

	/* Offline: endpoint is on the device; set phyId. */
	if (g_hcomm.network_mode == NETWORK_OFFLINE) {
		ep->loc.locType = ENDPOINT_LOC_TYPE_DEVICE;
		ep->loc.device.devPhyId = g_hcomm.phy_id;
	}

	/* Fill commAddr with an IP address.  HcommEndpointCreate internally
	 * converts this IP to an EID via HrtRaGetDevEidInfoList (matching
	 * against the device's NIC EID table).  So the target IP works as
	 * long as it's in the device's network — no need to specify a local
	 * IP explicitly.  SPDK_HCOMM_LOCAL_IP overrides if the target IP is
	 * not on the device's NIC. */
	{
		const char *ip = getenv("SPDK_HCOMM_LOCAL_IP");
		bool filled = false;
		if (ip != NULL && ip[0] != '\0') {
			if (inet_pton(AF_INET6, ip, &ep->commAddr.addr6) == 1) {
				ep->commAddr.type = COMM_ADDR_TYPE_IP_V6;
				filled = true;
			} else if (inet_pton(AF_INET, ip, &ep->commAddr.addr) == 1) {
				ep->commAddr.type = COMM_ADDR_TYPE_IP_V4;
				filled = true;
			}
		}
		if (!filled) {
			/* Use target address — HcommEndpointCreate will match it
			 * to the device's EID internally. */
			if (inet_pton(AF_INET6, hctrlr->target_addr,
				      &ep->commAddr.addr6) == 1) {
				ep->commAddr.type = COMM_ADDR_TYPE_IP_V6;
			} else if (inet_pton(AF_INET, hctrlr->target_addr,
					     &ep->commAddr.addr) == 1) {
				ep->commAddr.type = COMM_ADDR_TYPE_IP_V4;
			} else {
				SPDK_ERRLOG("Cannot parse target addr '%s' as IP\n",
					    hctrlr->target_addr);
				return -1;
			}
		}
	}

	return 0;
}

/*
 * Detect local HccpEid (16B) via RaGetIfaddrs + RaGetEidByIp, matching the
 * family of target_addr.  Stored in hctrlr->local_eid and published in the
 * NOF handshake request so the TGT knows the initiator endpoint.  Must run
 * after acquire_global_ctx (needs g_hcomm.ctx_handle for RaGetEidByIp) and before
 * the admin qpair connect (which sends local_eid in the NOF frame).
 *
 * Non-fatal: if detection fails (e.g. peer_online with no device EID),
 * local_eid stays zero and the TGT must rely on the channel's internal
 * state instead of the NOF frame's eid field.
 */
static int
nvme_hcomm_detect_local_eid(struct nvme_hcomm_ctrlr *hctrlr)
{
	struct RaGetIfattr ifattr;
	struct InterfaceInfo ifaddrs[16];
	unsigned int ifnum = 8;
	struct in6_addr dst;
	int family = 0;
	unsigned int i;
	int rc;

	if (inet_pton(AF_INET, hctrlr->target_addr, &dst) == 1) {
		family = AF_INET;
	} else if (inet_pton(AF_INET6, hctrlr->target_addr, &dst) == 1) {
		family = AF_INET6;
	} else {
		SPDK_ERRLOG("Invalid target address for local EID detection: %s\n",
			    hctrlr->target_addr);
		return -1;
	}

	memset(hctrlr->local_eid, 0, sizeof(hctrlr->local_eid));
	memset(&ifattr, 0, sizeof(ifattr));
	ifattr.phyId = g_hcomm.phy_id;
	ifattr.nicPosition = (unsigned int)g_hcomm.network_mode;
	ifattr.isAll = true;
	SPDK_DEBUGLOG(nvme_hcomm, "detecting local EID (RaGetIfaddrs + RaGetEidByIp, "
		      "target=%s family=%d)\n", hctrlr->target_addr, family);
	rc = RaGetIfaddrs(&ifattr, ifaddrs, &ifnum);
	if (rc != 0) {
		SPDK_WARNLOG("RaGetIfaddrs failed: ret=%d; local_eid stays zero\n", rc);
		return 0;   /* non-fatal: req will carry zero EID */
	}

	for (i = 0; i < ifnum; i++) {
		struct IpInfo ipInfo;
		union HccpEid eid;
		unsigned int eidNum = 1;

		if (ifaddrs[i].family != family) {
			continue;
		}
		if (family == AF_INET) {
			struct in_addr loopback;
			inet_pton(AF_INET, "127.0.0.1", &loopback);
			if (memcmp(&ifaddrs[i].ifaddr.ip.addr, &loopback,
				   sizeof(loopback)) == 0) {
				continue;
			}
		}
		memset(&ipInfo, 0, sizeof(ipInfo));
		ipInfo.family = ifaddrs[i].family;
		ipInfo.ip = ifaddrs[i].ifaddr.ip;
		memset(&eid, 0, sizeof(eid));
		if (RaGetEidByIp(g_hcomm.ctx_handle, &ipInfo, &eid, &eidNum) != 0 || eidNum == 0) {
			continue;
		}
		memcpy(hctrlr->local_eid, &eid, sizeof(hctrlr->local_eid));
		SPDK_DEBUGLOG(nvme_hcomm, "Local EID detected for %s, eid[0]=0x%02x\n",
			      hctrlr->target_addr, hctrlr->local_eid[0]);
		return 0;
	}

	const char *local_ip_env = getenv("SPDK_HCOMM_LOCAL_IP");
	struct in6_addr loc_addr;

	/* The env value may be unset - inet_pton(AF_INET6, NULL, ...) would
	 * dereference NULL and crash the process (verified: SIGSEGV). */
	if (local_ip_env == NULL || inet_pton(AF_INET6, local_ip_env, &loc_addr) != 1) {
		SPDK_WARNLOG("No matching local interface for %s and no usable "
			     "SPDK_HCOMM_LOCAL_IP; local_eid stays zero\n",
			     hctrlr->target_addr);
		return -1;
	}
	memcpy(hctrlr->local_eid, &loc_addr, sizeof(hctrlr->local_eid));
	SPDK_WARNLOG("No matching local interface for %s; local_eid stays %s\n",
		     hctrlr->target_addr, local_ip_env);
	return 0;
}


/*
 * ========== INI-Side Queues (HOST ↔ Device AI CPU) ==========
 *
 * CQE array: cid-indexed, 64B cacheline-aligned slots.
 *   - Physically contiguous DMA memory (TGT writes completions)
 *   - cqe_buf[cid] is at offset cid × 64
 *
 * SQ array: cid-indexed standard 64B spdk_nvme_cmd entries (mirrors
 * nvme_rdma's cmds[rdma_req->id]).  Allocating a cid == allocating an
 * SQE slot — no ring header, no doorbell, no separate full-check.
 */

/* Allocate a free cid for a new in-flight request.  cid ∈ [0, cqe_depth),
 * so it can index cqe_buf / sq_buf / req_for_cid directly.
 * Returns UINT32_MAX when the queue is fully occupied (no free cid). */
static uint32_t
nvme_hcomm_qpair_alloc_cid(struct nvme_hcomm_qpair *hqpair)
{
	uint32_t cid = hqpair->free_cid_head;

	if (cid == UINT32_MAX) {
		return UINT32_MAX; /* 队列已满，无空闲 cid */
	}
	hqpair->free_cid_head = hqpair->free_cids[cid];
	return cid;
}

/* Release a cid back to the free pool. */
static void
nvme_hcomm_qpair_free_cid(struct nvme_hcomm_qpair *hqpair, uint32_t cid)
{
	hqpair->free_cids[cid] = hqpair->free_cid_head;
	hqpair->free_cid_head = cid;
}

/* Remove the element at index idx from the in-flight list.
 * Swap-remove: the last element moves into the vacated slot (O(1),
 * order is not preserved — callers must not rely on it). */
static void
nvme_hcomm_qpair_remove_active(struct nvme_hcomm_qpair *hqpair, uint32_t idx)
{
	hqpair->active_cids[idx] = hqpair->active_cids[--hqpair->num_active];
}

static int
nvme_hcomm_create_queues(struct nvme_hcomm_qpair *hqpair, uint32_t depth)
{
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	uint32_t i;
	size_t cqe_size, sq_size;

	if (hqpair->queues_created) {
		return 0;
	}

	/* depth is the already-clamped effective depth, computed once by
	 * create_qpair (single source = qsize, clamped to sqMaxDepth).  Do NOT
	 * recompute it here — that would reintroduce a second source. */
	hqpair->cqe_depth = depth;
	hqpair->sq_depth = depth;
	hqpair->sqe_size = NVME_HCOMM_SQE_96B_SIZE;
	hqpair->sqe_stride = hqpair->sqe_size + NVME_HCOMM_MAX_SGL * 16u;

	SPDK_DEBUGLOG(nvme_hcomm, "create_queues qid=%u: depth=%u "
		      "(device sqMaxDepth=%u)\n",
		      hqpair->qpair.id, depth,
		      g_hcomm.dev_attr_inited ? g_hcomm.dev_attr.sqMaxDepth : 0);

	/* ---- Allocate CQE ring (cid-indexed, 64B-aligned, phys-contiguous) ---- */
	cqe_size = hqpair->cqe_depth * sizeof(struct nvme_hcomm_cqe_slot);
	hqpair->cqe_buf = spdk_zmalloc(cqe_size, 64, NULL, SPDK_ENV_SOCKET_ID_ANY,
				       SPDK_MALLOC_SHARE | SPDK_MALLOC_DMA);
	if (hqpair->cqe_buf == NULL) {
		SPDK_ERRLOG("Failed to allocate CQE ring for %s (size=%zu, depth=%u)\n",
			    hctrlr->target_addr, cqe_size, hqpair->cqe_depth);
		return -1;
	}
	hqpair->cqe_phys = (void *)(uintptr_t)spdk_vtophys(hqpair->cqe_buf, NULL);
	if (hqpair->cqe_phys == NULL || hqpair->cqe_phys == (void *)-1) {
		SPDK_ERRLOG("Failed to get physical addr for CQE ring\n");
		goto err_cqe;
	}

	/* ---- Allocate SQ array (cid-indexed SQE slots, stride = base +
	 * tail SGL area so any cid can host a max-size multi-SGL SQE) ---- */
	sq_size = hqpair->sq_depth * hqpair->sqe_stride;
	hqpair->sq_buf = spdk_zmalloc(sq_size, 64, NULL, SPDK_ENV_SOCKET_ID_ANY,
				      SPDK_MALLOC_SHARE | SPDK_MALLOC_DMA);
	if (hqpair->sq_buf == NULL) {
		SPDK_ERRLOG("Failed to allocate SQ array for %s (size=%zu, depth=%u)\n",
			    hctrlr->target_addr, sq_size, hqpair->sq_depth);
		goto err_sq;
	}

	hqpair->batch = spdk_zmalloc(sizeof(*hqpair->batch), 64, NULL,
				     SPDK_ENV_SOCKET_ID_ANY,
				     SPDK_MALLOC_SHARE | SPDK_MALLOC_DMA);
	hqpair->send_status = spdk_zmalloc(sizeof(*hqpair->send_status), 0, NULL,
					   SPDK_ENV_SOCKET_ID_ANY,
					   SPDK_MALLOC_SHARE | SPDK_MALLOC_DMA);
	if (hqpair->batch == NULL || hqpair->send_status == NULL) {
		SPDK_ERRLOG("Failed to allocate batch-send state for qid=%u\n",
			    hqpair->qpair.id);
		spdk_free(hqpair->batch);
		hqpair->batch = NULL;
		spdk_free(hqpair->send_status);
		hqpair->send_status = NULL;
		goto err_sq;
	}

	/* ---- Allocate cid→request tracking array ---- */
	hqpair->req_for_cid = spdk_zmalloc(hqpair->cqe_depth * sizeof(struct nvme_request *),
					   0, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_SHARE);
	if (hqpair->req_for_cid == NULL) {
		SPDK_ERRLOG("Failed to allocate req_for_cid for qid=%u\n", hqpair->qpair.id);
		goto err_req;
	}

	/* ---- Allocate free-cid pool: cids [0, cqe_depth) start free ---- */
	hqpair->free_cids = spdk_zmalloc(hqpair->cqe_depth * sizeof(uint32_t),
					0, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_SHARE);
	if (hqpair->free_cids == NULL) {
		SPDK_ERRLOG("Failed to allocate free_cids for qid=%u\n", hqpair->qpair.id);
		goto err_free;
	}
	hqpair->free_cid_head = 0;
	for (i = 0; i + 1 < hqpair->cqe_depth; i++) {
		hqpair->free_cids[i] = i + 1;
	}
	hqpair->free_cids[hqpair->cqe_depth - 1] = UINT32_MAX;

	/* ---- Allocate in-flight cid array (empty at start) ---- */
	hqpair->active_cids = spdk_zmalloc(hqpair->cqe_depth * sizeof(uint32_t),
					   0, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_SHARE);
	if (hqpair->active_cids == NULL) {
		SPDK_ERRLOG("Failed to allocate active_cids for qid=%u\n", hqpair->qpair.id);
		goto err_active;
	}
	hqpair->num_active = 0;

	/* ---- Allocate per-cid mem_map reference records (zeroed: count=0).
	 * HOST-only metadata (unlike sq_buf/cqe_buf it is not shared with the
	 * Device), but spdk_zmalloc in the dpdk env rejects flags==0, so use
	 * SPDK_MALLOC_SHARE like the other metadata arrys. */
	hqpair->xlation_for_cid = spdk_zmalloc(hqpair->cqe_depth *
			sizeof(struct nvme_hcomm_xlation_refs),
			0, NULL, SPDK_ENV_SOCKET_ID_ANY, SPDK_MALLOC_SHARE);
	if (hqpair->xlation_for_cid == NULL) {
		SPDK_ERRLOG("Failed to allocate xlation_for_cid for qid=%u\n",
			    hqpair->qpair.id);
		goto err_xlation;
	}

	hqpair->queues_created = true;

	SPDK_DEBUGLOG(nvme_hcomm, "Queues created for %s:%u qid=%u "
		      "(cqe=%zuB depth=%u, sq=%zuB depth=%u)\n",
		      hctrlr->target_addr, hctrlr->target_port, hqpair->qpair.id,
		      cqe_size, hqpair->cqe_depth, sq_size, hqpair->sq_depth);
	return 0;

	/* Error paths: each label frees only the resource allocated at its
	 * step (plus what earlier labels free via fall-through), so every
	 * failure path releases exactly the resources acquired so far — no
	 * double free, no leak. */
err_xlation:
	spdk_free(hqpair->xlation_for_cid);
	hqpair->xlation_for_cid = NULL;
err_active:
	spdk_free(hqpair->active_cids);
	hqpair->active_cids = NULL;
err_free:
	spdk_free(hqpair->free_cids);
	hqpair->free_cids = NULL;
err_req:
	spdk_free(hqpair->req_for_cid);
	hqpair->req_for_cid = NULL;
err_sq:
	spdk_free(hqpair->sq_buf);
	hqpair->sq_buf = NULL;
err_cqe:
	spdk_free(hqpair->cqe_buf);
	hqpair->cqe_buf = NULL;
	hqpair->cqe_phys = NULL;
	return -1;
}

static void
nvme_hcomm_destroy_queues(struct nvme_hcomm_qpair *hqpair)
{
	if (!hqpair->queues_created) {
		return;
	}

	if (hqpair->req_for_cid) {
		spdk_free(hqpair->req_for_cid);
		hqpair->req_for_cid = NULL;
	}
	if (hqpair->free_cids) {
		spdk_free(hqpair->free_cids);
		hqpair->free_cids = NULL;
	}
	if (hqpair->active_cids) {
		spdk_free(hqpair->active_cids);
		hqpair->active_cids = NULL;
	}
	if (hqpair->xlation_for_cid) {
		spdk_free(hqpair->xlation_for_cid);
		hqpair->xlation_for_cid = NULL;
	}
	hqpair->num_active = 0;
	hqpair->free_cid_head = UINT32_MAX;
	if (hqpair->sq_buf) {
		spdk_free(hqpair->sq_buf);
		hqpair->sq_buf = NULL;
	}
	if (hqpair->cqe_buf) {
		spdk_free(hqpair->cqe_buf);
		hqpair->cqe_buf = NULL;
		hqpair->cqe_phys = NULL;
	}
	if (hqpair->batch) {
		spdk_free(hqpair->batch);
		hqpair->batch = NULL;
	}
	if (hqpair->send_status) {
		spdk_free(hqpair->send_status);
		hqpair->send_status = NULL;
	}

	hqpair->queues_created = false;

	SPDK_DEBUGLOG(nvme_hcomm, "Queues destroyed for %s:%u qid=%u\n",
		      hqpair->hctrlr->target_addr, hqpair->hctrlr->target_port,
		      hqpair->qpair.id);
}

/*
 * ========== Controller Lifecycle ==========
 */

/* Forward declarations: ctrlr_construct creates the admin qpair (qid=0) via
 * the same create_qpair path as create_io_qpair, and unwinds construct
 * failures through nvme_ctrlr_destruct (whose pipeline deletes the adminq
 * via the transport destruct); destruct also deletes adminq.  Both are
 * defined later in the file than their first use here. */
static struct spdk_nvme_qpair *
nvme_hcomm_ctrlr_create_qpair(struct spdk_nvme_ctrlr *ctrlr, uint16_t qid,
			      uint32_t qsize, enum spdk_nvme_qprio qprio,
			      uint32_t num_requests, bool async);

static int
nvme_hcomm_ctrlr_delete_io_qpair(struct spdk_nvme_ctrlr *ctrlr,
				 struct spdk_nvme_qpair *qpair);

static struct spdk_nvme_ctrlr *
nvme_hcomm_ctrlr_construct(const struct spdk_nvme_transport_id *trid,
			   const struct spdk_nvme_ctrlr_opts *opts,
			   void *devhandle)
{
	struct nvme_hcomm_ctrlr *hctrlr;
	int rc;

	/* Resolve deployment mode (env SPDK_HCOMM_NETWORK_MODE, default offline)
	 * before any HCCP init that depends on it (socket/ctx ops selection,
	 * nicPosition, UB transport profile). */
	nvme_hcomm_init_global_settings();

	hctrlr = spdk_zmalloc(sizeof(*hctrlr), 0, NULL, SPDK_ENV_SOCKET_ID_ANY,
			      SPDK_MALLOC_SHARE);
	if (hctrlr == NULL) {
		SPDK_ERRLOG("Failed to allocate HCOMM ctrlr\n");
		return NULL;
	}

	/* spdk_zmalloc zeroes the whole ctrlr (flags, handles, dead
	 * fields) — only the socket lock needs explicit initialization. */
	pthread_mutex_init(&hctrlr->socket_lock, NULL);
	snprintf(hctrlr->target_addr, sizeof(hctrlr->target_addr), "%s", trid->traddr);
	if (trid->trsvcid[0] != '\0') {
		unsigned long port = strtoul(trid->trsvcid, NULL, 10);
		if (port > 0 && port <= UINT16_MAX) {
			hctrlr->target_port = (uint16_t)port;
		}
	}

	SPDK_DEBUGLOG(nvme_hcomm, "ctrlr_construct: target=%s:%u\n",
		      hctrlr->target_addr, hctrlr->target_port);

	/* Step C0: Initialize base NVMe controller */
	hctrlr->ctrlr.trid = *trid;
	hctrlr->ctrlr.opts = *opts;
	rc = nvme_ctrlr_construct(&hctrlr->ctrlr);
	if (rc != 0) {
		/* Base construct failed (ctrlr_lock init is its only failure
		 * mode) — the mutex is not valid, and the destruct pipeline
		 * would destroy it via nvme_ctrlr_destruct_finish(), so free
		 * directly instead of going through destruct_ctrlr. */
		spdk_free(hctrlr);
		return NULL;
	}

	/* cntlid=0xFFFF asks the TGT to dynamically assign a controller id on
	 * the admin fabrics CONNECT (same convention as the standard CONNECT
	 * data - nvme_fabric.c fills 0xFFFF there too).  The TGT returns the
	 * assigned cntlid in the CONNECT response and the fabric layer stores
	 * it back into ctrlr->cntlid, so the IO CONNECTs and every later SQE
	 * carry the real id. */
	hctrlr->ctrlr.cntlid = 0xFFFF;

	/* Step C1: Set ACL device context + create (or acquire) global endpoint.
	 * aclrtSetDevice selects which NPU to use (via thread-local context).
	 * acquire_global_endpoint builds the desc from the first caller's
	 * hctrlr and creates the endpoint; subsequent callers reuse it. */
	if (g_hcomm.network_mode == NETWORK_OFFLINE) {
		if (nvme_hcomm_acl_acquire_device() != 0) {
			goto destruct_ctrlr;
		}
	}
	if (nvme_hcomm_acquire_global_endpoint(hctrlr) != 0) {
		goto destruct_ctrlr;
	}

	/* Create the per-ctrlr memory registration cache (zero-copy
	 * user buffer registration).  Shared by all qpairs, lock-protected. */
	hctrlr->mem_map = nvme_hcomm_mem_map_create(hctrlr->endpoint);
	if (hctrlr->mem_map == NULL) {
		SPDK_ERRLOG("Failed to create mem_map\n");
		goto destruct_ctrlr;
	}

	/* Step C2: Query device capabilities (sqMaxDepth / rqMaxDepth) via
	 * RaCtxInit + RaGetDevBaseAttr.  HcommChannelCreate handles UB
	 * connection + memory exchange internally, but it doesn't expose
	 * device caps.  We need g_hcomm.dev_attr to clamp queue depths in
	 * nvme_hcomm_qpair_sq_size() before create_qpair allocates the
	 * SQ/CQE arrays — otherwise the depth may exceed the device's
	 * sqMaxDepth and HcommChannelCreate will fail.
	 *
	 * The ctx handle (g_hcomm.ctx_handle) is ONLY used for this query; it is
	 * NOT used for manual RaCtxQpCreate etc. (those are handled by
	 * HcommChannelCreate now).  Ref-counted so the first ctrlr inits,
	 * the last ctrlr deinits. */
	if (nvme_hcomm_acquire_global_ctx() != 0) {
		goto destruct_ctrlr;
	}

	/* Step C3: Detect local EID (needs g_hcomm.ctx_handle from C2 for RaGetEidByIp).
	 * Published in the NOF handshake so the TGT can identify the INI endpoint.
	 * Non-fatal: stays zero if detection fails. */
	nvme_hcomm_detect_local_eid(hctrlr);

	/* Step C5: Create the admin qpair (qid=0) — ALLOCATE ONLY, no connect.
	 *
	 * ctrlr_create_qpair allocates the qpair struct, cqe/sq arrays, registers
	 * them on the endpoint, and allocates the HCOMM thread — but does NOT
	 * create the channel or run the NOF handshake.  That happens later when
	 * the SPDK state machine reaches NVME_CTRLR_STATE_CONNECT_ADMINQ and
	 * calls ctrlr_connect_qpair(adminq):
	 *
	 *   NVME_CTRLR_STATE_CONNECT_ADMINQ
	 *     -> nvme_transport_ctrlr_connect_qpair(ctrlr, adminq)
	 *        -> ctrlr_connect_qpair(adminq)
	 *           -> spawn worker
	 *              -> qpair_connect(adminq)
	 *                 -> qpair_create_channel(adminq)  [per-qpair channel]
	 *                 -> nof_handshake(adminq)          [qid=0 CM frame]
	 *     -> NVME_CTRLR_STATE_WAIT_FOR_CONNECT_ADMINQ
	 *        -> poll process_completions until CONNECTED
	 *
	 * adminq MUST exist before construct returns (SPDK asserts adminq != NULL
	 * on the ENABLED transition).  Admin uses async=true (nvme_ctrlr.c
	 * asserts ctrlr->adminq->async == true).  Mirrors nvme_rdma_ctrlr_construct
	 * (nvme_rdma.c:2244). */
	hctrlr->ctrlr.adminq = nvme_hcomm_ctrlr_create_qpair(&hctrlr->ctrlr, 0,
			hctrlr->ctrlr.opts.admin_queue_size,
			SPDK_NVME_QPRIO_URGENT,
			hctrlr->ctrlr.opts.admin_queue_size,
			true);
	if (hctrlr->ctrlr.adminq == NULL) {
		SPDK_ERRLOG("failed to create admin qpair for %s:%u\n",
			    hctrlr->target_addr, hctrlr->target_port);
		goto destruct_ctrlr;
	}

	/* Step C6: Register the controller with the current process.  Required
	 * by the SPDK process model (mirrors nvme_rdma_ctrlr_construct at
	 * nvme_rdma.c:2252).  Pass 0 as the devhandle: it is a per-process
	 * struct spdk_pci_device * consumed only by the PCIe transport
	 * (spdk_pci_device_detach on remove_process,
	 * spdk_nvme_ctrlr_get_pci_device — both gated on trtype == PCIE).
	 * Fabric probes always hand ctrlr_construct a NULL devhandle; pass 0
	 * explicitly like nvme_rdma/tcp/vfio_user instead of storing the
	 * passed-through pointer into a PCI-typed field. */
	if (nvme_ctrlr_add_process(&hctrlr->ctrlr, 0) != 0) {
		SPDK_ERRLOG("nvme_ctrlr_add_process() failed for %s:%u\n",
			    hctrlr->target_addr, hctrlr->target_port);
		goto destruct_ctrlr;
	}

	SPDK_DEBUGLOG(nvme_hcomm, "ctrlr_construct done for %s:%u\n",
		      hctrlr->target_addr, hctrlr->target_port);
	return &hctrlr->ctrlr;

destruct_ctrlr:
	/* Full teardown via the base pipeline (same shape as nvme_rdma's
	 * destruct_ctrlr label).  The pipeline ends in
	 * nvme_hcomm_ctrlr_destruct(), which releases everything this
	 * function acquired — admin qpair, ctrlr channel, NOF socket,
	 * endpoint, global ctx, ACL device — and finishes with
	 * nvme_ctrlr_destruct_finish() to destroy the ctrlr_lock that
	 * nvme_ctrlr_construct() initialized in Step C0.
	 *
	 * Safe even on the paths that fail before the admin qpair exists
	 * (acl/endpoint/ctx/create_qpair failures) thanks to the adminq
	 * NULL guards backported from upstream cf45c42b5 ("nvme: make
	 * controller destruct safe before the admin qpair exists"). */
	nvme_ctrlr_destruct(&hctrlr->ctrlr);
	return NULL;
}

/* Forward declarations — qpair connect/disconnect are defined later
 * (after ctrlr_construct/destruct which reference them via delete_io_qpair). */
static void nvme_hcomm_qpair_disconnect_channels(struct nvme_hcomm_qpair *hqpair);
static int  nvme_hcomm_qpair_connect(struct nvme_hcomm_qpair *hqpair);
static void nvme_hcomm_qpair_abort_reqs(struct spdk_nvme_qpair *qpair, uint32_t dnr);

/* SocketDestroy is exported by libhcomm but its header is not public;
 * declared in full next to struct nvme_hcomm_socket_desc below.  Hoisted
 * here because nvme_hcomm_ctrlr_destruct calls it earlier in the file. */
extern HcommResult SocketDestroy(void *handle);

static int
nvme_hcomm_ctrlr_destruct(struct spdk_nvme_ctrlr *ctrlr)
{
	struct nvme_hcomm_ctrlr *hctrlr = nvme_hcomm_ctrlr(ctrlr);

	/* CONTRACT: must stay safe on a half-constructed ctrlr.  The
	 * ctrlr_construct failure paths unwind through nvme_ctrlr_destruct()
	 * (see the destruct_ctrlr label there), whose pipeline terminates
	 * here — every resource below is guarded (adminq / channel_created /
	 * nof_fd_handle / endpoint_created / ctx refcount / ACL), so a
	 * partially built ctrlr tears down cleanly.  Keep it that way when
	 * restructuring (e.g. the channel ctrlr→qpair sink). */
	SPDK_DEBUGLOG(nvme_hcomm, "ctrlr_destruct: %s:%u\n",
		      hctrlr->target_addr, hctrlr->target_port);

	/* Delete the admin qpair owned by the controller (created in
	 * ctrlr_construct).  Mirrors nvme_rdma_ctrlr_destruct at
	 * nvme_rdma.c:2271-2273.  disconnect_qpair must have run first for IO
	 * qpairs; adminq is torn down here.
	 *
	 * delete_io_qpair destroys the qpair's channel (per-qpair) if still
	 * alive, then frees cqe/sq/thread.  The ctrlr's shared socket and
	 * NOF fd are torn down below. */
	if (ctrlr->adminq != NULL) {
		SPDK_DEBUGLOG(nvme_hcomm, "ctrlr_destruct: deleting admin qpair\n");
		nvme_hcomm_ctrlr_delete_io_qpair(ctrlr, ctrlr->adminq);
		ctrlr->adminq = NULL;
	}

	/* Close the NOF side-channel socket if open (any protocol).
	 * Uses the shared disconnect helper + release_global_socket to
	 * balance the acquire from nvme_hcomm_nof_socket_connect.
	 * This is ctrlr-level: the NOF fd is shared by all qpairs.
	 * The socket_lock guards against a late worker thread still
	 * accessing the shared fd during destruct. */
	pthread_mutex_lock(&hctrlr->socket_lock);
	SPDK_DEBUGLOG(nvme_hcomm, "ctrlr_destruct: closing NOF side-channel socket\n");
	nvme_hcomm_nof_socket_disconnect(&hctrlr->ctrlr);

	/* Destroy the pre-created CLIENT socket (UBOE workaround, ctrlr-level). */
	if (hctrlr->socket_handle != NULL) {
		SPDK_DEBUGLOG(nvme_hcomm, "ctrlr_destruct: destroying pre-created "
			      "UBOE CLIENT socket (SocketDestroy)\n");
		SocketDestroy(hctrlr->socket_handle);
		hctrlr->socket_handle = NULL;
	}
	pthread_mutex_unlock(&hctrlr->socket_lock);

	/* Destroy the memory registration cache (unregisters all cached user
	 * buffers) before the endpoint handle is released below. */
	SPDK_DEBUGLOG(nvme_hcomm, "ctrlr_destruct: destroying mem_map "
		      "(unregisters all cached user buffers)\n");
	nvme_hcomm_mem_map_destroy(hctrlr->mem_map);
	hctrlr->mem_map = NULL;

	if (hctrlr->endpoint_created) {
		nvme_hcomm_release_global_endpoint(hctrlr);
	}

	/* Release global ctx (only used for device cap query). */
	nvme_hcomm_release_global_ctx();
	nvme_hcomm_acl_release_device();

	/* Finish base-ctrlr teardown (mirrors nvme_rdma_ctrlr_destruct at
	 * nvme_rdma.c:2288).  Must run after the adminq is deleted. */
	nvme_ctrlr_destruct_finish(ctrlr);

	pthread_mutex_destroy(&hctrlr->socket_lock);
	spdk_free(hctrlr);
	return 0;
}

static int
nvme_hcomm_ctrlr_enable(struct spdk_nvme_ctrlr *ctrlr)
{
	SPDK_DEBUGLOG(nvme_hcomm, "ctrlr_enable\n");
	return 0;
}

static uint32_t
nvme_hcomm_ctrlr_get_max_xfer_size(struct spdk_nvme_ctrlr *ctrlr)
{
	/* Zero-copy: no bounce buffer limiting transfer size.  Let the
	 * controller's MDTS moderate this value, same as nvme_rdma. */
	return UINT32_MAX;
}

static uint16_t
nvme_hcomm_ctrlr_get_max_sges(struct spdk_nvme_ctrlr *ctrlr)
{
	uint32_t sqe_base = NVME_HCOMM_SQE_96B_SIZE;
	uint32_t max_sge = NVME_HCOMM_MAX_SGL;
	uint32_t max_in_capsule_sge;

	/* Multi-SGL payloads are carried in the SQE tail array (up to
	 * NVME_HCOMM_MAX_SGL keyed descriptors; single-segment payloads
	 * keep the legacy DPTR layout).  When the TGT reports
	 * real capabilities in the Identify Controller data, clamp the
	 * usable count like nvme_rdma does (min of storage, ICD, MSDBD):
	 *
	 *   - ICD (ioccsz, 16B units of the whole SQE including the base):
	 *     the tail descriptors must fit the reported capsule size.
	 *     ioccsz == 0 or smaller than the mandatory base SQE means
	 *     "not reported" - keep the lockstep default instead of
	 *     clamping to 0 (an unupgraded private TGT reports nothing).
	 *   - MSDBD (msdbd, max SGL data block descriptors): 0 means no
	 *     limit (same reading as nvme_rdma).
	 *
	 * The slot storage stays NVME_HCOMM_MAX_SGL regardless - this only
	 * caps what submit walks, mirroring rdma's fixed 16-slot cmds[]
	 * with a runtime-clamped effective max.  The single-descriptor
	 * DPTR path needs no tail space, so floor at 1. */
	if (ctrlr->cdata.nvmf_specific.ioccsz != 0 &&
	    (uint64_t)ctrlr->cdata.nvmf_specific.ioccsz * 16u >= sqe_base) {
		max_in_capsule_sge = ((uint64_t)ctrlr->cdata.nvmf_specific.ioccsz * 16u -
				      sqe_base) / 16u;
		max_sge = spdk_min(max_sge, max_in_capsule_sge);
	}
	if (ctrlr->cdata.nvmf_specific.msdbd != 0) {
		max_sge = spdk_min(max_sge, ctrlr->cdata.nvmf_specific.msdbd);
	}

	max_sge = spdk_max(1u, max_sge);
	return max_sge;
}

/*
 * ========== Queue Pair Lifecycle ==========
 *
 * Mirror the nvme_rdma split: a private create_qpair() takes raw parameters
 * (qid + queue sizing + async flag) so that both the admin qpair (qid=0,
 * created from ctrlr_construct) and IO qpairs (qid>0, created from
 * create_io_qpair) share the exact same construction path.  create_io_qpair()
 * is the public ops entry point that just translates spdk_nvme_io_qpair_opts
 * into the raw parameters.
 */
static struct spdk_nvme_qpair *
nvme_hcomm_ctrlr_create_qpair(struct spdk_nvme_ctrlr *ctrlr, uint16_t qid,
			      uint32_t qsize, enum spdk_nvme_qprio qprio,
			      uint32_t num_requests, bool async)
{
	struct nvme_hcomm_ctrlr *hctrlr = nvme_hcomm_ctrlr(ctrlr);
	struct nvme_hcomm_qpair *hqpair;
	struct spdk_nvme_qpair *qpair;
	uint32_t effective_qsize;
	int rc;

	if (qsize < SPDK_NVME_QUEUE_MIN_ENTRIES) {
		SPDK_ERRLOG("Failed to create qpair with size %u. Minimum queue size is %d.\n",
			    qsize, SPDK_NVME_QUEUE_MIN_ENTRIES);
		return NULL;
	}

	hqpair = spdk_zmalloc(sizeof(*hqpair), 0, NULL, SPDK_ENV_SOCKET_ID_ANY,
			      SPDK_MALLOC_SHARE);
	if (hqpair == NULL) {
		SPDK_ERRLOG("Failed to allocate HCOMM qpair\n");
		return NULL;
	}

	/* hqpair is spdk_zmalloc'd — the bool/pointer/state fields below are
	 * already zeroed (thread_allocated, channel, channel_created,
	 * local_tx_psn, connect_state=IDLE, connect_result,
	 * worker_started, in_connect_poll, connect_start_ticks,
	 * debug_last_phase=IDLE).  Only hctrlr needs explicit
	 * assignment. */
	hqpair->hctrlr = hctrlr;
	qpair = &hqpair->qpair;

	/* Effective per-qpair depth (cid pool size) — single source = qsize,
	 * clamped to device sqMaxDepth.  The software request pool (free_req,
	 * sized by num_requests) cannot usefully exceed this: 1 request == 1 cid,
	 * so structs beyond sq_depth can never be in-flight.  Trim to avoid
	 * allocating dead weight. */
	effective_qsize = nvme_hcomm_qpair_sq_size(qsize);
	if (num_requests > effective_qsize) {
		SPDK_WARNLOG("qid=%u: num_requests=%u exceeds effective depth=%u, trimming\n",
			     qid, num_requests, effective_qsize);
		num_requests = effective_qsize;
	}

	/* Initialise the base spdk_nvme_qpair (state machine, free_req pool,
	 * num_outstanding_reqs, qprio, async, ctrlr back-pointer).
	 */
	rc = nvme_qpair_init(qpair, qid, ctrlr, qprio, num_requests, async);
	if (rc != 0) {
		SPDK_ERRLOG("nvme_qpair_init failed for qid=%u: rc=%d\n", qid, rc);
		spdk_free(hqpair);
		return NULL;
	}

	/* Allocate this qpair's INI-side queues (cqe_buf + sq_buf +
	 * req_for_cid).  These are PER-QPAIR resources (each qpair — admin and
	 * every IO — gets its own set), mirroring nvme_rdma_qpair. */
	rc = nvme_hcomm_create_queues(hqpair, effective_qsize);
	if (rc != 0) {
		SPDK_ERRLOG("nvme_hcomm_create_queues failed for qid=%u: rc=%d\n", qid, rc);
		nvme_qpair_deinit(qpair);
		spdk_free(hqpair);
		return NULL;
	}

	/* The blocking HCCP setup (register SQ/CQE, export CQE, allocate the
	 * comm thread) is deferred to nvme_hcomm_qpair_connect, which runs on
	 * the connect worker thread — keeping create_qpair non-blocking on the
	 * reactor (mirrors nvme_rdma deferring QP/CQ/MR to connect). */
	return qpair;
}

static struct spdk_nvme_qpair *
nvme_hcomm_ctrlr_create_io_qpair(struct spdk_nvme_ctrlr *ctrlr, uint16_t qid,
				 const struct spdk_nvme_io_qpair_opts *opts)
{
	return nvme_hcomm_ctrlr_create_qpair(ctrlr, qid, opts->io_queue_size, opts->qprio,
					    opts->io_queue_requests, opts->async_mode);
}

static int
nvme_hcomm_ctrlr_delete_io_qpair(struct spdk_nvme_ctrlr *ctrlr, struct spdk_nvme_qpair *qpair)
{
	struct nvme_hcomm_ctrlr *hctrlr = nvme_hcomm_ctrlr(ctrlr);
	struct nvme_hcomm_qpair *hqpair = nvme_hcomm_qpair(qpair);

	SPDK_DEBUGLOG(nvme_hcomm, "delete_io_qpair qid=%u\n", qpair->id);

	/* If a connect worker is still running, join it first.  After the
	 * join this thread exclusively owns every hcomm resource of the
	 * qpair (the worker may have published LINK_UP without noticing an
	 * abort that raced in - the idempotent sweep below covers every
	 * terminal phase it may have left behind). */
	if (hqpair->worker_started) {
		SPDK_DEBUGLOG(nvme_hcomm, "delete_io_qpair qid=%u: joining connect worker "
			      "(it may still be inside the blocking handshake)\n",
			      qpair->id);
		pthread_join(hqpair->connect_worker, NULL);
		hqpair->worker_started = false;
	}

	/* Reap any in-flight requests before the queue arrays are freed below,
	 * so destroy_queues never frees memory still referenced by an
	 * outstanding request (mirrors nvme_rdma_ctrlr_delete_io_qpair). */
	SPDK_DEBUGLOG(nvme_hcomm, "delete_io_qpair qid=%u: aborting in-flight "
		      "requests\n", qpair->id);
	nvme_hcomm_qpair_abort_reqs(qpair, 0);

	/* Release any mem_map references still held by this qpair.  The
	 * base layer normally aborts in-flight requests (abort_reqs
	 * releases each reference); this sweep is the safety net for the
	 * paths where it does not — without it a pinned entry could never
	 * be evicted again. */
	{
		uint32_t i;

		for (i = 0; i < hqpair->cqe_depth; i++) {
			nvme_hcomm_mem_map_release_all(hctrlr->mem_map,
						   &hqpair->xlation_for_cid[i]);
		}
	}

	/* Unregister HcommMemReg segments (reverse order of create_qpair). */
	nvme_hcomm_unregister_seg(hctrlr->endpoint, hqpair->cqe_lmem_handle);
	hqpair->cqe_lmem_handle = NULL;
	nvme_hcomm_unregister_seg(hctrlr->endpoint, hqpair->sq_lmem_handle);
	hqpair->sq_lmem_handle = NULL;

	/* Destroy this qpair's channel if still alive (per-qpair, not shared).
	 * Normally disconnect_qpair runs first, but guard for safety. */
	nvme_hcomm_qpair_disconnect_channels(hqpair);

	if (hqpair->thread_allocated) {
		HcommResult hret = HcommThreadFree(&hqpair->thread, 1);
		if (hret != 0) {
			SPDK_ERRLOG("HcommThreadFree failed: ret=0x%x\n", hret);
		}
		hqpair->thread_allocated = false;
	}


	/* Free CQE array + SQ array + req_for_cid.
	 * Symmetric to nvme_hcomm_create_queues in create_qpair. */
	nvme_hcomm_destroy_queues(hqpair);

	/* Tear down the base qpair (frees the request pool allocated in
	 * nvme_qpair_init).  Mirrors nvme_rdma_ctrlr_delete_io_qpair. */
	nvme_qpair_deinit(qpair);

	spdk_free(hqpair);
	return 0;
}

/*
 * ========== Connect / Disconnect ==========
 *
 * Both UBOE and UBC_TP use HcommChannelCreate, which handles socket
 * connect + UB connection + memory exchange (exchangeAllMems) internally.
 * The protocol field in the channel desc (nvme_hcomm_protocol()) determines
 * which transport is used.
 */

/* ---- Channel path: HcommChannelCreate (both UBOE and UBC_TP) ---- */

/*
 * channelName format: "nvme_hcomm_<ctrlid>_<qid>".  The name doubles as
 * the HCCP socketTag on the library-internal socket path (UBC_TP /
 * RoCE): the tag travels with the connection, keys the whitelist match
 * and pools sockets in SocketMgr, so the TGT can bind the socket
 * handshake to the right qpair and concurrent qpair connects each get
 * their own socket.  ctrlid is the currently assigned controller id -
 * 0 on the first admin connect (the controller does not exist yet),
 * the assigned cntlid on IO qpairs, reconnects included.
 *
 * Must stay below HCOMM_CHANNEL_NAME_MAX_LEN (191, strnlen-checked in
 * hcomm_c_adpt.cc).  Longest value "nvme_hcomm_65535_65535" is 24+NUL.
 */
#define NVME_HCOMM_CHANNEL_NAME_LEN	32

/*
 * Why we pre-create the socket: AicpuTsUboeChannel::BuildSocket
 * (hcomm/.../aicpu_ts_uboe_channel.cc) ignores channelDesc.role and
 * auto-determines the role by IP byte-comparison:
 *
 *   SocketConfig(linkData, tag, noRankId) ->
 *     role(link.GetLocalAddr() < link.GetRemoteAddr() ? SERVER : CLIENT)
 *
 * (socket_config.h, SocketConfig(link, tag, noRankId) ctor).  When the
 * NPU local IP < TGT remote IP (e.g. 10.10.24.2 < 10.10.30.2 in network
 * byte order), SPDK becomes SERVER; Socket::ConnectAsync() then early-
 * returns (socket.cc: "if (role == SERVER) return;") and no packet is
 * ever sent to TGT.
 *
 * desc.role (set to HCOMM_SOCKET_ROLE_CLIENT above) is only honoured by
 * the RoCE/URMA/HCCS channels (host_cpu_urma_channel.cc:112,
 * aicpu_ts_roce_channel_v2.cc:265, …), NOT by the UBOE channel.
 *
 * Workaround: pre-create a CLIENT socket via SocketCreate() — which
 * DOES respect socketDesc.role (socket_process.cc:BuildSocket uses the
 * SocketConfig(link, tag, role, port) ctor with the explicit role) —
 * and hand it to HcommChannelCreate through desc.socket.  BuildSocket
 * then early-returns on socket_ != nullptr and uses our CLIENT socket
 * directly, skipping the IP-comparison path entirely.
 *
 * SocketCreate/SocketDestroy are exported from libhcomm but their header
 * (hccl_comm_socket_c_adpt.h) is not in the public include path, so we
 * forward-declare them here.  The SocketDesc layout below mirrors the
 * library struct exactly; all member types (CommAbiHeader, EndpointDesc,
 * HcommSocketRole) come from the public hcomm_res_defs.h already included.
 */
#define NVME_HCOMM_SOCKET_TAG_LEN	192
#define NVME_HCOMM_UBOE_LISTEN_PORT	60001	/* TempServerListenPort, matches aicpu_ts_uboe_channel.cc */

struct nvme_hcomm_socket_desc {
	CommAbiHeader		header;
	EndpointDesc		localEndpoint;
	EndpointDesc		remoteEndpoint;
	char			tag[NVME_HCOMM_SOCKET_TAG_LEN];
	HcommSocketRole		role;
	uint16_t		listenPort;
	char			reserved[256];
};

extern HcommResult SocketCreate(struct nvme_hcomm_socket_desc *desc, void **handle);

/* Build a SocketDesc for the pre-created CLIENT socket.  localEndpoint
 * mirrors the ctrlr's endpoint (same local IP + protocol); remoteEndpoint
 * is the TGT.  role=CLIENT forces outbound connect regardless of IP order. */
static int
nvme_hcomm_create_socket(struct nvme_hcomm_ctrlr *hctrlr, void **out_handle)
{
	struct nvme_hcomm_socket_desc sd;
	HcommResult ret;

	memset(&sd, 0, sizeof(sd));
	sd.header.version = HCOMM_CHANNEL_VERSION;
	sd.header.magicWord = HCOMM_CHANNEL_MAGIC_WORD;
	sd.header.size = sizeof(sd);

	/* Local endpoint: same as the one passed to HcommEndpointCreate
	 * (hctrlr->endpointDesc, built by nvme_hcomm_build_local_endpoint_desc).
	 * Carries the NPU NIC IP used for EID resolution + source binding. */
	sd.localEndpoint = hctrlr->endpointDesc;

	/* Remote endpoint: TGT IP + protocol (from env var). */
	sd.remoteEndpoint.protocol = nvme_hcomm_protocol();
	sd.remoteEndpoint.loc.locType = ENDPOINT_LOC_TYPE_HOST;
	if (inet_pton(AF_INET, hctrlr->target_addr,
		      &sd.remoteEndpoint.commAddr.addr) == 1) {
		sd.remoteEndpoint.commAddr.type = COMM_ADDR_TYPE_IP_V4;
	} else if (inet_pton(AF_INET6, hctrlr->target_addr,
			     &sd.remoteEndpoint.commAddr.addr6) == 1) {
		sd.remoteEndpoint.commAddr.type = COMM_ADDR_TYPE_IP_V6;
	} else {
		SPDK_ERRLOG("Cannot parse target addr '%s'\n", hctrlr->target_addr);
		return -1;
	}

	/* The pre-created socket is ctrlr-level (shared by all qpairs), so
	 * its tag cannot carry a per-qpair identity - keep the base name.
	 * Per-qpair binding on UBOE goes through the NOF handshake frame
	 * (qid); the channelName tag only takes effect on the
	 * library-internal socket path (UBC_TP). */
	snprintf(sd.tag, sizeof(sd.tag), "nvme_hcomm");
	sd.role = HCOMM_SOCKET_ROLE_CLIENT;
	sd.listenPort = NVME_HCOMM_UBOE_LISTEN_PORT;

	SPDK_NOTICELOG("SocketCreate CLIENT to %s (local %s, port %u)\n",
		       hctrlr->target_addr,
		       hctrlr->endpointDesc.commAddr.type == COMM_ADDR_TYPE_IP_V4
		       ? "ipv4" : "ipv6", NVME_HCOMM_UBOE_LISTEN_PORT);

	ret = SocketCreate(&sd, out_handle);
	if (ret != 0 || *out_handle == NULL) {
		SPDK_ERRLOG("SocketCreate failed: ret=0x%x\n", ret);
		*out_handle = NULL;
		return -1;
	}
	return 0;
}

static int
nvme_hcomm_qpair_create_channel(struct nvme_hcomm_qpair *hqpair)
{
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	char channel_name[NVME_HCOMM_CHANNEL_NAME_LEN];
	HcommChannelDesc desc;
	ChannelHandle channel;
	HcommResult hret;

	/* Build HcommChannelDesc. */
	HcommChannelDescInit(&desc, 1);
	desc.header.version = HCOMM_CHANNEL_VERSION;
	desc.header.magicWord = HCOMM_CHANNEL_MAGIC_WORD;
	desc.header.size = sizeof(HcommChannelDesc);

	/* Remote endpoint: target IP + protocol (UBOE or UBC_TP, from env var). */
	desc.remoteEndpoint.protocol = nvme_hcomm_protocol();
	desc.remoteEndpoint.loc.locType = ENDPOINT_LOC_TYPE_HOST;
	if (inet_pton(AF_INET, hctrlr->target_addr,
		      &desc.remoteEndpoint.commAddr.addr) == 1) {
		desc.remoteEndpoint.commAddr.type = COMM_ADDR_TYPE_IP_V4;
	} else if (inet_pton(AF_INET6, hctrlr->target_addr,
			     &desc.remoteEndpoint.commAddr.addr6) == 1) {
		desc.remoteEndpoint.commAddr.type = COMM_ADDR_TYPE_IP_V6;
	} else {
		SPDK_ERRLOG("Cannot parse target addr '%s'\n", hctrlr->target_addr);
		goto err;
	}

	desc.role = HCOMM_SOCKET_ROLE_CLIENT;
	/* Channel-socket connect port: the TGT's HCCP listening port, NOT the
	 * trsvcid/NOF port.  Defaults to the library's own DEFAULT_LISTENING
	 *_PORT (60001); SPDK_HCOMM_CHANNEL_PORT overrides (see
	 * nvme_hcomm_channel_port_init). */
	desc.port = g_hcomm.channel_port;
	desc.exchangeAllMems = true;
	desc.notifyNum = NVME_HCOMM_DEFAULT_NOTIFY_NUM;
	/* Per-qpair identity for the TGT - see the channelName note above
	 * (ctrlid is 0 on the first admin connect).  The UBOE pre-created
	 * socket path bypasses BuildSocket and keeps the ctrlr-level
	 * "nvme_hcomm" tag, so there the qpair binding stays with the NOF
	 * handshake frame. */
	snprintf(channel_name, sizeof(channel_name), "nvme_hcomm_%u_%u",
		 hctrlr->ctrlr.cntlid, hqpair->qpair.id);
	desc.channelName = channel_name;

	/* Pre-create a CLIENT socket for UBOE only — this works around the
	 * AicpuTsUboeChannel::BuildSocket IP-comparison bug (which makes
	 * SPDK a SERVER when local IP < remote IP).  UBC_TP uses
	 * AivUrmaChannel which handles socket creation internally (via
	 * RaSocketInit, supporting HDC/offline mode), so desc.socket stays
	 * NULL and the library manages the socket.
	 *
	 * The socket is stored on hctrlr (shared by all qpairs): the first
	 * qpair to connect creates it, subsequent qpairs reuse it.  The
	 * socket_lock serialises the check-then-create race against
	 * concurrent IO qpair connect (worker threads). */
	pthread_mutex_lock(&hctrlr->socket_lock);
	if (nvme_hcomm_protocol() == COMM_PROTOCOL_UBOE) {
		if (hctrlr->socket_handle == NULL) {
			if (nvme_hcomm_create_socket(hctrlr, &hctrlr->socket_handle) != 0) {
				pthread_mutex_unlock(&hctrlr->socket_lock);
				return -1;
			}
		}
		desc.socket = hctrlr->socket_handle;
	}

	/* NOF side-channel socket (connect to target_port, CLIENT only).
	 * Shared on hctrlr — first qpair creates, rest reuse. */
	if (hctrlr->nof_fd_handle == NULL) {
		SPDK_DEBUGLOG(nvme_hcomm, "worker qid=%u: connecting NOF side-channel "
			      "socket to %s:%u (first qpair only)\n",
			      hqpair->qpair.id, hctrlr->target_addr,
			      hctrlr->target_port);
		if (nvme_hcomm_nof_socket_connect(&hctrlr->ctrlr,
						  hctrlr->target_port) != 0) {
			SPDK_ERRLOG("NOF socket connect to %s:%u failed\n",
				    hctrlr->target_addr, hctrlr->target_port);
			pthread_mutex_unlock(&hctrlr->socket_lock);
			goto err;
		}
	}
	pthread_mutex_unlock(&hctrlr->socket_lock);

	SPDK_NOTICELOG("HcommChannelCreate to %s:%u (qid=%u)\n",
		       hctrlr->target_addr, hctrlr->target_port, hqpair->qpair.id);

	hret = HcommChannelCreate(hctrlr->endpoint, NVME_HCOMM_DEFAULT_ENGINE,
				  &desc, 1, &channel);
	if (hret != 0) {
		SPDK_ERRLOG("HcommChannelCreate failed: ret=0x%x\n", hret);
		goto err;
	}
	hqpair->channel = channel;
	hqpair->channel_created = true;

	SPDK_DEBUGLOG(nvme_hcomm, "channel ready for %s:%u (qid=%u)\n",
		      hctrlr->target_addr, hctrlr->target_port, hqpair->qpair.id);
	return 0;

err:
	if (hqpair->channel_created) {
		HcommChannelDestroy(&hqpair->channel, 1);
		hqpair->channel_created = false;
	}
	return -1;
}

/*
 * Unwind everything nvme_hcomm_qpair_connect sets up (channel, comm
 * thread, SQ/CQE registrations), in reverse order.  Idempotent - safe to
 * call from any failure/abort path regardless of how far the connect
 * got.  Used by qpair_connect's error path, by the connect worker when
 * a disconnect races the handshake, and by the connect poll failure and
 * disconnect paths, so a later reconnect always starts clean.
 */
static void
nvme_hcomm_qpair_connect_cleanup(struct nvme_hcomm_qpair *hqpair)
{
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;

	SPDK_DEBUGLOG(nvme_hcomm, "connect_cleanup qid=%u: releasing handshake resources\n",
		      hqpair->qpair.id);

	SPDK_DEBUGLOG(nvme_hcomm, "connect_cleanup qid=%u: unregistering CQE "
		      "(HcommMemUnreg)\n", hqpair->qpair.id);
	nvme_hcomm_unregister_seg(hctrlr->endpoint, hqpair->cqe_lmem_handle);
	hqpair->cqe_lmem_handle = NULL;
	SPDK_DEBUGLOG(nvme_hcomm, "connect_cleanup qid=%u: unregistering SQ "
		      "(HcommMemUnreg)\n", hqpair->qpair.id);
	nvme_hcomm_unregister_seg(hctrlr->endpoint, hqpair->sq_lmem_handle);
	hqpair->sq_lmem_handle = NULL;

	nvme_hcomm_qpair_disconnect_channels(hqpair);
	if (hqpair->thread_allocated) {
		SPDK_DEBUGLOG(nvme_hcomm, "connect_cleanup qid=%u: freeing comm thread "
			      "(HcommThreadFree)\n", hqpair->qpair.id);
		HcommThreadFree(&hqpair->thread, 1);
		hqpair->thread_allocated = false;
	}
	SPDK_DEBUGLOG(nvme_hcomm, "connect_cleanup qid=%u: done\n", hqpair->qpair.id);
}

static int
nvme_hcomm_qpair_connect(struct nvme_hcomm_qpair *hqpair)
{
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	uint32_t notifyNum = NVME_HCOMM_DEFAULT_NOTIFY_NUM;
	HcommResult hret;

	/* Clear the CQE array: slots may still carry padding[0] valid flags
	 * from a previous connection whose CQEs were never reaped (aborted
	 * in flight) - a stale flag would read as a bogus completion after
	 * reconnect.  No command is outstanding on this qpair yet, so the
	 * TGT cannot race this write. */
	memset(hqpair->cqe_buf, 0,
	       (size_t)hqpair->cqe_depth * sizeof(struct nvme_hcomm_cqe_slot));

	/* ---- Blocking HCCP setup (runs on this connect worker thread, NOT in
	 * create_qpair): register SQ/CQE, export CQE, allocate the comm thread.
	 * Ordering: registrations precede HcommChannelCreate (exchangeAllMems
	 * swaps all registered memory there), and the CQE export precedes the
	 * NOF handshake (which ships cqe_tokenid). ---- */

	/* Register SQ array (source buffer of the bilateral SEND — the
	 * Device-side A5 transport reads the SQE bytes from it when
	 * executing HcommSendOnThread).  The registration must cover the
	 * full slot stride - the tail SGL descriptor area is part of the
	 * sent bytes for multi-SGL SQEs. */
	SPDK_DEBUGLOG(nvme_hcomm, "worker qid=%u: registering SQ array %u x %u B "
		      "(HcommMemReg)\n", hqpair->qpair.id, hqpair->sq_depth,
		      hqpair->sqe_stride);
	if (nvme_hcomm_register_seg(hctrlr->endpoint, hqpair->sq_buf,
				    (size_t)hqpair->sq_depth * hqpair->sqe_stride,
				    &hqpair->sq_lmem_handle) != 0) {
		SPDK_ERRLOG("Failed to HcommMemReg SQ for qid=%u\n", hqpair->qpair.id);
		goto err_cleanup;
	}
	SPDK_DEBUGLOG(nvme_hcomm, "worker qid=%u: SQ registered (%u slots x %u B)\n",
		      hqpair->qpair.id, hqpair->sq_depth, hqpair->sqe_stride);

	/* Register CQE ring (TGT RDMA-writes completions into it). */
	SPDK_DEBUGLOG(nvme_hcomm, "worker qid=%u: registering CQE ring %u x 64 B "
		      "(HcommMemReg)\n", hqpair->qpair.id, hqpair->cqe_depth);
	if (nvme_hcomm_register_seg(hctrlr->endpoint, hqpair->cqe_buf,
				    (size_t)hqpair->cqe_depth * sizeof(struct nvme_hcomm_cqe_slot),
				    &hqpair->cqe_lmem_handle) != 0) {
		SPDK_ERRLOG("Failed to HcommMemReg CQE for qid=%u\n", hqpair->qpair.id);
		goto err_cleanup;
	}
	SPDK_DEBUGLOG(nvme_hcomm, "worker qid=%u: CQE ring registered (%u x 64 B)\n",
		      hqpair->qpair.id, hqpair->cqe_depth);

	/* Export CQE → tokenId (TGT uses it to locate the CQE ring).
	 * Ownership: HcommMemExport hands out Desc.data() - a BORROWED
	 * pointer into a std::vector member of the library's registered
	 * buffer (urma_mem.cc UbRegedMemMgr::MemoryExport).  Parse and
	 * forget it; NEVER free - the vector still owns the memory and
	 * freeing it here double-frees when the buffer is destroyed
	 * (cleanupSIGABRT: free() directly inside ~LocalUbRmaBuffer). */
	{
		void *desc = NULL;
		uint32_t desc_len = 0;
		HcommResult hrc;

		SPDK_DEBUGLOG(nvme_hcomm, "worker qid=%u: exporting CQE ring "
			      "(HcommMemExport)\n", hqpair->qpair.id);
		hrc = HcommMemExport(hctrlr->endpoint, hqpair->cqe_lmem_handle,
				     &desc, &desc_len);
		if (hrc != 0 || desc == NULL) {
			SPDK_ERRLOG("HcommMemExport (cqe) failed: ret=0x%x\n", hrc);
			goto err_cleanup;
		}
		hqpair->cqe_tokenid = nvme_hcomm_export_tokenid(desc, desc_len);
		if (hqpair->cqe_tokenid == 0) {
			SPDK_ERRLOG("Failed to extract tokenId from CQE export desc\n");
			goto err_cleanup;
		}
		SPDK_NOTICELOG("CQE ring exported: tokenid=%u\n", hqpair->cqe_tokenid);
	}

	/* Allocate the HCOMM communication thread (AICPU+TS engine). */
	SPDK_DEBUGLOG(nvme_hcomm, "worker qid=%u: allocating comm thread "
		      "(HcommThreadAlloc, engine=AICPU_TS)\n", hqpair->qpair.id);
	hret = HcommThreadAlloc(NVME_HCOMM_DEFAULT_ENGINE, 1, &notifyNum, &hqpair->thread);
	if (hret != 0) {
		SPDK_ERRLOG("HcommThreadAlloc failed for qid=%u: ret=0x%x\n",
			    hqpair->qpair.id, hret);
		goto err_cleanup;
	}
	hqpair->thread_allocated = true;
	SPDK_DEBUGLOG(nvme_hcomm, "worker qid=%u: comm thread allocated\n",
		      hqpair->qpair.id);

	/* Each qpair creates its own channel (one qpair = one channel).
	 * The ctrlr's shared socket_handle / nof_fd_handle are reused. */
	if (!hqpair->channel_created) {
		if (nvme_hcomm_qpair_create_channel(hqpair) != 0) {
			goto err_cleanup;
		}
	}
	SPDK_DEBUGLOG(nvme_hcomm, "worker qid=%u: channel ready\n",
		      hqpair->qpair.id);

	/* Per-qpair NOF handshake.
	 * Sends qid + sq/rq_size + ini_cqe_addr (rsq_phys) to TGT. */
	if (hctrlr->nof_fd_handle != NULL) {
		if (nvme_hcomm_nof_handshake(hqpair) != 0) {
			SPDK_ERRLOG("NOF handshake to %s:%u (qid=%u) failed\n",
				    hctrlr->target_addr, hctrlr->target_port,
				    hqpair->qpair.id);
			goto err_cleanup;
		}
		SPDK_DEBUGLOG(nvme_hcomm, "worker qid=%u: NOF handshake done\n",
			      hqpair->qpair.id);
	}

	return 0;

err_cleanup:
	nvme_hcomm_qpair_connect_cleanup(hqpair);
	return -1;
}

static void
nvme_hcomm_qpair_disconnect_channels(struct nvme_hcomm_qpair *hqpair)
{
	/* Destroy this qpair's channel (per-qpair, not shared). */
	if (hqpair->channel_created) {
		SPDK_DEBUGLOG(nvme_hcomm, "qid=%u: destroying channel "
			      "(HcommChannelDestroy, may block on teardown)\n",
			      hqpair->qpair.id);
		HcommChannelDestroy(&hqpair->channel, 1);
		hqpair->channel_created = false;
	}
	/* Socket/NOF teardown is ctrlr-level (shared by all qpairs);
	 * handled in ctrlr_destruct, not here. */
}

/* ---- Connect/dispatch ---- */

/*
 * Async connect, three phases (per qpair):
 *
 *   phase 1 (worker thread):  nvme_hcomm_qpair_connect - blocking
 *      HcommMemReg/export + HcommThreadAlloc + HcommChannelCreate + NOF
 *      handshake (bounded by SPDK_HCOMM_SOCKET_TIMEOUT_MS).  The ctrlr's
 *      shared socket_handle / nof_fd_handle are reused by all qpairs -
 *      the first qpair to connect creates them.
 *
 *   phase 2 (poll thread):    fabrics CONNECT submit - after the worker
 *      publishes LINK_UP, nvme_hcomm_qpair_connect_poll sends the
 *      NVMe-oF CONNECT command via nvme_fabric_qpair_connect_async (the
 *      admin CONNECT creates the controller at the TGT and returns
 *      cntlid; IO CONNECTs bind their qid).  rdma/tcp drive their
 *      fabrics CONNECT from the poll path the same way.
 *
 *   phase 3 (poll thread):    fabrics CONNECT CQE reap - on success the
 *      qpair is set CONNECTED; the framework then promotes it
 *      (WAIT_FOR_CONNECT_ADMINQ / check_enabled) and resends anything
 *      queued while it was CONNECTING.
 *
 * ctrlr_connect_qpair only spawns the worker and returns 0 - the qpair
 * stays CONNECTING and every later step is driven by polling the qpair
 * (framework adminq poller, the sync io-qpair busy-wait, or a poll
 * group), which is also the only thread that ever submits/reaps NVMe
 * traffic on it.
 *
 * Disconnect path (per qpair):
 *   ctrlr_disconnect_qpair(qpair)
 *     ├─ WORKING: wait (bounded) for the worker to publish LINK_UP / FAILED,
 *     │           then unwind here — the disconnector owns the cleanup
 *     ├─ FABRIC_SENT: drain the fabrics CONNECT, then unwind
 *     └─ otherwise: nvme_hcomm_qpair_connect_cleanup(hqpair)
 *     then nvme_transport_ctrlr_disconnect_qpair_done(qpair) - abort
 *     queued requests and set DISCONNECTED (pcie/tcp pattern) so both
 *     WAIT_FOR_CONNECT_ADMINQ and reconnect observe a terminal state.
 *
 * Cleanup ownership: the worker only unwinds its own error path (before
 * publishing FAILED).  After any terminal publish the handshake resources
 * belong to the next disconnect/unwind - or to the stale-resource sweep in
 * ctrlr_connect_qpair when a disconnect gave up on a stuck worker - so
 * exactly one thread ever cleans a given qpair's handshake resources.
 */

/* Drain an in-flight fabrics CONNECT during disconnect: poll its CQE
 * (bounded) so the fabric layer frees its status/data and completes the
 * reserved request before the channel is destroyed.  Bounded by the
 * fabrics connect timeout when configured, else a fixed floor - the
 * fabric layer itself never times out when the opt is 0. */
static void
nvme_hcomm_connect_drain_fabric(struct spdk_nvme_qpair *qpair)
{
	struct spdk_nvme_ctrlr *ctrlr = qpair->ctrlr;
	uint64_t timeout_us = ctrlr->opts.fabrics_connect_timeout_us;
	uint64_t waited_us = 0;
	uint64_t deadline;

	if (timeout_us == 0) {
		timeout_us = NVME_HCOMM_CONNECT_DRAIN_TIMEOUT_US;
	}
	SPDK_DEBUGLOG(nvme_hcomm, "qid=%u: draining in-flight fabrics CONNECT "
		      "(timeout=%llu us)\n", qpair->id, (unsigned long long)timeout_us);
	deadline = spdk_get_ticks() + timeout_us * spdk_get_ticks_hz() /
		   SPDK_SEC_TO_USEC;

	while (spdk_get_ticks() < deadline) {
		if (nvme_fabric_qpair_connect_poll(qpair) != -EAGAIN) {
			SPDK_DEBUGLOG(nvme_hcomm, "qid=%u: fabrics CONNECT drained "
				      "(%llu us)\n", qpair->id,
				      (unsigned long long)waited_us);
			return;
		}
		spdk_delay_us(100);
		waited_us += 100;
		if (waited_us % 1000000 == 0) {
			/* Once-per-second breadcrumb while the drain waits. */
			SPDK_DEBUGLOG(nvme_hcomm, "qid=%u: fabrics CONNECT drain still "
				      "waiting (%llu us elapsed)\n", qpair->id,
				      (unsigned long long)waited_us);
		}
	}
	SPDK_ERRLOG("fabrics CONNECT drain timed out on qid=%u; its request "
		    "is aborted at qpair destruction\n", qpair->id);
}

/* Worker thread body (phase 1): runs the blocking hcomm-level handshake
 * and publishes the result through connect_state.  It never touches
 * NVMe/qpair state machine fields - see the enum above. */
static void *
nvme_hcomm_connect_worker(void *arg)
{
	struct nvme_hcomm_qpair *hqpair = arg;
	int rc;

	SPDK_DEBUGLOG(nvme_hcomm, "connect worker start (qid=%u)\n",
		      hqpair->qpair.id);

	if (g_hcomm.network_mode == NETWORK_OFFLINE) {
		SPDK_DEBUGLOG(nvme_hcomm, "worker qid=%u: offline mode, calling aclrtSetDevice(%d)"
			      " (may block on device init)\n", hqpair->qpair.id,
			      g_hcomm.acl_logic_dev_id);
		rc = aclrtSetDevice(g_hcomm.acl_logic_dev_id);
		if (rc != 0) {
			SPDK_ERRLOG("aclrtSetDevice(%d) failed rc=%d\n", g_hcomm.acl_logic_dev_id, rc);
			goto fail;
		}
		SPDK_DEBUGLOG(nvme_hcomm, "worker aclrtSetDevice(%d) ok\n", g_hcomm.acl_logic_dev_id);
	}

	rc = nvme_hcomm_qpair_connect(hqpair);
	if (rc == 0) {
		/* Publish for phase 2 (the poll thread sends the fabrics
		 * CONNECT).  The worker never cleans up after publishing:
		 * once LINK_UP (or FAILED, after the error-path unwind) is
		 * visible, cleanup belongs to whoever disconnects/unwinds
		 * next — see ctrlr_disconnect_qpair. */
		SPDK_DEBUGLOG(nvme_hcomm, "connect worker qid=%u: handshake done in %u ms, publishing LINK_UP\n",
			      hqpair->qpair.id, nvme_hcomm_connect_elapsed_ms(hqpair));
		nvme_hcomm_conn_state_store(hqpair, NVME_HCOMM_CONN_LINK_UP);
	} else {
		/* the error path of qpair_connect already unwound */
		SPDK_DEBUGLOG(nvme_hcomm, "connect worker qid=%u: handshake failed rc=%d after %u ms (error path already unwound)\n",
			      hqpair->qpair.id, rc, nvme_hcomm_connect_elapsed_ms(hqpair));
		hqpair->connect_result = rc;
		nvme_hcomm_conn_state_store(hqpair, NVME_HCOMM_CONN_FAILED);
	}

	if (g_hcomm.network_mode == NETWORK_OFFLINE) {
		SPDK_DEBUGLOG(nvme_hcomm, "worker qid=%u: offline tail, calling "
			      "aclrtResetDevice(%d) before exit (may block; a join "
			      "on this worker waits for it)\n",
			      hqpair->qpair.id, g_hcomm.acl_logic_dev_id);
		rc = aclrtResetDevice(g_hcomm.acl_logic_dev_id);
		SPDK_DEBUGLOG(nvme_hcomm, "worker aclrtResetDevice(%d) rc=%d\n", g_hcomm.acl_logic_dev_id, rc);
	}

	return NULL;

fail:
	/* No handshake resources exist (qpair_connect never ran), but a
	 * terminal state MUST be published: ctrlr_disconnect_qpair waits
	 * (bounded) for LINK_UP/FAILED, and reconnect is locked out for as
	 * long as connect_state stays WORKING. */
	hqpair->connect_result = -EIO;
	nvme_hcomm_conn_state_store(hqpair, NVME_HCOMM_CONN_FAILED);
	return NULL;
}

/* Failure unwind for the async connect (phases 2/3): reset the phase,
 * release the handshake resources (idempotent - the worker already did
 * for handshake errors) and finalize the qpair as DISCONNECTED so every
 * framework waiter observes a terminal state. */
static void
nvme_hcomm_connect_unwind(struct spdk_nvme_qpair *qpair, int rc)
{
	struct nvme_hcomm_qpair *hqpair = nvme_hcomm_qpair(qpair);

	hqpair->connect_result = rc;
	nvme_hcomm_conn_state_store(hqpair, NVME_HCOMM_CONN_IDLE);
	/* Skip the resource release when the watchdog abandoned this
	 * connect with the worker possibly still inside the library - the
	 * reconnect sweep (which joins the worker first) owns them. */
	if (!hqpair->connect_abandoned) {
		nvme_hcomm_qpair_connect_cleanup(hqpair);
	}
	SPDK_ERRLOG("connect to %s:%u (qid=%u) failed: rc=%d\n",
		    hqpair->hctrlr->target_addr, hqpair->hctrlr->target_port,
		    qpair->id, rc);
	nvme_transport_ctrlr_disconnect_qpair_done(qpair);
}

/*
 * Async connect phases 2+3 (poll thread) - see the block comment above.
 * Returns 0 when the qpair just became CONNECTED, -EAGAIN while in
 * progress, and -EIO once the failure has been fully unwound here.
 *
 * The in_connect_poll guard mirrors rdma/tcp: fabric connect poll
 * re-enters spdk_nvme_qpair_process_completions on this qpair, and the
 * nested call must only reap CQEs, never re-enter this state machine.
 */
static int
nvme_hcomm_qpair_connect_poll(struct spdk_nvme_qpair *qpair)
{
	struct nvme_hcomm_qpair *hqpair = nvme_hcomm_qpair(qpair);
	int rc;

	if (hqpair->in_connect_poll) {
		return -EAGAIN;
	}
	hqpair->in_connect_poll = true;

	/* Connect watchdog: the whole async connect (worker handshake +
	 * fabrics CONNECT) must finish within connect_watchdog_ms of the
	 * spawn, else this poll loop would return -EAGAIN forever (a
	 * worker stuck inside the library, or a fabric layer that never
	 * times out when fabrics_connect_timeout_us is 0). */
	if (spdk_unlikely(spdk_get_ticks() >= hqpair->connect_deadline)) {
		SPDK_ERRLOG("connect watchdog expired after %u ms (qid=%u, phase=%d)\n",
			    g_hcomm.connect_watchdog_ms, qpair->id,
			    (int)nvme_hcomm_conn_state_load(hqpair));
		hqpair->in_connect_poll = false;
		if (nvme_hcomm_conn_state_load(hqpair) == NVME_HCOMM_CONN_WORKING) {
			/* The worker still owns the handshake resources -
			 * publish a terminal state and finalize WITHOUT
			 * touching them (cleaning underneath a live worker
			 * races the library).  The reconnect sweep joins the
			 * worker first and releases whatever it left behind. */
			hqpair->connect_result = -ETIMEDOUT;
			hqpair->connect_abandoned = true;
			nvme_hcomm_conn_state_store(hqpair, NVME_HCOMM_CONN_FAILED);
			nvme_transport_ctrlr_disconnect_qpair_done(qpair);
			return -EIO;
		}
		nvme_hcomm_connect_unwind(qpair, -ETIMEDOUT);
		return -EIO;
	}

	{
		enum nvme_hcomm_connect_state st = nvme_hcomm_conn_state_load(hqpair);

		/* DFX: connect_poll runs at poll rate - log phase TRANSITIONS
		 * only, not every call.  The poll thread spends the WORKING
		 * window doing exactly this: one acquire load + -EAGAIN,
		 * staying free for the framework's other work. */
		if (spdk_unlikely(st != hqpair->debug_last_phase)) {
			SPDK_DEBUGLOG(nvme_hcomm, "connect_poll qid=%u: phase %d -> %d (+%u ms)\n",
				      qpair->id, (int)hqpair->debug_last_phase, (int)st,
				      nvme_hcomm_connect_elapsed_ms(hqpair));
			hqpair->debug_last_phase = st;
		}

		switch (st) {
		case NVME_HCOMM_CONN_IDLE:
		case NVME_HCOMM_CONN_WORKING:
			rc = -EAGAIN;    /* worker still handshaking / not started */
			break;

		case NVME_HCOMM_CONN_LINK_UP:
			rc = nvme_fabric_qpair_connect_async(qpair, hqpair->cqe_depth);
			if (rc == 0) {
				SPDK_DEBUGLOG(nvme_hcomm, "connect_poll qid=%u: fabrics CONNECT submitted (admin=%d)\n",
					      qpair->id, nvme_qpair_is_admin_queue(qpair));
				nvme_hcomm_conn_state_store(hqpair, NVME_HCOMM_CONN_FABRIC_SENT);
				rc = -EAGAIN;
			}
			break;

		case NVME_HCOMM_CONN_FABRIC_SENT:
			rc = nvme_fabric_qpair_connect_poll(qpair);
			if (rc == 0) {
				/* admin CONNECT reply also carried cntlid (stored by
				 * the fabric layer); the qpair is now usable */
				SPDK_DEBUGLOG(nvme_hcomm, "connect_poll qid=%u: fabrics CONNECT complete -> CONNECTED "
					      "(cntlid=0x%x, +%u ms)\n", qpair->id,
					      hqpair->hctrlr->ctrlr.cntlid,
					      nvme_hcomm_connect_elapsed_ms(hqpair));
				nvme_qpair_set_state(qpair, NVME_QPAIR_CONNECTED);
				nvme_hcomm_conn_state_store(hqpair, NVME_HCOMM_CONN_IDLE);
			}
			break;

		default:            /* NVME_HCOMM_CONN_FAILED */
			rc = hqpair->connect_result ? hqpair->connect_result : -EIO;
			break;
		}
	}

	hqpair->in_connect_poll = false;
	if (hqpair->connect_abandoned) {
		/* Watchdog force-failed this connect with the worker still
		 * inside the library: the resources stay with the worker,
		 * the reconnect sweep (after join) releases them - never
		 * unwind underneath a live worker. */
		return rc;
	}
	if (rc == 0 || rc == -EAGAIN) {
		return rc;
	}

	nvme_hcomm_connect_unwind(qpair, rc);
	return -EIO;
}

static int
nvme_hcomm_ctrlr_connect_qpair(struct spdk_nvme_ctrlr *ctrlr, struct spdk_nvme_qpair *qpair)
{
	struct nvme_hcomm_ctrlr *hctrlr = nvme_hcomm_ctrlr(ctrlr);
	struct nvme_hcomm_qpair *hqpair = nvme_hcomm_qpair(qpair);
	int rc;

	SPDK_DEBUGLOG(nvme_hcomm, "connect_qpair: %s:%u (qid=%u)\n",
		      hctrlr->target_addr, hctrlr->target_port, qpair->id);

	if (!hctrlr->endpoint_created) {
		SPDK_ERRLOG("Endpoint not created\n");
		return -1;
	}

	/* Guard against double-connect while a worker is still running. */
	if (nvme_hcomm_conn_state_load(hqpair) == NVME_HCOMM_CONN_WORKING) {
		return 0;
	}

	/* Reap a worker that already published a terminal state but has not
	 * exited yet (offline-mode aclrtResetDevice tail): pthread_create
	 * below must never overwrite a live, unjoined connect_worker. */
	if (hqpair->worker_started) {
		SPDK_DEBUGLOG(nvme_hcomm, "qid=%u: reaping previous connect worker "
			      "(join; it published a terminal state but may still be "
			      "inside the aclrtResetDevice tail)\n", qpair->id);
		pthread_join(hqpair->connect_worker, NULL);
		hqpair->worker_started = false;
	}

	/* Stale-resource sweep: a disconnect that gave up on a stuck worker,
	 * or a terminal LINK_UP nobody unwound, can leave the handshake
	 * resources (channel, thread, SQ/CQE registrations) behind; simply
	 * re-registering on top would orphan the old handles on the endpoint
	 * forever (every later channel re-exchanges them).  Start clean -
	 * but never sweep while the qpair is actually CONNECTED (its
	 * resources are live). */
	{
		enum nvme_hcomm_connect_state stale = nvme_hcomm_conn_state_load(hqpair);

		if (nvme_qpair_get_state(qpair) != NVME_QPAIR_CONNECTED &&
		    (hqpair->sq_lmem_handle != NULL || hqpair->cqe_lmem_handle != NULL ||
		     hqpair->channel_created || hqpair->thread_allocated ||
		     stale == NVME_HCOMM_CONN_LINK_UP || stale == NVME_HCOMM_CONN_FABRIC_SENT)) {
			SPDK_WARNLOG("connect_qpair (qid=%u): sweeping stale connect "
				     "resources (connect_state=%d)\n", qpair->id, (int)stale);
			if (stale == NVME_HCOMM_CONN_FABRIC_SENT) {
				/* Let the fabric layer complete and free its
				 * in-flight CONNECT before the channel goes
				 * away - same ordering as the disconnect
				 * FABRIC_SENT branch. */
				nvme_hcomm_connect_drain_fabric(qpair);
			}
			nvme_hcomm_qpair_connect_cleanup(hqpair);
			nvme_hcomm_conn_state_store(hqpair, NVME_HCOMM_CONN_IDLE);
		}
	}

	hqpair->connect_result = 0;
	hqpair->connect_abandoned = false;
	hqpair->connect_start_ticks = spdk_get_ticks();
	hqpair->connect_deadline = hqpair->connect_start_ticks +
				    (uint64_t)g_hcomm.connect_watchdog_ms * spdk_get_ticks_hz() / 1000;
	hqpair->debug_last_phase = NVME_HCOMM_CONN_IDLE;
	nvme_hcomm_conn_state_store(hqpair, NVME_HCOMM_CONN_WORKING);

	/* Joinable worker; reaped by delete_io_qpair if the qpair is torn
	 * down before the connect finishes. */
	rc = pthread_create(&hqpair->connect_worker, NULL,
			    nvme_hcomm_connect_worker, hqpair);
	if (rc != 0) {
		SPDK_ERRLOG("Failed to spawn connect worker: %s\n", strerror(rc));
		hqpair->connect_result = -rc;
		nvme_hcomm_conn_state_store(hqpair, NVME_HCOMM_CONN_FAILED);
		return -rc;
	}
	hqpair->worker_started = true;

	SPDK_DEBUGLOG(nvme_hcomm, "qid=%u: connect worker spawned - qpair stays CONNECTING, "
		      "this thread returns to the framework and keeps driving phases 2/3 "
		      "(fabrics CONNECT) via qpair polling: adminq ctrlr poller, sync-io "
		      "busy-wait or poll group\n", qpair->id);

	/* The qpair stays CONNECTING; the poll path finishes the job
	 * (fabrics CONNECT -> CONNECTED).  Returning 0 makes the framework
	 * either poll the adminq (WAIT_FOR_CONNECT_ADMINQ) or busy-wait in
	 * process_completions (sync io qpairs). */
	return 0;
}

static void
nvme_hcomm_ctrlr_disconnect_qpair(struct spdk_nvme_ctrlr *ctrlr, struct spdk_nvme_qpair *qpair)
{
	struct nvme_hcomm_qpair *hqpair = nvme_hcomm_qpair(qpair);
	enum nvme_hcomm_connect_state st = nvme_hcomm_conn_state_load(hqpair);

	SPDK_DEBUGLOG(nvme_hcomm, "disconnect_qpair: (qid=%u)\n",
		      qpair ? qpair->id : 0xFFFF);

	if (st == NVME_HCOMM_CONN_WORKING) {
		/* The worker owns the hcomm resources until it publishes a
		 * terminal phase (LINK_UP after a successful handshake,
		 * FAILED after its own error-path unwind).  Wait for that
		 * publish - bounded by the socket timeout, which also bounds
		 * every blocking step inside the handshake - then unwind
		 * HERE.  The disconnector is thus the sole cleanup owner once
		 * the worker goes terminal; the old abort-flag handshake (and
		 * its missed-flag race, where neither side unwound and the
		 * resources leaked with connect_state stuck at LINK_UP) is
		 * gone. */
		uint64_t deadline = spdk_get_ticks() +
				    (uint64_t)g_hcomm.socket_timeout_ms * spdk_get_ticks_hz() / 1000;
		uint32_t waited_ms = 0;

		SPDK_DEBUGLOG(nvme_hcomm, "disconnect qid=%u: worker still handshaking, "
			      "waiting (bounded, %u ms)\n", qpair->id,
			      g_hcomm.socket_timeout_ms);
		while (nvme_hcomm_conn_state_load(hqpair) == NVME_HCOMM_CONN_WORKING &&
		       spdk_get_ticks() < deadline) {
			spdk_delay_us(1000);
			waited_ms++;
			if (waited_ms % 1000 == 0) {
				/* Once-per-second breadcrumb while waiting for
				 * the worker's terminal publish. */
				SPDK_DEBUGLOG(nvme_hcomm, "disconnect qid=%u: still waiting "
					      "for connect worker (%u ms elapsed)\n",
					      qpair->id, waited_ms);
			}
		}
		if (nvme_hcomm_conn_state_load(hqpair) == NVME_HCOMM_CONN_WORKING) {
			/* Stuck worker: it still owns the resources - cleaning
			 * underneath it would race the library.  Leave them
			 * for the stale-resource sweep on the next connect. */
			SPDK_ERRLOG("connect worker (qid=%u) still running after "
				    "%u ms; deferring resource cleanup to the "
				    "next connect\n", qpair->id,
				    g_hcomm.socket_timeout_ms);
		} else {
			SPDK_DEBUGLOG(nvme_hcomm, "disconnect qid=%u: worker published terminal, "
				      "unwinding here\n", qpair->id);
			nvme_hcomm_qpair_connect_cleanup(hqpair);
			nvme_hcomm_conn_state_store(hqpair, NVME_HCOMM_CONN_IDLE);
		}
	} else if (st == NVME_HCOMM_CONN_FABRIC_SENT) {
		SPDK_DEBUGLOG(nvme_hcomm, "disconnect qid=%u: draining in-flight fabrics CONNECT\n",
			      qpair->id);
		nvme_hcomm_connect_drain_fabric(qpair);
		nvme_hcomm_qpair_connect_cleanup(hqpair);
		nvme_hcomm_conn_state_store(hqpair, NVME_HCOMM_CONN_IDLE);
	} else {
		/* LINK_UP (never connected), FAILED (worker already unwound)
		 * or IDLE (connected earlier / already reset). */
		if (hqpair->connect_abandoned) {
			/* Watchdog abandoned this connect with the worker
			 * possibly still inside the library - resources are
			 * not ours to release; the reconnect sweep (after
			 * join) owns them. */
			SPDK_DEBUGLOG(nvme_hcomm, "disconnect qid=%u: abandoned connect, "
				      "deferring cleanup to the next connect\n", qpair->id);
		} else {
			SPDK_DEBUGLOG(nvme_hcomm, "disconnect qid=%u: unwinding (connect_state=%d)\n",
				      qpair->id, (int)st);
			nvme_hcomm_qpair_connect_cleanup(hqpair);
		}
		nvme_hcomm_conn_state_store(hqpair, NVME_HCOMM_CONN_IDLE);
	}

	/* Finalize: abort queued requests + DISCONNECTED (pcie/tcp pattern);
	 * this also unblocks reconnect, which requires DISCONNECTED. */
	nvme_transport_ctrlr_disconnect_qpair_done(qpair);
}

static int
nvme_hcomm_ctrlr_get_memory_domains(const struct spdk_nvme_ctrlr *ctrlr,
				    struct spdk_memory_domain **domains, int array_size)
{
	return 0;
}

static void
nvme_hcomm_qpair_abort_reqs(struct spdk_nvme_qpair *qpair, uint32_t dnr)
{
	struct nvme_hcomm_qpair *hqpair = nvme_hcomm_qpair(qpair);
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	struct spdk_nvme_cpl cpl;
	uint32_t i = 0;

	cpl.sqid = qpair->id;
	cpl.status.sc = SPDK_NVME_SC_ABORTED_SQ_DELETION;
	cpl.status.sct = SPDK_NVME_SCT_GENERIC;
	cpl.status.dnr = dnr;

	/* Disconnect first so the TGT stops DMAing into cqe_buf while we
	 * reap the outstanding requests below (mirrors nvme_rdma). */
	if (nvme_qpair_get_state(qpair) > NVME_QPAIR_DISCONNECTING &&
	    nvme_qpair_get_state(qpair) != NVME_QPAIR_DESTROYING) {
		nvme_ctrlr_disconnect_qpair(qpair);
	}

	/* Reap every in-flight request with an ABORTED completion.  Traverse
	 * active_cids[] with swap-remove (do not advance i after removal). */
	while (i < hqpair->num_active) {
		struct nvme_request *req;
		uint32_t cid = hqpair->active_cids[i];

		if (hqpair->req_for_cid[cid] == NULL) {
			nvme_hcomm_mem_map_release_all(hctrlr->mem_map,
						   &hqpair->xlation_for_cid[cid]);
			nvme_hcomm_qpair_remove_active(hqpair, i);
			continue;
		}

		req = hqpair->req_for_cid[cid];
		hqpair->req_for_cid[cid] = NULL;

		/* Drop the mem_map reference before the completion callback
		 * (it may resubmit synchronously). */
		nvme_hcomm_mem_map_release_all(hctrlr->mem_map,
					   &hqpair->xlation_for_cid[cid]);

		nvme_complete_request(req->cb_fn, req->cb_arg, qpair, req, &cpl);

		nvme_hcomm_qpair_free_cid(hqpair, cid);
		nvme_hcomm_qpair_remove_active(hqpair, i);
	}
}

static int
nvme_hcomm_qpair_reset(struct spdk_nvme_qpair *qpair)
{
	return 0;
}

/* Write an SGL descriptor into the SQE DPTR location: cmd.dptr.sgl1
 * (standard NVMe offset 24) - the legacy byte-identical layout. */
static void
nvme_hcomm_write_dptr(uint8_t *sqe, const struct spdk_nvme_sgl_descriptor *desc)
{
	((struct nvme_hcomm_sqe_96b *)sqe)->cmd.dptr.sgl1 = *desc;
}

/* Tail SGL descriptor slot i (the walk fills the slots in order). */
static struct spdk_nvme_sgl_descriptor *
nvme_hcomm_tail_desc(uint8_t *sqe, uint32_t index)
{
	return &((struct nvme_hcomm_sqe_96b *)sqe)->sgl[index];
}

/*
 * Build the DPTR keyed SGL descriptor for a CONTIG payload: register
 * the buffer through mem_map (cached) and write one keyed descriptor
 * {tokenId, addr, length} into the SQE DPTR location - cmd.dptr.sgl1
 * (standard NVMe) - the
 * legacy byte-identical layout.  Mirrors nvme_rdma_build_contig_request.
 *
 * The acquired mem_map reference is recorded on xlation_for_cid[cid]
 * (exactly one) before returning 0, so the caller releases it on any
 * later failure path.
 */
static int
nvme_hcomm_build_contig_request(struct nvme_hcomm_qpair *hqpair,
				struct nvme_request *req, uint8_t *sqe, uint32_t cid)
{
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	struct nvme_hcomm_xlation_refs *xrefs = &hqpair->xlation_for_cid[cid];
	struct nvme_hcomm_mem_translation xlate;
	struct spdk_nvme_sgl_descriptor sgl = {};
	void *payload = (void *)((uintptr_t)req->payload.contig_or_cb_arg +
				 req->payload_offset);
	int rc;

	if (spdk_unlikely(req->payload_size > NVME_HCOMM_MAX_KEYED_SGL_LENGTH)) {
		SPDK_ERRLOG("contiguous payload %u exceeds max keyed SGL "
			    "block size %u for %s:%u\n",
			    req->payload_size, NVME_HCOMM_MAX_KEYED_SGL_LENGTH,
			    hctrlr->target_addr, hctrlr->target_port);
		return -EINVAL;
	}

	rc = nvme_hcomm_get_memory_translation(hctrlr->mem_map, payload,
					       req->payload_size, &xlate);
	if (rc != 0) {
		SPDK_ERRLOG("memory translation failed for %s:%u "
			    "(buf=%p len=%u)\n",
			    hctrlr->target_addr, hctrlr->target_port,
			    payload, req->payload_size);
		return -EFAULT;
	}

	xrefs->refs[0].held = true;
	xrefs->refs[0].entry_idx = xlate.entry_idx;
	xrefs->refs[0].handle = xlate.handle;
	xrefs->count = 1;

	sgl.keyed.type    = SPDK_NVME_SGL_TYPE_KEYED_DATA_BLOCK;
	sgl.keyed.subtype = SPDK_NVME_SGL_SUBTYPE_ADDRESS;
	sgl.keyed.key     = xlate.tokenId;
	sgl.keyed.length  = xlate.length;
	sgl.address       = (uint64_t)(uintptr_t)xlate.addr;

	/* PSDT was already set on req->cmd before the fill helper ran,
	 * so the SQE header is correct - no re-copy of DW0-3 needed. */
	nvme_hcomm_write_dptr(sqe, &sgl);

	return 0;
}

/* Fetch the next payload segment from the SGL iterator, skipping
 * zero-length iovec entries (rdma emits a 0-length descriptor and
 * continues; we omit them, but they still consume a walked slot so a
 * runaway iterator cannot spin forever).  The callback error value is
 * not pinned by the contract, so failures normalize to -EINVAL.
 * Returns 0 with a non-zero *seg_len, 1 for a skipped entry, or
 * -EINVAL on callback failure. */
static int
nvme_hcomm_sgl_next_segment(struct nvme_request *req, uint32_t remaining,
			    void **seg_addr, uint32_t *seg_len,
			    uint32_t *walked)
{
	int rc;

	rc = req->payload.next_sge_fn(req->payload.contig_or_cb_arg,
				      seg_addr, seg_len);
	if (rc != 0) {
		return -EINVAL;
	}
	(*walked)++;
	*seg_len = spdk_min(remaining, *seg_len);
	if (*seg_len == 0) {
		return 1;
	}
	return 0;
}

/* Register one segment through mem_map, record its reference on
 * xlation_for_cid, and fill tail slot `index` with the keyed
 * descriptor {tokenId, addr, length}.  A segment larger than the
 * 24-bit keyed length field is rejected outright: it would silently
 * truncate the high bits (same guard as nvme_rdma's
 * NVME_RDMA_MAX_KEYED_SGL_LENGTH). */
static int
nvme_hcomm_sgl_add_segment(struct nvme_hcomm_qpair *hqpair, uint8_t *sqe,
			   struct nvme_hcomm_xlation_refs *xrefs,
			   uint32_t index, void *seg_addr, uint32_t seg_len)
{
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	struct nvme_hcomm_mem_translation xlate;
	struct spdk_nvme_sgl_descriptor *desc;
	int rc;

	if (spdk_unlikely(seg_len > NVME_HCOMM_MAX_KEYED_SGL_LENGTH)) {
		SPDK_ERRLOG("SGL segment length %u exceeds max keyed "
			    "SGL block size %u (seg %u)\n",
			    seg_len, NVME_HCOMM_MAX_KEYED_SGL_LENGTH,
			    index + 1);
		return -EINVAL;
	}

	rc = nvme_hcomm_get_memory_translation(hctrlr->mem_map, seg_addr,
					       seg_len, &xlate);
	if (rc != 0) {
		SPDK_ERRLOG("memory translation failed for %s:%u "
			    "(seg=%p len=%u)\n",
			    hctrlr->target_addr, hctrlr->target_port,
			    seg_addr, seg_len);
		return -EFAULT;
	}

	xrefs->refs[index].held = true;
	xrefs->refs[index].entry_idx = xlate.entry_idx;
	xrefs->refs[index].handle = xlate.handle;
	xrefs->count = index + 1;

	desc = nvme_hcomm_tail_desc(sqe, index);
	memset(desc, 0, sizeof(*desc));
	desc->keyed.type    = SPDK_NVME_SGL_TYPE_KEYED_DATA_BLOCK;
	desc->keyed.subtype = SPDK_NVME_SGL_SUBTYPE_ADDRESS;
	desc->keyed.key     = xlate.tokenId;
	desc->keyed.length  = xlate.length;
	desc->address       = (uint64_t)(uintptr_t)xlate.addr;
	return 0;
}

/* Finalize the DPTR after the walk.  A single segment collapses to
 * the legacy DPTR descriptor (an old TGT sees the byte-identical
 * layout; the tail slot is simply not sent).  Multiple segments carry
 * the count in a LAST_SEGMENT+OFFSET descriptor whose 24-bit length =
 * N*16 - the single count encoding (standard NVMe-oF "SGL in capsule"
 * convention, same as nvme_rdma build_sgl_request); the TGT derives
 * N = length/16.  Returns the number of tail descriptors to send. */
static uint32_t
nvme_hcomm_sgl_set_count(struct nvme_hcomm_qpair *hqpair, uint8_t *sqe,
			 uint32_t count)
{
	struct spdk_nvme_sgl_descriptor *tail;
	struct spdk_nvme_sgl_descriptor ls = {};

	if (count == 1) {
		tail = nvme_hcomm_tail_desc(sqe, 0);
		nvme_hcomm_write_dptr(sqe, tail);
		return 0;
	}

	ls.unkeyed.type    = SPDK_NVME_SGL_TYPE_LAST_SEGMENT;
	ls.unkeyed.subtype = SPDK_NVME_SGL_SUBTYPE_OFFSET;
	ls.unkeyed.length  = count * sizeof(ls);
	ls.address         = 0;
	nvme_hcomm_write_dptr(sqe, &ls);
	return count;
}

/* Walk the SGL payload segments (reset_sgl_fn/next_sge_fn, mirrors
 * nvme_rdma_build_sgl_request's loop), filling tail slots 0..count-1
 * via nvme_hcomm_sgl_add_segment().  `walked` counts next_sge calls
 * (zero-length entries included) and is capped by max_sgl so the
 * negotiated limit is enforced, not just reported.  Returns 0 and the
 * descriptor count, or a negative errno. */
static int
nvme_hcomm_sgl_walk(struct nvme_hcomm_qpair *hqpair, struct nvme_request *req,
		    uint8_t *sqe, struct nvme_hcomm_xlation_refs *xrefs,
		    uint32_t max_sgl, uint32_t *count)
{
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	uint32_t remaining = req->payload_size;
	uint32_t walked = 0;
	int rc = 0;

	req->payload.reset_sgl_fn(req->payload.contig_or_cb_arg,
				  req->payload_offset);
	while (remaining > 0) {
		void *seg_addr;
		uint32_t seg_len;

		if (walked == max_sgl) {
			SPDK_ERRLOG("SGL payload exceeds %u segments "
				    "(opc=0x%x, size=%u) for %s:%u\n",
				    max_sgl, req->cmd.opc,
				    req->payload_size, hctrlr->target_addr,
				    hctrlr->target_port);
			return -EINVAL;
		}

		rc = nvme_hcomm_sgl_next_segment(req, remaining, &seg_addr,
						 &seg_len, &walked);
		if (rc < 0) {
			return rc;
		}
		if (rc > 0) {
			continue;
		}

		rc = nvme_hcomm_sgl_add_segment(hqpair, sqe, xrefs,
						*count, seg_addr, seg_len);
		if (rc != 0) {
			return rc;
		}
		remaining -= seg_len;
		(*count)++;
	}

	return 0;
}

/*
 * Build the multi-SGL tail array from an SGL payload (mirrors
 * nvme_rdma_build_sgl_request): walk the segments, register each one
 * through mem_map, fill the tail slots, then finalize the DPTR via
 * nvme_hcomm_sgl_set_count().  Every acquired mem_map reference is
 * recorded on xlation_for_cid[cid] (one per segment) and must be
 * released by the caller on failure or on reap (process_completions /
 * abort_reqs / admin_qpair_abort_aers).
 *
 * On success *nsgl carries the number of tail descriptors (0 for the
 * collapsed single-segment case) so the caller computes the variable
 * SEND length (base + N*16).  The walk cap is the negotiated
 * ctrlr->max_sges; 0 (a private TGT that reports nothing) falls back
 * to the lockstep default, the full tail array. */
static int
nvme_hcomm_build_sgl_request(struct nvme_hcomm_qpair *hqpair,
			     struct nvme_request *req, uint8_t *sqe,
			     uint32_t cid, uint32_t *nsgl)
{
	struct nvme_hcomm_xlation_refs *xrefs = &hqpair->xlation_for_cid[cid];
	uint32_t max_sgl = hqpair->qpair.ctrlr->max_sges;
	uint32_t count = 0;
	int rc;

	if (max_sgl == 0 || max_sgl > NVME_HCOMM_MAX_SGL) {
		max_sgl = NVME_HCOMM_MAX_SGL;
	}

	rc = nvme_hcomm_sgl_walk(hqpair, req, sqe, xrefs, max_sgl, &count);
	if (rc != 0) {
		*nsgl = 0;
		return rc;
	}

	*nsgl = nvme_hcomm_sgl_set_count(hqpair, sqe, count);
	return 0;
}

/*
 * ========== INI-Side Data Plane: Submit & Complete ==========
 *
 * HOST side operations for the INI data plane.  SQ is a cid-indexed
 * array (sq_buf[cid], mirroring nvme_rdma's cmds[rdma_req->id]):
 * allocating a cid == allocating an SQE slot, so there is no separate
 * ring / doorbell / full-check.
 *
 * ======= submit_request() — CMD submission =======
 *
 *   Step 1: Reserve cid (== reserve an SQE slot)
 *     - cid = alloc_cid();  cid ∈ [0, cqe_depth)
 *     - req_for_cid[cid] = req; active_cids[] append
 *
 *   Step 2: Build the SQE at sq_buf[cid]
 *     - Copy req->cmd into sq_buf[cid]
 *     - cmd.cid (DW0) = cid (TGT uses it to locate cqe_buf[cid])
 *     - DPTR = Keyed SGL {key=tokenId, address=user_buf, length}
 *       (register user buf via mem_map if not cached)
 *
 *   Step 3: Write barrier + forward
 *     - spdk_wmb() so SQE is visible before forwarding
 *     - No bounce-copy (zero-copy, TGT accesses user_buf directly)
 *
 *   Step 4: Forward the SQE to the TGT via HcommSendOnThread
 *     - HcommSendOnThread(thread, channel, sqe, len) — bilateral
 *       SEND primitive: the Device-side A5 transport executes it and
 *       delivers the SQE bytes directly to the TGT (the channel peer).
 *
 * ======= process_completions() — CQE from TGT =======
 *
 *   Step 1: TGT processes the SQE
 *     - Reads DPTR keyed SGL (tokenId + user_buf addr + length)
 *     - Reads cmd.cid (DW0) to locate cqe_buf[cid]
 *     - For READ: TGT RDMA-writes data to user_buf directly (zero-copy)
 *     - For WRITE: TGT RDMA-reads data from user_buf directly (zero-copy)
 *
 *   Step 2: TGT writes CQE directly into cqe_buf[cid]
 *     - RDMA-write 16B spdk_nvme_cpl into cqe_buf[cid].cpl
 *     - Set cqe_buf[cid].padding[0]=1 (valid flag, with wmb)
 *
 *   Step 3: HOST polls and completes (scan active_cids[])
 *     - For each in-flight cid:
 *       a. Check cqe_buf[cid].padding[0] (valid flag) with spdk_rmb()
 *       b. If set, read cqe_buf[cid].cpl (16B spdk_nvme_cpl)
 *       c. assert(cpl.cid == cid) — defensive check vs positional index
 *       d. No bounce-copy (data already in user_buf — zero-copy)
 *       e. nvme_complete_request(req, &cqe_buf[cid].cpl)
 *       f. free cid, memset cqe_buf[cid], remove from active list
 *     - Traversal is O(in-flight) via the active_cids swap-remove list
 */

/* Reserve a cid for the request: allocate from the free pool, tag the
 * command, and append to the in-flight list.  cid is allocated by the
 * transport itself (standard SPDK pattern, same as rdma/tcp/pcie):
 * the range is [0, cqe_depth) and cid directly indexes cqe_buf /
 * sq_buf / req_for_cid, keeping cid one-to-one with the SQE/CQE
 * slots.  Allocating a cid == allocating an SQE slot (like RDMA's
 * free_reqs): no separate ring / doorbell / full-check is needed.
 * Returns the cid, or UINT32_MAX when the pool is empty. */
static uint32_t
nvme_hcomm_qpair_reserve_cid(struct nvme_hcomm_qpair *hqpair,
			     struct nvme_request *req)
{
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	uint32_t cid;

	cid = nvme_hcomm_qpair_alloc_cid(hqpair);
	if (cid == UINT32_MAX) {
		SPDK_ERRLOG("no free cid for %s:%u\n",
			    hctrlr->target_addr, hctrlr->target_port);
		return UINT32_MAX;
	}
	req->cmd.cid = (uint16_t)cid;
	hqpair->req_for_cid[cid] = req;
	/* Append so process_completions only walks active cids. */
	assert(hqpair->num_active < hqpair->cqe_depth);
	hqpair->active_cids[hqpair->num_active++] = cid;
	return cid;
}

/* Build the SQE at the cid-th slot.  PSDT is set on req->cmd BEFORE
 * the fill helper runs so it lands in the SQE in one pass (no re-copy
 * of DW0-3); the fill helper zeroes the whole slot (MPTR included),
 * and the standard DW0 cmd.cid carries the cid (set in reserve_cid),
 * which is what the TGT uses to locate cqe_buf[cid].  Returns the SQE
 * pointer. */
static uint8_t *
nvme_hcomm_qpair_fill_sqe(struct nvme_hcomm_qpair *hqpair,
			  struct nvme_request *req, uint32_t cid)
{
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	struct spdk_nvme_qpair *qpair = &hqpair->qpair;
	uint8_t *sqe = nvme_hcomm_qpair_sqe(hqpair, cid);
	struct nvme_hcomm_sqe_96b *sqe96 = (struct nvme_hcomm_sqe_96b *)sqe;

	if (req->payload_size > 0) {
		req->cmd.psdt = SPDK_NVME_PSDT_SGL_MPTR_CONTIG;
	}

	/* 96B SQE: standard NVMe cmd + reserved 24B + routing(DW22) + token(DW23). */
	nvme_hcomm_sqe_96b_fill(sqe96, &req->cmd,
				hctrlr->ctrlr.cntlid, qpair->id,
				NVME_HCOMM_INI_JFR_TOKEN);
	return sqe;
}

/* Undo a cid reservation on a submit failure path: drop any mem_map
 * references the SQE acquired and recycle the cid.  The failed cid is
 * the most recent active_cids append (submit runs single-threaded on
 * the reactor, nothing appends in between), so the list truncates. */
static void
nvme_hcomm_qpair_unsubmit_cid(struct nvme_hcomm_qpair *hqpair, uint32_t cid)
{
	nvme_hcomm_mem_map_release_all(hqpair->hctrlr->mem_map,
				       &hqpair->xlation_for_cid[cid]);
	assert(hqpair->num_active > 0 &&
	       hqpair->active_cids[hqpair->num_active - 1] == cid);
	hqpair->num_active--;
	hqpair->req_for_cid[cid] = NULL;
	nvme_hcomm_qpair_free_cid(hqpair, cid);
}

/* Build the payload keyed SGL (common to both formats, zero-copy):
 * register the CONTIG buffer or walk the SGL segments through mem_map
 * and write the descriptors.  A recycled cid must carry no stale
 * references - every path that frees a cid releases them first; the
 * assert compiles out under NDEBUG, so also release stragglers
 * (no-op when count == 0) as defense-in-depth. */
static int
nvme_hcomm_build_payload(struct nvme_hcomm_qpair *hqpair,
			 struct nvme_request *req, uint8_t *sqe,
			 uint32_t cid, uint32_t *nsgl)
{
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	struct nvme_hcomm_xlation_refs *xrefs = &hqpair->xlation_for_cid[cid];

	assert(xrefs->count == 0);
	nvme_hcomm_mem_map_release_all(hctrlr->mem_map, xrefs);

	/* CONTIG -> one keyed descriptor in the DPTR (build_contig);
	 * SGL -> per-segment walk into the tail array (build_sgl).  An
	 * SGL payload must never be treated as one contiguous buffer
	 * (contig_or_cb_arg holds the SGL callbacks' cb_arg). */
	if (nvme_payload_type(&req->payload) == NVME_PAYLOAD_TYPE_CONTIG) {
		return nvme_hcomm_build_contig_request(hqpair, req, sqe, cid);
	}

	if (req->payload.next_sge_fn == NULL) {
		/* nvme_payload_type() returns SGL iff reset_sgl_fn is set;
		 * a missing next_sge_fn is a malformed payload. */
		SPDK_ERRLOG("SGL payload without next_sge_fn (opc=0x%x) for %s:%u\n",
			    req->cmd.opc, hctrlr->target_addr,
			    hctrlr->target_port);
		return -EINVAL;
	}

	return nvme_hcomm_build_sgl_request(hqpair, req, sqe, cid, nsgl);
}

/* Forward the SQE to the TGT via the HComm bilateral SEND primitive:
 * write barrier first so the SQE is visible before forwarding, then
 * a variable-length send (base SQE + nsgl tail descriptors; the slot
 * stride stays fixed, only the sent bytes grow).  No bounce-copy -
 * the TGT accesses the user buffer directly via keyed SGL (zero-copy
 * fast path, identical to nvme_rdma).
 *
 * A send failure is FATAL to the request and must be returned as
 * -EIO, never -EAGAIN (the only legit -EAGAIN in the submit path is
 * cid-pool exhaustion in reserve_cid, mirroring nvme_rdma's req_get).
 * The base layer converts a transport -EAGAIN into "queue the request
 * and report success" (nvme_qpair_submit_request), which for the
 * fabrics CONNECT (submitted on reserved_req from
 * nvme_fabric_qpair_connect_async) means reporting success and then
 * waiting forever for a CQE that was never sent: resubmission is
 * completion-driven (no CQEs arrive), and the failure reason set
 * below is only acted upon once the qpair reaches ENABLED.  RDMA
 * follows the same contract - its only -EAGAIN is the request-pool
 * miss; a post failure completes the request with an error instead.
 *
 * On failure the caller recycles the cid, and the qpair is marked
 * locally failed so the base layer disconnects and reconnects it
 * (nvme_qpair_check_enabled sees a non-NONE reason on the next
 * submit).  RDMA detects a broken qpair via async WC/CM events; HCOMM
 * has no event source, so a send failure is the only signal - without
 * this, base would requeue onto a dead channel forever. */
/*
 * Launch the HixlBatchSend AICPU kernel with a single-batch param.  The
 * kernel runs async on the AI CPU and calls HcommSendWithImmOnThread +
 * HcommThreadJoin internally, so the WQE is enqueued and flushed before the
 * kernel returns.  Fire-and-forget from the host's view: the NVMe completion
 * still arrives via cqe_buf written by the TGT.
 */
static int
nvme_hcomm_launch_batch_send(struct nvme_hcomm_batch_send_param *param)
{
	aclrtArgsHandle args_handle = NULL;
	aclrtParamHandle param_handle = NULL;
	aclError rc;

	rc = aclrtKernelArgsInit(g_hcomm.batch_send_func, &args_handle);
	if (rc != ACL_SUCCESS) {
		SPDK_ERRLOG("aclrtKernelArgsInit failed: %d\n", rc);
		return -1;
	}
	rc = aclrtKernelArgsAppend(args_handle, param, sizeof(*param), &param_handle);
	if (rc != ACL_SUCCESS) {
		SPDK_ERRLOG("aclrtKernelArgsAppend failed: %d\n", rc);
		aclrtKernelArgsFinalize(args_handle);
		return -1;
	}
	rc = aclrtKernelArgsFinalize(args_handle);
	if (rc != ACL_SUCCESS) {
		SPDK_ERRLOG("aclrtKernelArgsFinalize failed: %d\n", rc);
		return -1;
	}
	/* numBlocks=1, default stream (NULL), no launch cfg / ext args. */
	rc = aclrtLaunchKernelWithConfig(g_hcomm.batch_send_func, 1, NULL, NULL,
					 args_handle, NULL);
	if (rc != ACL_SUCCESS) {
		SPDK_ERRLOG("aclrtLaunchKernelWithConfig failed: %d\n", rc);
		return -1;
	}
	return 0;
}

static int
nvme_hcomm_send_sqe(struct nvme_hcomm_qpair *hqpair, uint8_t *sqe,
		    uint32_t nsgl, uint32_t cid)
{
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	struct nvme_hcomm_batch_send_param param = {};
	struct nvme_hcomm_send_io_batch *batch = hqpair->batch;
	uint32_t len = hqpair->sqe_size +
		       nsgl * (uint32_t)sizeof(struct spdk_nvme_sgl_descriptor);
	int rc;

	spdk_wmb();

	rc = nvme_hcomm_load_batch_send_kernel();
	if (rc != 0) {
		SPDK_ERRLOG("load HixlBatchSend kernel failed for %s:%u\n",
			    hctrlr->target_addr, hctrlr->target_port);
		hqpair->qpair.transport_failure_reason = SPDK_NVME_QPAIR_FAILURE_LOCAL;
		return -EIO;
	}

	batch->channel = hqpair->channel;
	batch->local_src = sqe;
	batch->len = len;
	batch->imm_data = 0;	/* cid lives in SQE DW0; imm unused for now */
	batch->reserved = 0;
	*hqpair->send_status = 0;

	param.thread = hqpair->thread;
	param.io_batches = batch;
	param.batch_size = 1;
	param.status_array = hqpair->send_status;
	param.timeout_ms = 0;
	param.stats = NULL;
	param.complete_sender_cqe = 0;

	rc = nvme_hcomm_launch_batch_send(&param);
	if (rc != 0) {
		SPDK_ERRLOG("launch HixlBatchSend failed for %s:%u cid=%u: rc=%d\n",
			    hctrlr->target_addr, hctrlr->target_port, cid, rc);
		hqpair->qpair.transport_failure_reason = SPDK_NVME_QPAIR_FAILURE_LOCAL;
		/* Fatal, not -EAGAIN - see the function comment above. */
		return -EIO;
	}
	return 0;
}

static int
nvme_hcomm_qpair_submit_request(struct spdk_nvme_qpair *qpair, struct nvme_request *req)
{
	struct nvme_hcomm_qpair *hqpair = nvme_hcomm_qpair(qpair);
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	uint8_t *sqe;
	uint32_t cid;
	uint32_t nsgl = 0;
	int rc;

	if (!hqpair->queues_created) {
		return -1;
	}

	/* Channel must be ready before we can submit commands. */
	if (!hqpair->channel_created) {
		SPDK_ERRLOG("channel not created for %s:%u (qid=%u)\n",
			    hctrlr->target_addr, hctrlr->target_port,
			    qpair->id);
		return -ENOTCONN;
	}

	cid = nvme_hcomm_qpair_reserve_cid(hqpair, req);
	if (cid == UINT32_MAX) {
		return -EAGAIN;
	}
	sqe = nvme_hcomm_qpair_fill_sqe(hqpair, req, cid);

	/* payload_size == 0: the DPTR stays zeroed from the fill. */
	if (req->payload_size > 0) {
		rc = nvme_hcomm_build_payload(hqpair, req, sqe, cid, &nsgl);
		if (rc != 0) {
			nvme_hcomm_qpair_unsubmit_cid(hqpair, cid);
			return rc;
		}
	}

	rc = nvme_hcomm_send_sqe(hqpair, sqe, nsgl, cid);
	if (rc != 0) {
		nvme_hcomm_qpair_unsubmit_cid(hqpair, cid);
		return rc;
	}

	return 0;
}

static int32_t
nvme_hcomm_qpair_process_completions(struct spdk_nvme_qpair *qpair, uint32_t max_completions)
{
	struct nvme_hcomm_qpair *hqpair = nvme_hcomm_qpair(qpair);
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	/* cid 用 uint32_t：cqe_depth 最大可为 65536，uint16_t 循环变量
	 * 在 cid 从 65535 递增到 65536 时会回绕为 0，导致死循环。 */
	uint32_t cid;
	uint32_t i = 0;    /* 遍历 active_cids[] 的下标 */
	uint32_t completed = 0;
	int rc;

	if (!hqpair->queues_created) {
		return 0;
	}

	/* max_completions == 0 means "no limit" (the framework contract -
	 * every admin poller, the fabrics connect poll and the sync connect
	 * busy-wait pass 0); clamp to the queue depth like nvme_rdma and
	 * nvme_tcp do. */
	if (max_completions == 0) {
		max_completions = hqpair->cqe_depth;
	}

	/*
	 * Poll CQE array for in-flight requests only.  TGT writes the 16B
	 * spdk_nvme_cpl at cqe_buf[cid].cpl and sets cqe_buf[cid].padding[0]=1
	 * as a valid flag (with a wmb between cpl and flag).
	 *
	 * Traversal is over active_cids[0..num_active) — the list of cids
	 * with outstanding requests — NOT a linear scan over cqe_depth, so
	 * per-poll cost is O(in-flight) instead of O(cqe_depth).  When a
	 * completion is processed the entry is removed by swap-remove (the
	 * last element moves into the vacated slot), so the index is NOT
	 * advanced in that case.
	 */
	while (i < hqpair->num_active && completed < max_completions) {
		struct nvme_hcomm_cqe_slot *cqe;
		struct nvme_request *req;

		cid = hqpair->active_cids[i];

		/* Defensive: every active entry must have a request; if not,
		 * drop it from the list and re-check the same slot.  Release
		 * any dangling mem_map reference so it cannot pin an entry. */
		if (hqpair->req_for_cid[cid] == NULL) {
			nvme_hcomm_mem_map_release_all(hctrlr->mem_map,
						   &hqpair->xlation_for_cid[cid]);
			nvme_hcomm_qpair_remove_active(hqpair, i);
			continue;
		}

		cqe = &hqpair->cqe_buf[cid];

		/* Check valid flag (padding[0]) — TGT sets it after writing cpl. */
		spdk_rmb();
		if (cqe->padding[0] == 0) {
			i++; /* still in flight — check the next cid */
			continue;
		}

		/* CQE is valid — read barrier ensures cpl visible after flag. */
		spdk_rmb();

		/* Defensive: the CQE's cid field must match the positional cid
		 * (TGT writes CQE at cqe_buf[cid] using the SQE's cmd.cid).
		 * Mirrors nvme_pcie_common.c:694 assert(cpl->cid == req->cmd.cid). */
		assert(cqe->cpl.cid == cid);

		req = hqpair->req_for_cid[cid];
		hqpair->req_for_cid[cid] = NULL;

		/* Drop the request's mem_map reference BEFORE completing:
		 * nvme_complete_request may synchronously resubmit, and the
		 * record must be stale-proof by then. */
		nvme_hcomm_mem_map_release_all(hctrlr->mem_map,
					   &hqpair->xlation_for_cid[cid]);

		/* Zero-copy: no bounce-copy needed.  For READ, the TGT
		 * already wrote data directly into the user buffer via
		 * keyed SGL (RDMA write).  For WRITE, the TGT already
		 * read from the user buffer (RDMA read).  Just complete. */

		/* Complete the NVMe request with the standard CQE.
		 * nvme_complete_request() calls req->cb_fn and frees
		 * the request internally. */
		nvme_complete_request(req->cb_fn, req->cb_arg,
				      qpair, req, &cqe->cpl);

		/* Clear CQE slot for reuse (wmb so TGT sees clear before
		 * a new submission reuses this cid). */
		memset(cqe, 0, sizeof(*cqe));
		spdk_wmb();

		/* Release the cid AFTER clearing the slot so the completion
		 * callback (which may synchronously resubmit and reuse this
		 * cid) does not race with the memset above.  Remove from the
		 * active list (swap-remove): do not advance i — the element
		 * moved into slot i must be checked. */
		nvme_hcomm_qpair_free_cid(hqpair, cid);
		nvme_hcomm_qpair_remove_active(hqpair, i);
		completed++;
	}

	/* Drive the async connect state machine after reaping (worker
	 * handshake -> fabrics CONNECT -> CONNECTED), so one poll call can
	 * both reap the fabrics CONNECT response and promote the qpair
	 * (nvme_tcp_qpair_process_completions pattern).  -EAGAIN also
	 * covers the in_connect_poll recursion guard firing from the
	 * nested fabric connect poll - it just means "still connecting". */
	if (spdk_unlikely(nvme_qpair_get_state(qpair) == NVME_QPAIR_CONNECTING)) {
		rc = nvme_hcomm_qpair_connect_poll(qpair);
		if (rc == 0) {
			/* the qpair just became CONNECTED - submit anything
			 * that piled up while it was CONNECTING */
			SPDK_DEBUGLOG(nvme_hcomm, "qid=%u CONNECTED: resubmitting up to %u queued requests\n",
				      qpair->id, hqpair->cqe_depth);
			nvme_qpair_resubmit_requests(qpair, hqpair->cqe_depth);
		} else if (rc != -EAGAIN) {
			return rc;    /* failure already unwound + finalized */
		}
	}

	return completed;
}

static int
nvme_hcomm_qpair_iterate_requests(struct spdk_nvme_qpair *qpair,
				 int (*iter_fn)(struct nvme_request *req, void *arg),
				 void *arg)
{
	struct nvme_hcomm_qpair *hqpair = nvme_hcomm_qpair(qpair);
	uint32_t i;
	int rc;

	assert(iter_fn != NULL);

	for (i = 0; i < hqpair->num_active; i++) {
		struct nvme_request *req;
		uint32_t cid = hqpair->active_cids[i];

		if (hqpair->req_for_cid[cid] == NULL) {
			continue;
		}

		req = hqpair->req_for_cid[cid];
		rc = iter_fn(req, arg);
		if (rc != 0) {
			return rc;
		}
	}

	return 0;
}

static void
nvme_hcomm_admin_qpair_abort_aers(struct spdk_nvme_qpair *qpair)
{
	struct nvme_hcomm_qpair *hqpair = nvme_hcomm_qpair(qpair);
	struct nvme_hcomm_ctrlr *hctrlr = hqpair->hctrlr;
	struct spdk_nvme_cpl cpl;
	uint32_t i = 0;

	cpl.sqid = qpair->id;
	cpl.status.sc = SPDK_NVME_SC_ABORTED_SQ_DELETION;
	cpl.status.sct = SPDK_NVME_SCT_GENERIC;

	while (i < hqpair->num_active) {
		struct nvme_request *req;
		uint32_t cid = hqpair->active_cids[i];

		if (hqpair->req_for_cid[cid] == NULL) {
			nvme_hcomm_mem_map_release_all(hctrlr->mem_map,
						   &hqpair->xlation_for_cid[cid]);
			nvme_hcomm_qpair_remove_active(hqpair, i);
			continue;
		}

		req = hqpair->req_for_cid[cid];
		if (req->cmd.opc != SPDK_NVME_OPC_ASYNC_EVENT_REQUEST) {
			i++;
			continue;
		}

		hqpair->req_for_cid[cid] = NULL;
		/* AERs carry no payload, but release defensively for safety. */
		nvme_hcomm_mem_map_release_all(hctrlr->mem_map,
					   &hqpair->xlation_for_cid[cid]);
		nvme_complete_request(req->cb_fn, req->cb_arg, qpair, req, &cpl);
		nvme_hcomm_qpair_free_cid(hqpair, cid);
		nvme_hcomm_qpair_remove_active(hqpair, i);
	}
}

/*
 * ========== Poll Group ==========
 */
static struct spdk_nvme_transport_poll_group *
nvme_hcomm_poll_group_create(void)
{
	struct nvme_hcomm_poll_group *group;
	group = calloc(1, sizeof(*group));
	if (group == NULL) {
		SPDK_ERRLOG("Unable to allocate poll group.\n");
		return NULL;
	}
	return &group->group;
}

/* No-op: the framework base class (nvme_transport_poll_group_connect_qpair)
 * moves the qpair from disconnected_qpairs to connected_qpairs after this
 * returns 0.  HCOMM has no shared poll-group-level resource (unlike TCP's
 * sock_group or RDMA's shared CQ) — each qpair independently polls its own
 * cqe_buf[cid] array, so there is nothing to do here. */
static int
nvme_hcomm_poll_group_connect_qpair(struct spdk_nvme_qpair *qpair)
{
	return 0;
}

/* No-op: the framework base class moves the qpair from connected_qpairs back
 * to disconnected_qpairs after this returns 0.  See connect_qpair comment. */
static int
nvme_hcomm_poll_group_disconnect_qpair(struct spdk_nvme_qpair *qpair)
{
	return 0;
}

/* No-op: the framework base class inserts the qpair into disconnected_qpairs
 * after this returns 0.  HCOMM needs no per-poll-group qpair bookkeeping. */
static int
nvme_hcomm_poll_group_add(struct spdk_nvme_transport_poll_group *tgroup,
			  struct spdk_nvme_qpair *qpair)
{
	return 0;
}

/* No-op: the framework base class removes the qpair from whichever STAILQ
 * it currently belongs to.  Nothing to clean up at the poll-group level. */
static int
nvme_hcomm_poll_group_remove(struct spdk_nvme_transport_poll_group *tgroup,
			     struct spdk_nvme_qpair *qpair)
{
	return 0;
}

static int64_t
nvme_hcomm_poll_group_process_completions(struct spdk_nvme_transport_poll_group *tgroup,
		uint32_t completions_per_qpair, spdk_nvme_disconnected_qpair_cb disconnected_qpair_cb)
{
	struct spdk_nvme_qpair *qpair, *tmp_qpair;
	int64_t total_completions = 0;

	/* Process disconnected qpairs: HCOMM disconnect is synchronous
	 * (HcommChannelDestroy is called in ctrlr_disconnect_qpair), so the
	 * qpair is already fully disconnected when it reaches this list.
	 * Call the callback for each to let SPDK finalize the removal. */
	STAILQ_FOREACH_SAFE(qpair, &tgroup->disconnected_qpairs, poll_group_stailq, tmp_qpair) {
		disconnected_qpair_cb(qpair, tgroup->group->ctx);
	}

	/* Poll CQEs on all connected qpairs.  Each qpair independently polls
	 * its own cqe_buf[cid] array (cid-indexed, not a shared CQ).  A
	 * negative return is surfaced to the group caller after the qpair
	 * itself was unwound/finalized inside process_completions - the
	 * framework connect busy-wait and the bdev layer both react to it. */
	STAILQ_FOREACH_SAFE(qpair, &tgroup->connected_qpairs, poll_group_stailq, tmp_qpair) {
		int32_t rc = nvme_hcomm_qpair_process_completions(qpair, completions_per_qpair);

		if (rc < 0) {
			return rc;
		}
		total_completions += rc;
	}

	return total_completions;
}

static int
nvme_hcomm_poll_group_destroy(struct spdk_nvme_transport_poll_group *tgroup)
{
	struct nvme_hcomm_poll_group *group = nvme_hcomm_poll_group(tgroup);
	free(group);
	return 0;
}

static int
nvme_hcomm_poll_group_get_stats(struct spdk_nvme_transport_poll_group *tgroup,
				struct spdk_nvme_transport_poll_group_stat **_stats)
{
	struct spdk_nvme_transport_poll_group_stat *stats;

	if (tgroup == NULL || _stats == NULL) {
		SPDK_ERRLOG("Invalid stats or group pointer\n");
		return -EINVAL;
	}

	stats = calloc(1, sizeof(*stats));
	if (stats == NULL) {
		SPDK_ERRLOG("Unable to allocate HCOMM stats\n");
		return -ENOMEM;
	}

	stats->trtype = SPDK_NVME_TRANSPORT_HCOMM;
	/* No custom-fabrics member exists in spdk_nvme_transport_poll_group_stat,
	 * so the aggregate counters are stored in the tcp sub-struct (same
	 * workaround as the original implementation).  num_connected_qpairs
	 * reflects how many qpairs this poll group is driving. */
	stats->tcp.queued_requests = tgroup->num_connected_qpairs;

	*_stats = stats;

	return 0;
}

static void
nvme_hcomm_poll_group_free_stats(struct spdk_nvme_transport_poll_group *tgroup,
				 struct spdk_nvme_transport_poll_group_stat *stats)
{
	free(stats);
}


/* ========================================================================
 * DEAD CODE PRESERVATION AREA
 *
 * The following functions implement the manual UB bringup path (JFC/Jetty
 * create → exchange → import → bind) that was superseded by HcommChannelCreate.
 * They are retained for reference and potential future use but excluded from
 * compilation via #if 0.  To revive any of it, copy the function out and
 * ensure the hctrlr fields it references are present.
 * ========================================================================
 */
#if 0

/* ---- UBC_TP control-plane socket (existing path, now using shared helpers) ---- */
static int
nvme_hcomm_target_connect(struct nvme_hcomm_ctrlr *hctrlr,
			   const EndpointDesc *remoteEp)
{
	/* remoteEp was used pre-refactor for the parsed IP; the shared
	 * helper now parses hctrlr->target_addr directly, so remoteEp
	 * is kept only for caller-side address validation. */
	(void)remoteEp;

	if (g_hcomm.socket_handle == NULL) {
		SPDK_ERRLOG("Global socket handle not initialised\n");
		return -1;
	}
	if (hctrlr->target_port == 0) {
		SPDK_ERRLOG("Target port required for connection\n");
		return -1;
	}
	return nvme_hcomm_ra_socket_connect(g_hcomm.socket_handle, hctrlr->target_addr,
					    hctrlr->target_port, "SPDK_HCOMM",
					    &hctrlr->fd_handle);
}

static void
nvme_hcomm_target_disconnect(struct nvme_hcomm_ctrlr *hctrlr)
{
	nvme_hcomm_ra_socket_disconnect(g_hcomm.socket_handle, hctrlr->fd_handle);
	hctrlr->fd_handle = NULL;
}

/*
 * ========== Per-target Jetty management ==========
 *
 * Each target gets its own JFC / Jetty pair through the shared g_hcomm.ctx_handle.
 *
 * Sequence:
 *   1. RaCtxCqCreate   -> jfc_handle
 *   2. RaCtxQpCreate   -> jetty_handle, local_qp_info
 *   3. Exchange local_qp_info with peer over socket fd
 *   4. (Optional) RaCtxGetTpInfoList -> tp_info
 *   5. RaCtxQpImport   -> rem_jetty_handle
 *   6. RaCtxQpBind     -> jetty link established
 */

/*
 * Desired (upper-bound) UB queue depths — *requests*, clamped at runtime by
 * the device's actual caps (g_hcomm.dev_attr, queried via RaGetDevBaseAttr after
 * RaCtxInit).  These fallbacks and the three clamp helpers below are used
 * only by the manual bringup path in this preservation area; the live
 * transport sizes queues from qsize via nvme_hcomm_qpair_sq_size().
 *
 * Rationale:
 *   - One Jetty is shared by ALL qpairs (admin + IO) of a ctrlr, so JFC depth
 *     must be >= the Jetty's sqDepth (one CQE per outstanding SQE).  Use the
 *     same value as JFC.
 *   - JFS sqDepth bounds outstanding SQEs (one per in-flight NVMe CMD across
 *     all qpairs of the ctrlr).  1024 covers the common case (1 admin +
 *     a few IO qpairs at SPDK's default io_queue_size of 1024) with headroom.
 *   - JFS rqDepth bounds posted receive buffers; under the fire-and-forget
 *     data-plane (TGT DMA-writes into HOST cqe_buf, no RDMA RECV posted on
 *     the INI side) the RQ is mostly unused, so 64 is plenty.
 */
#define NVME_HCOMM_DESIRED_JFC_DEPTH	4096U
#define NVME_HCOMM_DESIRED_JFS_SQ_DEPTH	1024U
#define NVME_HCOMM_DESIRED_JFS_RQ_DEPTH	64U

/* Clamp the fallbacks by the device caps; on query failure use as-is so the
 * bringup still comes up (RaCtxCqCreate / RaCtxQpCreate will not fail). */
static uint32_t
nvme_hcomm_jfc_depth(void)
{
	uint32_t desired = NVME_HCOMM_DESIRED_JFC_DEPTH;

	if (g_hcomm.dev_attr_inited && g_hcomm.dev_attr.sqMaxDepth > 0) {
		desired = spdk_min(desired, g_hcomm.dev_attr.sqMaxDepth);
	}
	return desired;
}

static uint32_t
nvme_hcomm_jfs_sq_depth(void)
{
	uint32_t desired = NVME_HCOMM_DESIRED_JFS_SQ_DEPTH;

	if (g_hcomm.dev_attr_inited && g_hcomm.dev_attr.sqMaxDepth > 0) {
		desired = spdk_min(desired, g_hcomm.dev_attr.sqMaxDepth);
	}
	return desired;
}

static uint32_t
nvme_hcomm_jfs_rq_depth(void)
{
	uint32_t desired = NVME_HCOMM_DESIRED_JFS_RQ_DEPTH;

	if (g_hcomm.dev_attr_inited && g_hcomm.dev_attr.rqMaxDepth > 0) {
		desired = spdk_min(desired, g_hcomm.dev_attr.rqMaxDepth);
	}
	return desired;
}

static int
nvme_hcomm_target_create_jfc(struct nvme_hcomm_ctrlr *hctrlr)
{
	struct CqInfoT cqInfo;
	int rc;

	if (hctrlr->jfc_created) {
		return 0;
	}
	if (g_hcomm.ctx_handle == NULL) {
		SPDK_ERRLOG("Global ctx_handle not initialised\n");
		return -1;
	}

	memset(&cqInfo, 0, sizeof(cqInfo));
	cqInfo.in.depth = nvme_hcomm_jfc_depth();

	rc = RaCtxCqCreate(g_hcomm.ctx_handle, &cqInfo, &hctrlr->jfc_handle);
	if (rc != 0 || hctrlr->jfc_handle == NULL) {
		SPDK_ERRLOG("RaCtxCqCreate (JFC) for target %s failed: ret=%d "
			    "(requested depth=%u, device sqMaxDepth=%u)\n",
			    hctrlr->target_addr, rc, cqInfo.in.depth,
			    g_hcomm.dev_attr_inited ? g_hcomm.dev_attr.sqMaxDepth : 0);
		hctrlr->jfc_handle = NULL;
		return -1;
	}

	hctrlr->jfc_created = true;
	SPDK_DEBUGLOG(nvme_hcomm, "JFC created for %s, handle=%p, depth=%u\n",
		      hctrlr->target_addr, hctrlr->jfc_handle, cqInfo.in.depth);
	return 0;
}

static int
nvme_hcomm_target_create_jetty(struct nvme_hcomm_ctrlr *hctrlr)
{
	struct QpCreateAttr qpAttr;
	int rc;

	if (hctrlr->jetty_created) {
		return 0;
	}
	if (!hctrlr->jfc_created || hctrlr->jfc_handle == NULL) {
		SPDK_ERRLOG("JFC not created for target %s\n", hctrlr->target_addr);
		return -1;
	}

	memset(&qpAttr, 0, sizeof(qpAttr));
	qpAttr.scqHandle = hctrlr->jfc_handle;
	qpAttr.rcqHandle = hctrlr->jfc_handle;
	qpAttr.srqHandle = NULL;
	qpAttr.sqDepth = nvme_hcomm_jfs_sq_depth();
	qpAttr.rqDepth = nvme_hcomm_jfs_rq_depth();
	qpAttr.transportMode = nvme_hcomm_trans_mode();
	qpAttr.ub.mode = JETTY_MODE_URMA_NORMAL;
	qpAttr.ub.priority = 4;

	memset(&hctrlr->local_qp_info, 0, sizeof(hctrlr->local_qp_info));

	rc = RaCtxQpCreate(g_hcomm.ctx_handle, &qpAttr, &hctrlr->local_qp_info, &hctrlr->jetty_handle);
	if (rc != 0 || hctrlr->jetty_handle == NULL) {
		SPDK_ERRLOG("RaCtxQpCreate (Jetty) for %s failed: ret=%d "
			    "(sqDepth=%u rqDepth=%u, device sqMaxDepth=%u rqMaxDepth=%u)\n",
			    hctrlr->target_addr, rc,
			    qpAttr.sqDepth, qpAttr.rqDepth,
			    g_hcomm.dev_attr_inited ? g_hcomm.dev_attr.sqMaxDepth : 0,
			    g_hcomm.dev_attr_inited ? g_hcomm.dev_attr.rqMaxDepth : 0);
		hctrlr->jetty_handle = NULL;
		return -1;
	}

	hctrlr->jetty_created = true;
	SPDK_DEBUGLOG(nvme_hcomm, "Jetty created for %s, jettyId=%u, va=0x%lx, "
		      "sqDepth=%u rqDepth=%u\n",
		      hctrlr->target_addr, hctrlr->local_qp_info.ub.id,
		      hctrlr->local_qp_info.va,
		      qpAttr.sqDepth, qpAttr.rqDepth);
	return 0;
}

/*
 * Negotiate the connection with the target (INI = client) over the connected
 * HCCP socket using the nvmf_cm_conn_req / nvmf_cm_conn_resp frame.
 *
 *   1. send nvmf_cm_conn_req   (INI jetty id + NVMe params + ini_cqe_addr)
 *   2. recv nvmf_cm_conn_resp  (TGT jetty id + active TP + status)
 *
 * The HCCP whitelist (256B tag) + "a5a5a\0" ack is NOT exchanged on this
 * socket: under the verified RaSocket OOB path the whitelist is consumed
 * internally by the HCCP state machine during RaSocketBatchConnect, so the
 * business layer speaks CM frames directly (reference: nids_aiv_ub
 * src/core/oob/ub_oob_rasocket.cpp).
 *
 * Under CONN_RM the INI rebuilds the 64B QpKey from the discrete
 * {remote_eid, remote_uasid, remote_jetty_id, trans_mode} identity the TGT
 * publishes in resp.ub_info — no opaque QpKey / MemKey / rmem seg is exchanged.
 * The TGT's tp_handle / psn (tx) are taken as the INI's peerTpHandle / rxPsn
 * for EXP import.  The TGT's jetty_token (its JFR token_value) is what the INI
 * presents when importing the TGT jetty.
 *
 * qpair is passed in so the req can carry the NVMe queue parameters
 * (qid / sq_size / rq_size) and the INI CQE array physical address (cqe_phys).
 */
static int
nvme_hcomm_target_exchange(struct nvme_hcomm_ctrlr *hctrlr,
			   struct spdk_nvme_qpair *qpair)
{
	struct nvmf_cm_conn_req req;
	struct nvmf_cm_conn_resp resp;
	struct nvme_hcomm_qpair *hqpair = nvme_hcomm_qpair(qpair);
	const struct QpCreateInfo *qpInfo = &hctrlr->local_qp_info;
	uint32_t sq_size, rq_size;

	/* NVMe queue parameters for the req.  admin qpair (id=0) uses the admin
	 * queue size, IO qpairs use the io queue size; both are then clamped by
	 * the device's sqMaxDepth via nvme_hcomm_qpair_sq_size() so the value
	 * told to the TGT matches what the INI's per-qpair rings actually hold
	 * (hqpair->cqe_depth / sq_depth, set in nvme_hcomm_create_queues) and
	 * what the underlying Jetty sqDepth can sustain.  rq_size == sq_size is
	 * the simplest defensible mapping (the TGT only needs the depth to size
	 * its side of the queue).  spdk_nvme_qpair names the queue identifier
	 * `id`. */
	if (qpair->id == 0) {
		sq_size = hctrlr->ctrlr.opts.admin_queue_size;
	} else {
		sq_size = hctrlr->ctrlr.opts.io_queue_size;
	}
	sq_size = nvme_hcomm_qpair_sq_size(sq_size);
	rq_size = sq_size;

	/* Step 1: build & send nvmf_cm_conn_req. */
	memset(&req, 0, sizeof(req));
	req.hdr.version   = NVME_HCOMM_CM_VERSION_V3;
	req.hdr.opcode    = NVME_HCOMM_CM_OP_REQ;
	req.hdr.trans_type = NVME_HCOMM_CM_TRANS_HCOMM;
	req.hdr.status    = NVME_HCOMM_CM_STATUS_OK;
	req.hdr.msg_len   = sizeof(req);

	/* Outer NVMe queue params.  ctrl_id is 0 here: cntlid is assigned by the
	 * target only after a successful Fabrics Connect, which has not run yet
	 * at connect_qpair time.  The TGT associates the req with the socket tag
	 * (target IP+port) instead. */
	req.conn_info.ctrl_id = 0;
	req.conn_info.qid     = qpair->id;
	req.conn_info.sq_size = sq_size;
	req.conn_info.rq_size = rq_size;
	req.conn_info.data_slot_size = 0;  /* unused (zero-copy, no bounce buffer) */
	req.conn_info.ini_cqe_addr = (uint64_t)(uintptr_t)hqpair->cqe_phys;
	req.conn_info.ini_cqe_tokenid = hqpair->cqe_tokenid;

	/* INI jetty identity + active-TP fields.  tp_handle=0 (peer EID unknown
	 * at req time; the INI queries its own local tpHandle in get_tp_info
	 * after parsing resp).  psn = INI's tx PSN (generated once here; the TGT
	 * takes it as its rx PSN; the same value is reused at import time — see
	 * hqpair->local_tx_psn).  jetty_token = INI JFR token (presented by the
	 * TGT when it imports the INI jetty). */
	hqpair->local_tx_psn = (uint32_t)rand();
	req.conn_info.ub_info.tp_handle   = 0;
	req.conn_info.ub_info.psn          = hqpair->local_tx_psn;
	req.conn_info.ub_info.jetty_type   = 1;   /* TARGET_TYPE_JETTY / urma_jetty */
	req.conn_info.ub_info.is_shared     = 0;
	memcpy(req.conn_info.ub_info.eid, hctrlr->local_eid, 16);
	req.conn_info.ub_info.uasid        = qpInfo->ub.uasid;
	req.conn_info.ub_info.jetty_id      = qpInfo->ub.id;
	req.conn_info.ub_info.jetty_token  = NVME_HCOMM_INI_JFR_TOKEN;

	if (nvme_hcomm_socket_send(hctrlr->fd_handle, &req, sizeof(req)) != 0) {
		SPDK_ERRLOG("exchange %s: send conn_req failed\n", hctrlr->target_addr);
		return -1;
	}

	/* Step 2: recv nvmf_cm_conn_resp. */
	memset(&resp, 0, sizeof(resp));
	if (nvme_hcomm_socket_recv(hctrlr->fd_handle, &resp, sizeof(resp)) != 0) {
		SPDK_ERRLOG("exchange %s: recv conn_resp failed\n", hctrlr->target_addr);
		return -1;
	}
	if (resp.hdr.opcode != NVME_HCOMM_CM_OP_RESP) {
		SPDK_ERRLOG("exchange %s: resp.opcode != RESP (got %u)\n",
			    hctrlr->target_addr, resp.hdr.opcode);
		return -1;
	}
	if (resp.hdr.trans_type != NVME_HCOMM_CM_TRANS_HCOMM) {
		SPDK_ERRLOG("exchange %s: resp.trans_type mismatch (%u)\n",
			    hctrlr->target_addr, resp.hdr.trans_type);
		return -1;
	}
	if (resp.hdr.version != NVME_HCOMM_CM_VERSION_V3) {
		SPDK_ERRLOG("exchange %s: resp.version mismatch (got %u, need %u). "
			    "TGT must be updated to the v3 registration frame.\n",
			    hctrlr->target_addr, resp.hdr.version,
			    NVME_HCOMM_CM_VERSION_V3);
		return -1;
	}
	if (resp.hdr.status != NVME_HCOMM_CM_STATUS_OK) {
		SPDK_ERRLOG("exchange %s: TGT rejected req (status=%u)\n",
			    hctrlr->target_addr, resp.hdr.status);
		return -1;
	}

	/* Parse TGT jetty identity + active TP into hqpair. */
	memcpy(hqpair->tgt_info.eid, resp.conn_info.ub_info.eid, sizeof(hqpair->tgt_info.eid));
	hqpair->tgt_info.tp_handle    = resp.conn_info.ub_info.tp_handle;   /* → expImportCfg.peerTpHandle */
	hqpair->tgt_info.psn          = resp.conn_info.ub_info.psn;         /* → expImportCfg.rxPsn */
	hqpair->tgt_info.uasid        = resp.conn_info.ub_info.uasid;
	hqpair->tgt_info.jetty_id     = resp.conn_info.ub_info.jetty_id;
	hqpair->tgt_info.jetty_token  = resp.conn_info.ub_info.jetty_token; /* TGT JFR token */
	hctrlr->remote_conn_mode      = nvme_hcomm_conn_mode();             /* always CONN_RM (both profiles) */

	SPDK_DEBUGLOG(nvme_hcomm, "exchange %s ok: peer_tp=0x%lx peer_txpsn=%u "
		      "conn_mode=%u uasid=%u jetty_id=%u jetty_token=0x%x "
		      "ctrl_id=%u qid=%u sq_size=%u rq_size=%u ini_cqe_addr=0x%lx\n",
		      hctrlr->target_addr,
		      (unsigned long)hqpair->tgt_info.tp_handle,
		      hqpair->tgt_info.psn,
		      hctrlr->remote_conn_mode, hqpair->tgt_info.uasid,
		      hqpair->tgt_info.jetty_id, hqpair->tgt_info.jetty_token,
		      resp.conn_info.ctrl_id, resp.conn_info.qid,
		      resp.conn_info.sq_size, resp.conn_info.rq_size,
		      (unsigned long)resp.conn_info.ini_cqe_addr);
	return 0;
}

/*
 * Query the local TP handle for the (localEid → peerEid) path.  The peer EID
 * comes from the negotiation response (hqpair->tgt_info.eid), not from
 * target_addr, so this must run after nvme_hcomm_target_exchange.
 * transMode=CONN_RM to match the EXP import (HCCP requires the get_tp_list
 * trans_mode to equal the rjetty trans_mode); the TP type is rtp for peer /
 * ctp for offline (see nvme_hcomm_use_rtp).
 */
static int
nvme_hcomm_target_get_tp_info(struct nvme_hcomm_ctrlr *hctrlr,
			      struct spdk_nvme_qpair *qpair)
{
	struct GetTpCfg tpCfg;
	struct nvme_hcomm_qpair *hqpair = nvme_hcomm_qpair(qpair);
	unsigned int num = 1;
	int rc;

	memset(&tpCfg, 0, sizeof(tpCfg));
	tpCfg.transMode = nvme_hcomm_trans_mode();
	/* PEER_ONLINE/UBoE -> Reliable TP (rtp); OFFLINE/UBC -> lightweight CTP. */
	if (nvme_hcomm_use_rtp()) {
		tpCfg.flag.bs.rtp = 1;
	} else {
		tpCfg.flag.bs.ctp = 1;
	}

	/* peerEid = target EID from the negotiation response. */
	memcpy(&tpCfg.peerEid, hqpair->tgt_info.eid, sizeof(tpCfg.peerEid));
	/* localEid = self EID detected earlier. */
	memcpy(&tpCfg.localEid, hctrlr->local_eid, sizeof(tpCfg.localEid));

	rc = RaCtxGetTpInfoList(g_hcomm.ctx_handle, &tpCfg, &hctrlr->tp_info, &num);
	if (rc != 0 || num == 0) {
		SPDK_ERRLOG("RaCtxGetTpInfoList for %s failed: ret=%d num=%u\n",
			    hctrlr->target_addr, rc, num);
		return -1;
	}

	SPDK_DEBUGLOG(nvme_hcomm, "TP info for %s: tpHandle=0x%lx (peer_eid[0]=0x%02x)\n",
		      hctrlr->target_addr, hctrlr->tp_info.tpHandle, hqpair->tgt_info.eid[0]);
	return 0;
}

/*
 * Build a 64B QpKey from the CONN_RM jetty identity {EID, uasid, jetty_id,
 * transport mode}.  Used when the peer published the discrete identity instead
 * of an opaque key.  Returns the used size (32).  conn_mode 0=Rc / 1=Rm maps to
 * transport mode RC=0x2 / RM=0x1 written at offset 24.
 */
static uint8_t
nvme_hcomm_fill_jetty_key(const uint8_t eid[16], uint32_t uasid, uint32_t jettyId,
			   uint8_t conn_mode, uint8_t outKey[64])
{
	struct nvme_hcomm_jetty_key_info info;

	memset(&info, 0, sizeof(info));
	memcpy(info.eid.raw, eid, 16);
	info.uasid = uasid;
	info.id = jettyId;
	info.trans_mode = (conn_mode == 0) ? 0x2 /*RC*/ : 0x1 /*RM*/;
	memset(outKey, 0, 64);
	memcpy(outKey, &info, sizeof(info));
	return (uint8_t)sizeof(info);
}

/*
 * Import the target jetty via RaCtxQpImport in EXP mode.  EXP requires the
 * expImportCfg to carry the local TP handle (from get_tp_info) and the peer TP
 * handle (from the negotiation response).
 *
 * EXP + CONN_RM is the verified form for BOTH deployment modes (reference:
 * nids_aiv_ub ub_v2_resource.cpp — UBC forces RM because RC+CTP is EINVAL on
 * HW; UBoE defaults to RM).  Under CTP (offline) the peer TP handle may be 0
 * (umdk perftest: CTP does not exchange peer handle), which the active-TP
 * form below tolerates (peer_tp_handle defaults to 0 for minor < V1_2).
 *
 * Key handling: CONN_RM (conn_mode==1) rebuilds the 64B QpKey from the discrete
 * {eid, uasid, jettyId, transMode} identity published in the response; CONN_RC
 * (conn_mode==0) uses the opaque qp_key_raw.  Bind is not done here — CONN_RM is
 * unilateral and skips bind (see connect_qpair).
 *
 * Active-TP form (verified on hardware): tag=0, txPsn=random, rxPsn=0,
 * peerTpHandle from the response.
 */
static int
nvme_hcomm_target_import_jetty(struct nvme_hcomm_ctrlr *hctrlr,
			       struct spdk_nvme_qpair *qpair)
{
	struct QpImportInfoT importInfo;
	struct nvme_hcomm_qpair *hqpair = nvme_hcomm_qpair(qpair);

	if (hctrlr->remote_imported) {
		return 0;
	}

	memset(&importInfo, 0, sizeof(importInfo));

	/* CONN_RM: rebuild QpKey from discrete id. */
	importInfo.in.key.size = nvme_hcomm_fill_jetty_key(
		hqpair->tgt_info.eid, hqpair->tgt_info.uasid,
		hqpair->tgt_info.jetty_id, hctrlr->remote_conn_mode,
		importInfo.in.key.value);

	importInfo.in.ub.mode            = JETTY_IMPORT_MODE_EXP;
	importInfo.in.ub.tokenValue      = hqpair->tgt_info.jetty_token;  /* TGT JFR token (from resp) */
	importInfo.in.ub.policy          = JETTY_GRP_POLICY_RR;
	importInfo.in.ub.type            = TARGET_TYPE_JETTY;
	importInfo.in.ub.flag.value      = 0;
	importInfo.in.ub.flag.bs.tokenPolicy = TOKEN_POLICY_PLAIN_TEXT;
	importInfo.in.ub.flag.bs.orderType   = 0;
	importInfo.in.ub.flag.bs.shareTp     = 0;
	importInfo.in.ub.tpType         = nvme_hcomm_tp_type();   /* RTP(peer) / CTP(offline), matches get_tp_info */

	/* expImportCfg (only effective under EXP mode).
	 *   peerTpHandle = TGT's local tpHandle (from tgt_info.tp_handle)
	 *   rxPsn        = TGT's tx PSN (from tgt_info.psn) — peer tx == local rx
	 *   txPsn        = INI's own tx PSN (the same value published in
	 *                 req.ub_info.psn, stored in hqpair->local_tx_psn; reusing
	 *                 it keeps the INI's advertised tx PSN consistent with what
	 *                 it actually uses at import)
	 *   tag          = 0 (active-TP form; EID-derived on both sides) */
	importInfo.in.ub.expImportCfg.tpHandle     = hctrlr->tp_info.tpHandle;
	importInfo.in.ub.expImportCfg.peerTpHandle = hqpair->tgt_info.tp_handle;  /* from resp */
	importInfo.in.ub.expImportCfg.tag          = 0;                           /* active-TP form */
	importInfo.in.ub.expImportCfg.txPsn        = hqpair->local_tx_psn;        /* INI own tx PSN (from req) */
	importInfo.in.ub.expImportCfg.rxPsn        = hqpair->tgt_info.psn;        /* TGT tx PSN → INI rx PSN */

	SPDK_DEBUGLOG(nvme_hcomm, "RaCtxQpImport(EXP) %s: mode=%s key.size=%u "
		      "localTp=0x%lx peerTp=0x%lx tag=0 txPsn=%u rxPsn=%u jettyToken=0x%x\n",
		      hctrlr->target_addr, "RM-rebuild",
		      importInfo.in.key.size,
		      (unsigned long)hctrlr->tp_info.tpHandle,
		      (unsigned long)hqpair->tgt_info.tp_handle,
		      importInfo.in.ub.expImportCfg.txPsn,
		      importInfo.in.ub.expImportCfg.rxPsn,
		      hqpair->tgt_info.jetty_token);

	if (RaCtxQpImport(g_hcomm.ctx_handle, &importInfo, &hctrlr->rem_jetty_handle) != 0 ||
	    hctrlr->rem_jetty_handle == NULL) {
		SPDK_ERRLOG("RaCtxQpImport(EXP) for %s failed\n", hctrlr->target_addr);
		hctrlr->rem_jetty_handle = NULL;
		return -1;
	}

	hctrlr->remote_imported = true;
	SPDK_DEBUGLOG(nvme_hcomm, "Remote jetty imported (EXP) for %s, remHandle=%p\n",
		      hctrlr->target_addr, hctrlr->rem_jetty_handle);
	return 0;
}

static void
nvme_hcomm_target_unimport_jetty(struct nvme_hcomm_ctrlr *hctrlr)
{
	if (hctrlr->remote_imported && hctrlr->rem_jetty_handle != NULL) {
		int rc = RaCtxQpUnimport(g_hcomm.ctx_handle, hctrlr->rem_jetty_handle);
		if (rc != 0) {
			SPDK_WARNLOG("RaCtxQpUnimport for %s failed: ret=%d\n",
				     hctrlr->target_addr, rc);
		}
		hctrlr->rem_jetty_handle = NULL;
		hctrlr->remote_imported = false;
	}
}

static void
nvme_hcomm_target_destroy_jetty(struct nvme_hcomm_ctrlr *hctrlr)
{
	if (hctrlr->jetty_created && hctrlr->jetty_handle != NULL) {
		int rc = RaCtxQpDestroy(hctrlr->jetty_handle);
		if (rc != 0) {
			SPDK_WARNLOG("RaCtxQpDestroy for %s failed: ret=%d\n",
				     hctrlr->target_addr, rc);
		}
		hctrlr->jetty_handle = NULL;
		hctrlr->jetty_created = false;
	}
}

static void
nvme_hcomm_target_destroy_jfc(struct nvme_hcomm_ctrlr *hctrlr)
{
	if (hctrlr->jfc_created && hctrlr->jfc_handle != NULL) {
		int rc = RaCtxCqDestroy(g_hcomm.ctx_handle, hctrlr->jfc_handle);
		if (rc != 0) {
			SPDK_WARNLOG("RaCtxCqDestroy for %s failed: ret=%d\n",
				     hctrlr->target_addr, rc);
		}
		hctrlr->jfc_handle = NULL;
		hctrlr->jfc_created = false;
	}
}

static int
nvme_hcomm_build_remote_endpoint_desc(struct nvme_hcomm_ctrlr *hctrlr,
				      const struct spdk_nvme_transport_id *trid,
				      EndpointDesc *remoteEp)
{
	int rc;

	rc = EndpointDescInit(remoteEp, 1);
	if (rc != 0) {
		return -1;
	}
	remoteEp->protocol = nvme_hcomm_protocol();
	remoteEp->loc.locType = ENDPOINT_LOC_TYPE_HOST;

	if (trid->adrfam == SPDK_NVMF_ADRFAM_IPV6) {
		remoteEp->commAddr.type = COMM_ADDR_TYPE_IP_V6;
		rc = inet_pton(AF_INET6, trid->traddr, &remoteEp->commAddr.addr6);
		if (rc <= 0) {
			SPDK_ERRLOG("Failed to parse IPv6 address: %s\n", trid->traddr);
			return -1;
		}
	} else {
		remoteEp->commAddr.type = COMM_ADDR_TYPE_IP_V4;
		rc = inet_pton(AF_INET, trid->traddr, &remoteEp->commAddr.addr);
		if (rc <= 0) {
			SPDK_ERRLOG("Failed to parse IPv4 address: %s\n", trid->traddr);
			return -1;
		}
	}
	return 0;
}



#endif /* DEAD CODE PRESERVATION AREA */

/*
 * ========== Transport Operations Registration ==========
 */
const struct spdk_nvme_transport_ops hcomm_ops = {
	.name = "HCOMM",
	.type = SPDK_NVME_TRANSPORT_HCOMM,
	.ctrlr_construct = nvme_hcomm_ctrlr_construct,
	.ctrlr_scan = nvme_fabric_ctrlr_scan,
	.ctrlr_destruct = nvme_hcomm_ctrlr_destruct,
	.ctrlr_enable = nvme_hcomm_ctrlr_enable,

	.ctrlr_set_reg_4 = nvme_fabric_ctrlr_set_reg_4,
	.ctrlr_set_reg_8 = nvme_fabric_ctrlr_set_reg_8,
	.ctrlr_get_reg_4 = nvme_fabric_ctrlr_get_reg_4,
	.ctrlr_get_reg_8 = nvme_fabric_ctrlr_get_reg_8,
	.ctrlr_set_reg_4_async = nvme_fabric_ctrlr_set_reg_4_async,
	.ctrlr_set_reg_8_async = nvme_fabric_ctrlr_set_reg_8_async,
	.ctrlr_get_reg_4_async = nvme_fabric_ctrlr_get_reg_4_async,
	.ctrlr_get_reg_8_async = nvme_fabric_ctrlr_get_reg_8_async,

	.ctrlr_get_max_xfer_size = nvme_hcomm_ctrlr_get_max_xfer_size,
	.ctrlr_get_max_sges = nvme_hcomm_ctrlr_get_max_sges,

	.ctrlr_create_io_qpair = nvme_hcomm_ctrlr_create_io_qpair,
	.ctrlr_delete_io_qpair = nvme_hcomm_ctrlr_delete_io_qpair,
	.ctrlr_connect_qpair = nvme_hcomm_ctrlr_connect_qpair,
	.ctrlr_disconnect_qpair = nvme_hcomm_ctrlr_disconnect_qpair,

	.ctrlr_get_memory_domains = nvme_hcomm_ctrlr_get_memory_domains,

	.qpair_abort_reqs = nvme_hcomm_qpair_abort_reqs,
	.qpair_reset = nvme_hcomm_qpair_reset,
	.qpair_submit_request = nvme_hcomm_qpair_submit_request,
	.qpair_process_completions = nvme_hcomm_qpair_process_completions,
	.qpair_iterate_requests = nvme_hcomm_qpair_iterate_requests,
	.admin_qpair_abort_aers = nvme_hcomm_admin_qpair_abort_aers,

	.poll_group_create = nvme_hcomm_poll_group_create,
	.poll_group_connect_qpair = nvme_hcomm_poll_group_connect_qpair,
	.poll_group_disconnect_qpair = nvme_hcomm_poll_group_disconnect_qpair,
	.poll_group_add = nvme_hcomm_poll_group_add,
	.poll_group_remove = nvme_hcomm_poll_group_remove,
	.poll_group_process_completions = nvme_hcomm_poll_group_process_completions,
	.poll_group_destroy = nvme_hcomm_poll_group_destroy,
	.poll_group_get_stats = nvme_hcomm_poll_group_get_stats,
	.poll_group_free_stats = nvme_hcomm_poll_group_free_stats,
};

SPDK_NVME_TRANSPORT_REGISTER(hcomm, &hcomm_ops);

SPDK_LOG_REGISTER_COMPONENT(nvme_hcomm)
