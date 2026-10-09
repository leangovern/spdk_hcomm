/*   SPDX-License-Identifier: BSD-3-Clause
 *
 *   HCOMM 96B SQE wire format — 96-byte extended SQE.
 *
 *   Layout (24 dwords + tail SGL array):
 *
 *     dword  0~15 : standard NVMe 64B SQE, byte-for-byte identical to
 *                   struct spdk_nvme_cmd (OPC/FUSE/PSDT/CID + NSID +
 *                   MPTR + DPTR(PRP/SGL) + CDW10~15)
 *     dword 16~21 : reserved (24 bytes, zero)
 *     dword 22~23 : routing information (8 bytes)
 *                     dword 22 : routing word — bit 0~15 = ctrlid,
 *                                bit 16~31 = sqid
 *                     dword 23 : jetty_token — INI JFR jetty token
 *     byte  96~   : tail SGL descriptor array (up to
 *                   NVME_HCOMM_MAX_SGL * 16B, keyed data blocks)
 *
 *   Multi-SGL (scatter-gather) extension - single count source, the
 *   standard NVMe-oF "SGL in capsule" convention (same as
 *   nvme_rdma build_sgl_request):
 *     - single segment: one keyed data block descriptor sits in
 *       cmd.dptr.sgl1 - the legacy layout, byte-identical to the
 *       pre-extension wire format, so an old TGT keeps working.
 *     - multiple segments: N keyed data block descriptors sit in the
 *       tail array at byte 96 (16B each, struct spdk_nvme_sgl_descriptor
 *       wire layout: address LE at +0, 24-bit length at +8, 32-bit key
 *       at +11, subtype|type nibbles at +15).  cmd.dptr.sgl1 carries a
 *       LAST_SEGMENT+OFFSET descriptor whose 24-bit length = N*16 -
 *       this is the one and only count encoding; the TGT derives
 *       N = length/16 and must bound it by NVME_HCOMM_MAX_SGL.  The
 *       SEND length grows to 96 + N*16.
 *     - Max descriptors per SQE: NVME_HCOMM_MAX_SGL (16).  More
 *       segments must be split by the caller before submit.
 *
 *   NOTE: the former 128B extended SQE format (nvme_hcomm_sqe.h) has
 *   been temporarily removed - 96B is the only wire format.  Restore
 *   the header and its submit/fill paths from git history if the 128B
 *   layout is ever needed again.  Both INI and TGT must speak the same
 *   format (no negotiation).
 *
 *   dword 0~15 are a verbatim copy of the standard NVMe command, so the
 *   struct starts with struct spdk_nvme_cmd and the whole 64B block can
 *   be memcpy'd from a spdk_nvme_cmd directly.
 */

#ifndef NVME_HCOMM_SQE_96B_H
#define NVME_HCOMM_SQE_96B_H

#include "spdk/stdinc.h"
#include "spdk/nvme_spec.h"
#include "spdk/assert.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Fixed wire size of the 96B extended SQE (24 dwords). */
#define NVME_HCOMM_SQE_96B_SIZE 96u

/*
 * Max SGL descriptors carried in the SQE tail array.  Matches
 * NVME_RDMA_MAX_SGL_DESCRIPTORS.  get_max_sges() reports this,
 * and the per-cid xlation record sizes to it.
 */
#define NVME_HCOMM_MAX_SGL 16u

/* Full slot size: base SQE + tail SGL descriptor array.  sq_buf slots
 * are allocated with this stride so any cid can host a max-size
 * multi-SGL SQE; the SEND length stays variable (base + count*16). */
#define NVME_HCOMM_SQE_96B_SLOT_SIZE \
	(NVME_HCOMM_SQE_96B_SIZE + NVME_HCOMM_MAX_SGL * 16u)

/*
 * INI-side JFR token value carried in dword23 of every SQE (nids_aiv_ub
 * kJfrToken convention).  Published in the CM handshake as
 * ub_cm_info.jetty_token so the TGT can import the INI jetty; the same
 * value is echoed per-command so the TGT/DEVICE can validate the source
 * jetty of each Ctrl Send SQE.
 */
#define NVME_HCOMM_INI_JFR_TOKEN 0xACFEu

/*
 * dword 22: routing word.  ctrlid occupies the low half (bit0~15), sqid the
 * high half (bit16~31).  The union provides both the packed value view and
 * the field view on a little-endian host.
 */
union nvme_hcomm_sqe_96b_dword22 {
	struct {
		uint16_t ctrlid; /* bit 0~15  */
		uint16_t sqid;   /* bit 16~31 */
	} bs;
	uint32_t value;
};
SPDK_STATIC_ASSERT(sizeof(union nvme_hcomm_sqe_96b_dword22) == 4,
		   "nvme_hcomm_sqe_96b_dword22 must be 4 bytes");

/*
 * The 96B HCOMM SQE as carried inline in the WQE.
 *
 * - cmd:         standard 64B NVMe command (dword 0~15)
 * - rsvd:        reserved 24B (dword 16~21), must be zero
 * - dword22:     routing word (dword 22), see union above
 * - jetty_token: INI JFR jetty token (dword 23)
 * - sgl:         tail SGL descriptor array (byte 96~), zeroed by the
 *                fill helper, filled by submit for multi-SGL payloads
 */
struct nvme_hcomm_sqe_96b {
	/* dword 0~15: standard NVMe command (byte-for-byte). */
	struct spdk_nvme_cmd	cmd;

	/* dword 16~21: reserved (24 bytes), zeroed by the fill helper. */
	uint32_t		rsvd[6];

	/* dword 22: routing word (ctrlid | sqid). */
	union nvme_hcomm_sqe_96b_dword22	dword22;

	/* dword 23 */
	uint32_t			jetty_token;	/* ini jfr jetty token */

	/* byte 96~: tail SGL descriptor array (up to NVME_HCOMM_MAX_SGL
	 * 16B keyed data blocks; zeroed by the fill helper, filled by
	 * submit for multi-SGL payloads - see the multi-SGL note in the
	 * header). */
	struct spdk_nvme_sgl_descriptor	sgl[NVME_HCOMM_MAX_SGL];
};
SPDK_STATIC_ASSERT(sizeof(struct nvme_hcomm_sqe_96b) == NVME_HCOMM_SQE_96B_SLOT_SIZE,
		   "nvme_hcomm_sqe_96b must be base + tail descriptors");
SPDK_STATIC_ASSERT(offsetof(struct nvme_hcomm_sqe_96b, cmd) == 0,
		   "nvme_hcomm_sqe_96b.cmd must start at offset 0");
SPDK_STATIC_ASSERT(offsetof(struct nvme_hcomm_sqe_96b, rsvd) == 64,
		   "nvme_hcomm_sqe_96b.rsvd must start at dword16 (offset 64)");
SPDK_STATIC_ASSERT(offsetof(struct nvme_hcomm_sqe_96b, dword22) == 88,
		   "nvme_hcomm_sqe_96b.dword22 must start at dword22 (offset 88)");
SPDK_STATIC_ASSERT(offsetof(struct nvme_hcomm_sqe_96b, jetty_token) == 92,
		   "nvme_hcomm_sqe_96b.jetty_token must start at dword23 (offset 92)");
SPDK_STATIC_ASSERT(offsetof(struct nvme_hcomm_sqe_96b, sgl) == NVME_HCOMM_SQE_96B_SIZE,
		   "nvme_hcomm_sqe_96b.sgl must start right after the base SQE (offset 96)");
SPDK_STATIC_ASSERT(sizeof(((struct nvme_hcomm_sqe_96b *)0)->sgl) ==
		   NVME_HCOMM_MAX_SGL * 16,
		   "nvme_hcomm_sqe_96b.sgl must hold NVME_HCOMM_MAX_SGL descriptors");

/*
 * Build a 96B HCOMM SQE: copy the standard 64B NVMe command verbatim,
 * zero the reserved area, then set the routing word and jetty token.
 * The memset zeroes the whole slot including the tail sgl[] array;
 * the submit path fills the descriptors (and the cmd.dptr.sgl1
 * LAST_SEGMENT count mirror) afterwards for multi-SGL payloads.
 *
 *   dst        : 96B SQE slot to fill
 *   cmd        : standard NVMe command (64B, copied whole)
 *   ctrlid     : controller id (dword22 bit 0~15)
 *   sqid       : submission queue id (dword22 bit 16~31)
 *   jetty_token: INI JFR jetty token (dword23)
 */
static inline void
nvme_hcomm_sqe_96b_fill(struct nvme_hcomm_sqe_96b *dst,
			const struct spdk_nvme_cmd *cmd,
			uint16_t ctrlid, uint16_t sqid, uint32_t jetty_token)
{
	memset(dst, 0, sizeof(*dst));

	/* dword 0~15: copy the standard NVMe command verbatim. */
	memcpy(&dst->cmd, cmd, sizeof(*cmd));

	/* dword 22: routing word. */
	dst->dword22.bs.ctrlid = ctrlid;
	dst->dword22.bs.sqid = sqid;

	/* dword 23: jetty token. */
	dst->jetty_token = jetty_token;
}

#ifdef __cplusplus
}
#endif

#endif /* NVME_HCOMM_SQE_96B_H */