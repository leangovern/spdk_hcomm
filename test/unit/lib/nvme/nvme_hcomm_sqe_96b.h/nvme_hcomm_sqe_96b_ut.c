/*   SPDX-License-Identifier: BSD-3-Clause
 *
 *   Standalone unit test for the HCOMM 96B SQE wire format
 *   (lib/nvme/nvme_hcomm_sqe_96b.h).
 *
 *   Zero-dependency: compiled with plain gcc against SPDK public headers.
 *     gcc -I../../../../../include -I../../../../../lib \
 *         -Wall -Wextra -o nvme_hcomm_sqe_96b_ut nvme_hcomm_sqe_96b_ut.c \
 *     && ./nvme_hcomm_sqe_96b_ut
 *
 *   Wire format under test:
 *     Dword  0~15 : standard NVMe 64B SQE (struct spdk_nvme_cmd)
 *     Dword 16~21 : reserved (24 bytes, zero)
 *     Dword 22    : routing word — bit 0~15 = ctrlid, bit 16~31 = sqid
 *     Dword 23    : token id (INI JFR jetty token)
 *     Byte   96~  : tail SGL descriptor array (up to
 *                   NVME_HCOMM_MAX_SGL * 16B, keyed data blocks)
 */

#include "spdk/stdinc.h"
#include "spdk/nvme_spec.h"

#include "nvme/nvme_hcomm_sqe_96b.h"

static int g_failures = 0;

#define CHECK(cond, ...) \
	do { \
		if (!(cond)) { \
			g_failures++; \
			printf("FAIL %s:%d: ", __func__, __LINE__); \
			printf(__VA_ARGS__); \
			printf("\n"); \
		} \
	} while (0)

static void
test_layout(void)
{
	/* Slot = 96B base SQE + NVME_HCOMM_MAX_SGL tail descriptors. */
	CHECK(sizeof(struct nvme_hcomm_sqe_96b) ==
	      96 + NVME_HCOMM_MAX_SGL * 16,
	      "sizeof=%zu expected %u", sizeof(struct nvme_hcomm_sqe_96b),
	      96 + NVME_HCOMM_MAX_SGL * 16);
	CHECK(NVME_HCOMM_SQE_96B_SIZE == 96, "NVME_HCOMM_SQE_96B_SIZE=%u expected 96",
	      (unsigned)NVME_HCOMM_SQE_96B_SIZE);
	CHECK(NVME_HCOMM_SQE_96B_SLOT_SIZE == 96 + NVME_HCOMM_MAX_SGL * 16,
	      "NVME_HCOMM_SQE_96B_SLOT_SIZE=%u expected %u",
	      (unsigned)NVME_HCOMM_SQE_96B_SLOT_SIZE,
	      96 + NVME_HCOMM_MAX_SGL * 16);
	CHECK(NVME_HCOMM_MAX_SGL == 16, "NVME_HCOMM_MAX_SGL=%u expected 16",
	      (unsigned)NVME_HCOMM_MAX_SGL);

	/* Dword 0~15: standard NVMe command at offset 0. */
	CHECK(offsetof(struct nvme_hcomm_sqe_96b, cmd) == 0,
	      "cmd offset=%zu expected 0", offsetof(struct nvme_hcomm_sqe_96b, cmd));

	/* Dword 16~21: reserved at offset 64 (6 dwords, former nsgl slot). */
	CHECK(offsetof(struct nvme_hcomm_sqe_96b, rsvd) == 64,
	      "rsvd offset=%zu expected 64", offsetof(struct nvme_hcomm_sqe_96b, rsvd));
	CHECK(sizeof(((struct nvme_hcomm_sqe_96b *)0)->rsvd) / 4 == 6,
	      "rsvd must cover dword16~21 (6 dwords)");

	/* Dword 22: dword22 at offset 88. */
	CHECK(offsetof(struct nvme_hcomm_sqe_96b, dword22) == 88,
	      "dword22 offset=%zu expected 88", offsetof(struct nvme_hcomm_sqe_96b, dword22));

	/* Dword 23: jetty_token at offset 92. */
	CHECK(offsetof(struct nvme_hcomm_sqe_96b, jetty_token) == 92,
	      "jetty_token offset=%zu expected 92", offsetof(struct nvme_hcomm_sqe_96b, jetty_token));

	/* Base SQE ends at 96 - no tail padding before the descriptor array. */
	CHECK(offsetof(struct nvme_hcomm_sqe_96b, jetty_token) + sizeof(uint32_t) == 96,
	      "tail padding: jetty_token ends at %zu expected 96",
	      offsetof(struct nvme_hcomm_sqe_96b, jetty_token) + sizeof(uint32_t));

	/* Tail SGL descriptor array at offset 96, 16B per descriptor. */
	CHECK(offsetof(struct nvme_hcomm_sqe_96b, sgl) == 96,
	      "sgl offset=%zu expected 96", offsetof(struct nvme_hcomm_sqe_96b, sgl));
	CHECK(sizeof(((struct nvme_hcomm_sqe_96b *)0)->sgl) ==
	      NVME_HCOMM_MAX_SGL * 16,
	      "sgl array size=%zu expected %u",
	      sizeof(((struct nvme_hcomm_sqe_96b *)0)->sgl),
	      NVME_HCOMM_MAX_SGL * 16);

	/* Total slot = base + tail, no trailing padding. */
	CHECK(sizeof(struct nvme_hcomm_sqe_96b) ==
	      offsetof(struct nvme_hcomm_sqe_96b, sgl) +
	      NVME_HCOMM_MAX_SGL * 16,
	      "slot tail padding: size=%zu after sgl=%zu",
	      sizeof(struct nvme_hcomm_sqe_96b),
	      offsetof(struct nvme_hcomm_sqe_96b, sgl) + NVME_HCOMM_MAX_SGL * 16);

	/* No packing needed - natural alignment should produce the slot size. */
	CHECK(__alignof__(struct nvme_hcomm_sqe_96b) == 4 ||
	      __alignof__(struct nvme_hcomm_sqe_96b) == 8,
	      "unexpected alignment=%zu", __alignof__(struct nvme_hcomm_sqe_96b));
}

static void
test_fill_basic(void)
{
	struct nvme_hcomm_sqe_96b sqe;
	struct spdk_nvme_cmd cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opc = 0x01;
	cmd.fuse = 0;
	cmd.cid = 0xABCD;
	cmd.nsid = 0x12345678;
	cmd.cdw10 = 0xAABBCCDD;
	cmd.cdw15 = 0xDEADBEEF;

	nvme_hcomm_sqe_96b_fill(&sqe, &cmd, 0x1A2B, 0x3C4D, 0xACFE);

	/* Check the header copy (first 64 bytes). */
	CHECK(memcmp(&sqe.cmd, &cmd, sizeof(cmd)) == 0,
	      "cmd copy mismatch");

	/* Check opcode/cid/nsid at the expected positions. */
	CHECK(sqe.cmd.opc == 0x01, "opc=%02x expected 01", sqe.cmd.opc);
	CHECK(sqe.cmd.cid == 0xABCD, "cid=%04x expected ABCD", sqe.cmd.cid);
	CHECK(sqe.cmd.nsid == 0x12345678, "nsid=%08x expected 12345678", sqe.cmd.nsid);
	CHECK(sqe.cmd.cdw10 == 0xAABBCCDD, "cdw10=%08x expected AABBCCDD", sqe.cmd.cdw10);
	CHECK(sqe.cmd.cdw15 == 0xDEADBEEF, "cdw15=%08x expected DEADBEEF", sqe.cmd.cdw15);

	/* Check the reserved area (dword 16~21) is zeroed. */
	{
		uint32_t i;
		for (i = 0; i < 6; i++) {
			CHECK(sqe.rsvd[i] == 0,
			      "rsvd[%u]=%08x expected 0", i, sqe.rsvd[i]);
		}
	}

	/* Check routing word (Dword 22). */
	CHECK(sqe.dword22.bs.ctrlid == 0x1A2B,
	      "ctrlid=%04x expected 1A2B", sqe.dword22.bs.ctrlid);
	CHECK(sqe.dword22.bs.sqid == 0x3C4D,
	      "sqid=%04x expected 3C4D", sqe.dword22.bs.sqid);
	CHECK(sqe.dword22.value == 0x3C4D1A2Bu,
	      "dword22.value=%08x expected 3C4D1A2B", sqe.dword22.value);

	/* Check token id (Dword 23). */
	CHECK(sqe.jetty_token == 0xACFE,
	      "jetty_token=%08x expected 0000ACFE", sqe.jetty_token);
}

static void
test_fill_zeros_everywhere(void)
{
	struct nvme_hcomm_sqe_96b sqe;
	struct spdk_nvme_cmd cmd;

	memset(&cmd, 0, sizeof(cmd));
	nvme_hcomm_sqe_96b_fill(&sqe, &cmd, 0, 0, 0);

	/* Every byte must be zero. */
	{
		uint8_t *p = (uint8_t *)&sqe;
		uint32_t i;
		for (i = 0; i < sizeof(sqe); i++) {
			CHECK(p[i] == 0, "byte[%u]=%02x expected 0", i, p[i]);
		}
	}
}

static void
test_routing_token_ids(void)
{
	struct nvme_hcomm_sqe_96b sqe;
	struct spdk_nvme_cmd cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opc = 0x09;
	cmd.cid = 0x5A5A;
	cmd.nsid = 0xCAFEBABE;

	/* Distinct ctrlid/sqid must not bleed into each other. */
	nvme_hcomm_sqe_96b_fill(&sqe, &cmd, 0x0001, 0xFFFF, 0xACFE);
	CHECK(sqe.dword22.bs.ctrlid == 0x0001,
	      "ctrlid=%04x expected 0001", sqe.dword22.bs.ctrlid);
	CHECK(sqe.dword22.bs.sqid == 0xFFFF,
	      "sqid=%04x expected FFFF", sqe.dword22.bs.sqid);

	/* Token id must land in Dword23, not contaminate the NVMe header. */
	CHECK(sqe.jetty_token == 0xACFE,
	      "jetty_token=%08x expected ACFE", sqe.jetty_token);
	CHECK(sqe.cmd.opc == 0x09,
	      "cmd.opc=%02x expected 09", sqe.cmd.opc);
	CHECK(sqe.cmd.cid == 0x5A5A,
	      "cmd.cid=%04x expected 5A5A", sqe.cmd.cid);

	/* Full routing/token round-trip via raw dword view. */
	nvme_hcomm_sqe_96b_fill(&sqe, &cmd, 0x1234, 0x5678, 0xDEADBEEF);
	CHECK(sqe.dword22.value == 0x56781234u,
	      "dword22.value=%08x expected 56781234", sqe.dword22.value);
	CHECK(sqe.jetty_token == 0xDEADBEEF,
	      "jetty_token=%08x expected DEADBEEF", sqe.jetty_token);

	/* The cmd must remain untouched even with non-zero routing/token. */
	CHECK(sqe.cmd.nsid == 0xCAFEBABE,
	      "cmd.nsid=%08x expected CAFEBABE", sqe.cmd.nsid);
}

static void
test_cmd_pointer_aliasing(void)
{
	/* Test that the first 4 bytes of the struct alias the spdk_nvme_cmd
	 * header (opc/fuse/psdt/cid).  This is important for TGT-side parsing
	 * that reads the start of the SQE as a standard NVMe command header.
	 * The routing info (Dword22~23) must not corrupt the cmd. */
	struct nvme_hcomm_sqe_96b sqe;
	struct spdk_nvme_cmd cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opc = 0x09;
	cmd.fuse = 0x01;
	cmd.cid = 0x5A5A;
	cmd.nsid = 0xCAFEBABE;
	nvme_hcomm_sqe_96b_fill(&sqe, &cmd, 0x7A7B, 0xF0F1, 0xACFE);

	/* The first 64 bytes of the struct must equal the NVMe command. */
	uint8_t *sqe_bytes = (uint8_t *)&sqe;
	uint8_t *cmd_bytes = (uint8_t *)&cmd;

	CHECK(memcmp(sqe_bytes, cmd_bytes, 64) == 0,
	      "first 64 bytes of SQE must match standard NVMe header");

	/* Dword22 = ctrlid|sqid = 0xF0F17A7B on LE host */
	uint32_t dword22_raw = *(uint32_t *)(sqe_bytes + 88);
	CHECK(dword22_raw == 0xF0F17A7Bu,
	      "dword22 raw at byte 88 = %08x expected F0F17A7B", dword22_raw);

	/* Dword23 = token = 0x0000ACFE */
	uint32_t dword23_raw = *(uint32_t *)(sqe_bytes + 92);
	CHECK(dword23_raw == 0x0000ACFE,
	      "dword23 raw at byte 92 = %08x expected 0000ACFE", dword23_raw);
}

/* Multi-SGL tail array contract: the transport writes N keyed data
 * block descriptors into the tail array (mirrors nvme_rdma
 * build_sgl_request's cmd->sgl[] loop).  The wire image must be:
 * descriptor i at byte 96+i*16 with the 16B
 * spdk_nvme_sgl_descriptor layout (address LE at +0, 24-bit length
 * at +8, 32-bit key at +11, subtype|type nibbles at +15), the
 * unused tail zeroed by the fill helper, dword 16~21 reserved and
 * zero, and the routing word / token / cid untouched. */
static void
test_tail_sgl_array(void)
{
	struct nvme_hcomm_sqe_96b sqe;
	struct spdk_nvme_cmd cmd;
	uint32_t i;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opc = 0x01;
	cmd.cid = 0x7777;
	nvme_hcomm_sqe_96b_fill(&sqe, &cmd, 0x1A2B, 0x3C4D, 0xACFE);

	/* Dword 16~21 must stay zero (the former nsgl slot is reserved). */
	{
		uint8_t *p = (uint8_t *)&sqe + offsetof(struct nvme_hcomm_sqe_96b, rsvd);

		for (i = 0; i < sizeof(sqe.rsvd); i++) {
			CHECK(p[i] == 0, "rsvd byte[%u]=%02x expected 0 (dword 16~21)",
			      i, p[i]);
		}
	}

	for (i = 0; i < 3; i++) {
		struct spdk_nvme_sgl_descriptor d;

		memset(&d, 0, sizeof(d));
		d.keyed.type = SPDK_NVME_SGL_TYPE_KEYED_DATA_BLOCK;
		d.keyed.subtype = SPDK_NVME_SGL_SUBTYPE_ADDRESS;
		d.keyed.key = 0x5A5A0000 + i;
		d.keyed.length = 4096 * (i + 1);
		d.address = 0x1122334455660000ULL * (i + 1);
		sqe.sgl[i] = d;
	}

	/* Descriptor wire image at 96 + i*16. */
	for (i = 0; i < 3; i++) {
		uint8_t *p = (uint8_t *)&sqe + NVME_HCOMM_SQE_96B_SIZE +
			     i * sizeof(sqe.sgl[0]);
		uint64_t addr = *(uint64_t *)p;
		uint32_t len24 = *(uint32_t *)(p + 8) & 0xFFFFFFu;
		uint32_t key = *(uint32_t *)(p + 11);
		uint8_t ts = p[15];

		CHECK(addr == 0x1122334455660000ULL * (i + 1),
		      "sgl[%u] address=%llx expected %llx", i,
		      (unsigned long long)addr,
		      (unsigned long long)(0x1122334455660000ULL * (i + 1)));
		CHECK(len24 == 4096 * (i + 1),
		      "sgl[%u] length=%u expected %u", i, len24, 4096 * (i + 1));
		CHECK(key == 0x5A5A0000 + i,
		      "sgl[%u] key=%08x expected %08x", i, key, 0x5A5A0000 + i);
		CHECK(ts == (uint8_t)((SPDK_NVME_SGL_TYPE_KEYED_DATA_BLOCK << 4) |
				      SPDK_NVME_SGL_SUBTYPE_ADDRESS),
		      "sgl[%u] type/subtype byte=%02x expected %02x", i, ts,
		      (uint8_t)((SPDK_NVME_SGL_TYPE_KEYED_DATA_BLOCK << 4) |
				SPDK_NVME_SGL_SUBTYPE_ADDRESS));
	}

	/* Tail beyond the 3 descriptors stays zero (fill helper zeroed it). */
	{
		uint8_t *p = (uint8_t *)&sqe + offsetof(struct nvme_hcomm_sqe_96b, sgl) +
			     3 * sizeof(sqe.sgl[0]);

		for (i = 0; i < (NVME_HCOMM_MAX_SGL - 3) * sizeof(sqe.sgl[0]); i++) {
			CHECK(p[i] == 0, "tail byte[%u]=%02x expected 0 (desc 3..15)", i, p[i]);
		}
	}

	/* Route word and token must be untouched by the tail writes. */
	CHECK(sqe.dword22.value == 0x3C4D1A2Bu,
	      "dword22.value=%08x expected 3C4D1A2B", sqe.dword22.value);
	CHECK(sqe.jetty_token == 0xACFE, "jetty_token=%08x expected ACFE",
	      sqe.jetty_token);
	CHECK(sqe.cmd.cid == 0x7777, "cmd.cid=%04x expected 7777", sqe.cmd.cid);
}

/* Multi-SGL count encoding: the DPTR carries a LAST_SEGMENT+OFFSET
 * descriptor whose 24-bit length = N*16 - the one and only count
 * encoding (standard NVMe-oF "SGL in capsule" convention, same as
 * nvme_rdma build_sgl_request).  The TGT derives N = length/16 and
 * bounds it by NVME_HCOMM_MAX_SGL.  The mirror lands in cmd.dptr.sgl1
 * (the standard NVMe DPTR offset) with the same 16B descriptor wire
 * layout. */
static void
test_multi_sgl_count_mirror(void)
{
	struct nvme_hcomm_sqe_96b sqe;
	struct spdk_nvme_sgl_descriptor ls;
	struct spdk_nvme_cmd cmd;
	uint8_t *p;
	uint32_t len24;
	uint32_t n;

	memset(&cmd, 0, sizeof(cmd));
	cmd.opc = 0x01;
	nvme_hcomm_sqe_96b_fill(&sqe, &cmd, 0x1A2B, 0x3C4D, 0xACFE);
	p = (uint8_t *)&sqe + offsetof(struct nvme_hcomm_sqe_96b, cmd.dptr.sgl1);

	/* Build the count mirror exactly like the submit path does. */
	memset(&ls, 0, sizeof(ls));
	ls.unkeyed.type = SPDK_NVME_SGL_TYPE_LAST_SEGMENT;
	ls.unkeyed.subtype = SPDK_NVME_SGL_SUBTYPE_OFFSET;
	ls.unkeyed.length = 3 * sizeof(ls);
	ls.address = 0;
	sqe.cmd.dptr.sgl1 = ls;

	/* Mirror wire image at the standard DPTR offset (byte 24). */
	CHECK(*(uint64_t *)p == 0, "count mirror address=%llx expected 0",
	      (unsigned long long)*(uint64_t *)p);
	len24 = *(uint32_t *)(p + 8) & 0xFFFFFFu;
	CHECK(len24 == 3 * sizeof(ls), "count mirror length=%u expected %u",
	      len24, 3 * (uint32_t)sizeof(ls));
	CHECK(p[15] == (uint8_t)((SPDK_NVME_SGL_TYPE_LAST_SEGMENT << 4) |
				 SPDK_NVME_SGL_SUBTYPE_OFFSET),
	      "count mirror type/subtype byte=%02x expected %02x", p[15],
	      (uint8_t)((SPDK_NVME_SGL_TYPE_LAST_SEGMENT << 4) |
			SPDK_NVME_SGL_SUBTYPE_OFFSET));

	/* TGT-side derivation: N = length/16, within the slot capacity. */
	n = len24 / (uint32_t)sizeof(ls);
	CHECK(n == 3, "derived segment count=%u expected 3", n);
	CHECK(n <= NVME_HCOMM_MAX_SGL, "derived segment count=%u exceeds %u",
	      n, (unsigned)NVME_HCOMM_MAX_SGL);

	/* Full capacity: 16 descriptors still fit the 24-bit length field
	 * and the derived count hits the bound exactly. */
	ls.unkeyed.length = NVME_HCOMM_MAX_SGL * sizeof(ls);
	sqe.cmd.dptr.sgl1 = ls;
	len24 = *(uint32_t *)(p + 8) & 0xFFFFFFu;
	n = len24 / (uint32_t)sizeof(ls);
	CHECK(n == NVME_HCOMM_MAX_SGL, "max derived count=%u expected %u",
	      n, (unsigned)NVME_HCOMM_MAX_SGL);
}

int main(void)
{
	printf("=== nvme_hcomm_sqe_96b_ut ===\n");

	test_layout();
	test_fill_basic();
	test_fill_zeros_everywhere();
	test_routing_token_ids();
	test_cmd_pointer_aliasing();
	test_tail_sgl_array();
	test_multi_sgl_count_mirror();

	printf("=== %s ===\n", g_failures ? "FAILED" : "PASSED");
	return g_failures ? 1 : 0;
}