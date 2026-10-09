#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) 2015 Intel Corporation.
#  All rights reserved.
#

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../..)
include $(SPDK_ROOT_DIR)/mk/spdk.common.mk

SO_VER := 12
SO_MINOR := 0

C_SRCS = nvme_ctrlr_cmd.c nvme_ctrlr.c nvme_fabric.c nvme_ns_cmd.c \
	nvme_ns.c nvme_pcie_common.c nvme_pcie.c nvme_qpair.c nvme.c \
	nvme_quirks.c nvme_transport.c nvme_discovery.c \
	nvme_ctrlr_ocssd_cmd.c nvme_ns_ocssd_cmd.c nvme_tcp.c \
	nvme_opal.c nvme_io_msg.c nvme_poll_group.c nvme_zns.c \
	nvme_cuse.c
ifeq ($(OS),Linux)
C_SRCS += nvme_vfio_user.c
endif
C_SRCS-$(CONFIG_RDMA) += nvme_rdma.c
C_SRCS-$(CONFIG_HCOMM) += nvme_hcomm.c

# hcomm_res_defs.h uses static const for COMM_ADDR_EID_LEN, which is not a
# compile-time constant in C.  Create a build-time fixed copy in build/ so
# the compiler picks it up (via -I) before the installed header.
HCOMM_HCOMPAT_DIR = $(SPDK_ROOT_DIR)/build/hcompat
ifeq ($(CONFIG_HCOMM),y)
$(shell mkdir -p $(HCOMM_HCOMPAT_DIR))
$(shell sed 's/static const uint32_t COMM_ADDR_EID_LEN = 16U;/enum { COMM_ADDR_EID_LEN = 16U };/' \
	$(CONFIG_HCOMM_DIR)/include/hcomm/hcomm_res_defs.h > $(HCOMM_HCOMPAT_DIR)/hcomm_res_defs.h 2>/dev/null || true)
CFLAGS += -I$(HCOMM_HCOMPAT_DIR)
endif

LIBNAME = nvme
LOCAL_SYS_LIBS =
ifeq ($(CONFIG_RDMA),y)
LOCAL_SYS_LIBS += -libverbs -lrdmacm
#Attach only if FreeBSD and RDMA is specified with configure
ifeq ($(OS),FreeBSD)
# Mellanox - MLX4 HBA Userspace Library
ifneq ("$(wildcard /usr/lib/libmlx4.*)","")
LOCAL_SYS_LIBS += -lmlx4
endif
# Mellanox - MLX5 HBA Userspace Library
ifneq ("$(wildcard /usr/lib/libmlx5.*)","")
LOCAL_SYS_LIBS += -lmlx5
endif
# Chelsio HBA Userspace Library
ifneq ("$(wildcard /usr/lib/libcxgb4.*)","")
LOCAL_SYS_LIBS += -lcxgb4
endif
endif
endif

ifeq ($(CONFIG_HCOMM),y)
# hcomm library path handling (include/lib/rpath) is configured globally in
# mk/spdk.common.mk so that both the nvme shared library build and the final
# application link can locate libhcomm. LOCAL_SYS_LIBS is needed here because
# the shared library link step (spdk_build_realname_shared_lib) only consumes
# LOCAL_SYS_LIBS, not the global SYS_LIBS.
LOCAL_SYS_LIBS += -lhcomm -lascendcl -lc_sec
endif

ifeq ($(CONFIG_NVME_CUSE),y)
# fuse requires to set _FILE_OFFSET_BITS to 64 bits even for 64 bit machines
CFLAGS += -D_FILE_OFFSET_BITS=64
LOCAL_SYS_LIBS += -lfuse3
endif

CFLAGS += -Wpointer-arith

SPDK_MAP_FILE = $(abspath $(CURDIR)/spdk_nvme.map)

include $(SPDK_ROOT_DIR)/mk/spdk.lib.mk
