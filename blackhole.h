// SPDX-FileCopyrightText: © 2023 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only

#ifndef TTDRIVER_BLACKHOLE_H_INCLUDED
#define TTDRIVER_BLACKHOLE_H_INCLUDED

#include <linux/types.h>
#include <linux/workqueue.h>
#include "device.h"

// Log entry header - 12 bytes total
struct fw_log_entry_header {
	u16 msg_size;     // Size of log message including header
	u8 log_level;     // FW log level
	u8 source;        // 0=SMC, 1=DMC
	u32 timestamp;    // FW timestamp
	u32 sequence;     // Sequence number
} __packed;

// Buffer header at start of shared buffer
struct fw_log_buffer_header {
	u32 write_offset; // Current write position (circular)
	u32 buffer_size;  // Total buffer size
	u32 magic;        // Magic number for validation
	u8 owner;         // Buffer owner: FW or host
	u8 reserved[3];   // Future use
} __packed;

// Log level mappings (match Zephyr standard levels)
#define FW_LOG_LEVEL_NONE    0
#define FW_LOG_LEVEL_ERROR   1
#define FW_LOG_LEVEL_WARN    2
#define FW_LOG_LEVEL_INFO    3
#define FW_LOG_LEVEL_DEBUG   4

#define FW_LOG_SOURCE_SMC    0
#define FW_LOG_SOURCE_DMC    1

#define FW_LOG_BUFFER_MAGIC      0x544C4F47  // "TLOG"
#define FW_LOG_BUFFER_OWNER_FW   0x0         // Firmware owns buffer, host has consumed
#define FW_LOG_BUFFER_OWNER_HOST 0x1         // Host owns buffer, firmware has produced data
#define FW_LOG_BUFFER_SIZE   4096

struct blackhole_device {
	struct tenstorrent_device tt;

	struct mutex kernel_tlb_mutex;	// Guards access to kernel_tlb
	u8 __iomem *tlb_regs;   // All TLB registers
	u8 __iomem *kernel_tlb; // Topmost 2M window, reserved for kernel
	u8 __iomem *noc2axi_cfg;
	u8 __iomem *bar2_mapping;

	u8 saved_mps;

	bool pcie_perf_group_registered;
	bool telemetry_group_registered;

	// FW tt_pcie_log support
	void *log_buffer_virt;    // Virtual address of log buffer
	dma_addr_t log_buffer_dma; // DMA address for FW
	struct work_struct log_work; // Work queue for log processing
	u32 last_read_offset;     // Last processed position
	u32 expected_sequence;    // Expected next sequence number
	bool tt_pcie_log_enabled;     // Whether tt_pcie_log is active
};

#define tt_dev_to_bh_dev(ttdev) \
	container_of((tt_dev), struct blackhole_device, tt)

#endif
