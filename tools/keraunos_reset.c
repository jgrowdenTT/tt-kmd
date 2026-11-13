// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only
//
// Keraunos reset helper via TENSTORRENT_IOCTL_NOC_READ/_NOC_WRITE.
//
// Behavior:
//   keraunos_reset 0 -> hold SMC RISC-V core in reset
//   keraunos_reset 1 -> release SMC RISC-V core from reset
//   keraunos_reset --kbl1 <image.bin> -> hold reset, load image, release reset
//
// Addressing:
//   SMC_CPU_SMC_CPU_CTRL_RESET_CTRL_REG_ADDR
//     global: 0x08010020
//     local : 0xC0010020
//     SPA   : 0x1202010020
//
//   SMC_CPU_SMC_CPU_CTRL_RESET_VECTOR_0_REG_ADDR
//     global: 0x08010000
//     local : 0xC0010000
//     SPA   : 0x1202010000
//
// Build:
//   gcc -O2 -Wall -Wextra -o keraunos_reset keraunos_reset.c
//
// Run:
//   ./keraunos_reset <0|1> [device_id]
//   ./keraunos_reset --sysbuild <dir> --topology <mmk|qsr1a1> [--skip-bl0p5-load] [device_id]
//
// --topology selects the chip family the sysbuild images are staged for:
//   mmk    : BL0 wipes M0, M1, K; BUN2 = kbl1[0], mbl1[1], d2d[2];
//            BUN3 = kmis[0], mmis[1]
//   qsr1a1 : BL0 wipes M1, M2, Q0, K; BUN2 = kbl1[0], mbl1[1], d2d[2], qbl1[4];
//            BUN3 = kmis[0], mmis[1], qmis[2]
//

#include <dirent.h>
#include <pwd.h>
#include <errno.h>
#include <ctype.h>
#include <time.h>
#include <fcntl.h>
#include <linux/types.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

#define TENSTORRENT_IOCTL_MAGIC 0xFA
#define TENSTORRENT_IOCTL_GET_DEVICE_INFO _IO(TENSTORRENT_IOCTL_MAGIC, 0)
#define TENSTORRENT_IOCTL_NOC_READ        _IO(TENSTORRENT_IOCTL_MAGIC, 18)
#define TENSTORRENT_IOCTL_NOC_WRITE       _IO(TENSTORRENT_IOCTL_MAGIC, 19)
#define TENSTORRENT_IOCTL_DEV_NOC_READ_BLOCK  _IO(TENSTORRENT_IOCTL_MAGIC, 0x81)
#define TENSTORRENT_IOCTL_DEV_NOC_WRITE_BLOCK _IO(TENSTORRENT_IOCTL_MAGIC, 0x82)

#define TENSTORRENT_PCI_VENDOR_ID 0x1e52
#define KERAUNOS_PCI_DEVICE_ID    0xfeed

#define KER_SMC_CORE_LOCAL_BASE 0xC0000000ULL
#define KER_SPA_BASE_K          0x1202000000ULL
#define KER_SPA_BASE_M0         0x1300000000ULL
#define KER_SPA_BASE_M1         0x1308000000ULL
#define KER_SPA_BASE_M2         0x1310000000ULL
#define KER_SPA_BASE_M3         0x1318000000ULL
#define KER_SPA_BASE_Q0         0x19D8000000ULL
#define KER_RESET_CTRL_OFFSET   0x10020ULL
#define KER_RESET_VECTOR0_OFFSET 0x10000ULL
#define KER_SCRATCH_BASE_OFFSET 0x10100ULL
#define KER_SCRATCH_STRIDE      0x8ULL
#define KER_SCRATCH_DUMP_COUNT  4u
#define KER_SCRATCH_CLEAR_COUNT 16u  /* covers indices 0-15, above our highest use of [12] */
/* BL0 owns SRAM below 0xC0066400. */
#define KER_SMC_BL1_RESET_VECTOR 0xC0066400ULL
#define KER_SMC_BL0_RESET_VECTOR 0xC0040000ULL
/* BL1 execute address: 128KB from the end of ram0 (0xC015b000 - 0x20000). */
#define KER_SMC_BL1_EXEC_ADDR    0xC013B000ULL
#define KER_SMC_SRAM_BASE 0xC0060000ULL
#define KER_SMC_SRAM_SIZE 0x00100000ULL
/* Bundle staging area is the upper 512KB of SRAM */
#define KER_SMC_BUNDLE_STAGING_BASE (KER_SMC_SRAM_BASE + 0x80000ULL)
/* KeraunosSmcCpu_ResetCtrl_reg_t.core0_reset_n_n0_scan */
#define KER_RESET_CTRL_CORE0_RESET_N_BIT 0u

/* BL0P5 execute location: 64KB from end of ram0 (0xC0160000 - 0x10000) */
#define KER_SMC_BL0P5_LOAD_ADDR         0xC0150000ULL
#define KER_SMC_KMIS_LOAD_ADDR          0xC0067000ULL
/* Scratch registers used for the BL0P5 <-> host handshake (local addresses) */
#define KER_HOST_BOOT_STATE_LOCAL        0xC0010160ULL  /* SCRATCH[12] */
#define KER_BUNDLE_VALIDATION_LOCAL      0xC0010150ULL  /* SCRATCH[10] */
#define KER_SMC_SCRATCH_SMC_STATUS_TO_SEP 0xC0010148ULL /* SCRATCH[9]  */
#define KER_SEP_SAFE_OFFSET_SCRATCH      13u
#define KER_SEP_SAFE_SIZE_SCRATCH        14u
#define KER_SEP_SAFE_OFFSET              0u
#define KER_SEP_SAFE_SIZE                (25u * 1024u)
/* SEP handshake values */
#define SMC_SEP_STATUS_SRAM_INIT_BIT 0      /* SMC SRAM initialized */
#define SMC_SEP_STATUS_MANIFEST_READY_BIT 1 /* Manifest ready */
#define SMC_SEP_STATUS_BUFFER_READY_BIT 2   /* Status buffer ready */

#define SMC_SEP_STATUS_SRAM_INIT (1U << SMC_SEP_STATUS_SRAM_INIT_BIT)
#define SMC_SEP_STATUS_MANIFEST_READY (1U << SMC_SEP_STATUS_MANIFEST_READY_BIT)
#define SMC_SEP_STATUS_BUFFER_READY (1U << SMC_SEP_STATUS_BUFFER_READY_BIT)
/* Handshake values written to the host-boot-state scratch register */
#define HOST_BOOT_STATE_WAIT_FOR_BUNDLE  1u
#define HOST_BOOT_STATE_BUNDLE_STAGED    2u
/* Bits in the bundle-validation scratch register */
#define BUNDLE_READY_FOR_VALIDATION_BIT  0x1u
#define BUNDLE_VALIDATED_BIT             0x2u
/* fw_bundle_manifest/toc layout constants (from tt_bundle_loader.h) */
#define BUNDLE_MANIFEST_SIZE             1184u
#define BUNDLE_MANIFEST_PAYLOAD_OFF_OFF  1160u   /* byte offset of payload_offset in manifest */
#define BUNDLE_TOC_ID                    0x434f5450u  /* "PTOC" */
#define BUNDLE_TOC_HDR_SIZE              32u
#define BUNDLE_TOC_VERSION_MAJOR         1u
#define BUNDLE_TOC_ENTRY_SIZE            216u
#define BUNDLE_TOC_IMG_TYPE_BL1_LO       0x42434d53u  /* low  32b of FW_BUNDLE_IMG_TYPE_SMC_BL1 */
#define BUNDLE_TOC_IMG_TYPE_BL1_HI       0x0000314cu  /* high 32b of FW_BUNDLE_IMG_TYPE_SMC_BL1 */
#define BUNDLE_TOC_IMG_TYPE_MIS_LO       0x42434d53u  /* low  32b of FW_BUNDLE_IMG_TYPE_SMC_MIS */
#define BUNDLE_TOC_IMG_TYPE_MIS_HI       0x0000324cu  /* high 32b of FW_BUNDLE_IMG_TYPE_SMC_MIS */
/* Quasar image slots. Mimir uses BUN2 0/1/2 and BUN3 0/1, so Quasar takes the next free
 * slots; BUN2 slot 3 is the GDDR params slot in the chiplet-boot overlays, so skip it. */
#define BUN2_QUASAR_BL1_TOC_INDEX        4u
#define BUN3_QUASAR_MIS_TOC_INDEX        2u
#define BUNDLE_POLL_TIMEOUT_US           100000000u  /* 100 s */

struct tenstorrent_get_device_info {
    struct {
        __u32 output_size_bytes;
    } in;
    struct {
        __u32 output_size_bytes;
        __u16 vendor_id;
        __u16 device_id;
        __u16 subsystem_vendor_id;
        __u16 subsystem_id;
        __u16 bus_dev_fn;
        __u16 max_dma_buf_size_log2;
        __u16 pci_domain;
        __u16 reserved;
    } out;
};

struct tenstorrent_noc_io {
    __u32 argsz;
    __u32 flags;
    __u16 x;
    __u16 y;
    __u8 noc;
    __u8 width;
    __u8 reserved0[2];
    __u64 addr;
    __u64 value;
};

struct tenstorrent_noc_block_io {
    __u32 argsz;
    __u32 flags;
    __u16 x;
    __u16 y;
    __u8 noc;
    __u8 reserved0[3];
    __u64 addr;
    __u64 data_ptr;
    __u64 data_len;
    __u64 bytes_transferred;
};

static uint64_t g_spa_base = KER_SPA_BASE_K;

static int read32_ioctl(int fd, uint64_t spa, uint32_t *value)
{
    struct tenstorrent_noc_io io = {0};

    io.argsz = sizeof(io);
    io.width = sizeof(uint32_t);
    io.addr = spa;
    if (ioctl(fd, TENSTORRENT_IOCTL_NOC_READ, &io) < 0) {
        return -errno;
    }

    *value = (uint32_t)io.value;
    return 0;
}

static int write32_ioctl(int fd, uint64_t spa, uint32_t value)
{
    struct tenstorrent_noc_io io = {0};

    io.argsz = sizeof(io);
    io.width = sizeof(uint32_t);
    io.addr = spa;
    io.value = value;
    if (ioctl(fd, TENSTORRENT_IOCTL_NOC_WRITE, &io) < 0) {
        return -errno;
    }

    return 0;
}

static int write64_ioctl(int fd, uint64_t spa, uint64_t value)
{
    struct tenstorrent_noc_io io = {0};

    io.argsz = sizeof(io);
    io.width = sizeof(uint64_t);
    io.addr = spa;
    io.value = value;
    if (ioctl(fd, TENSTORRENT_IOCTL_NOC_WRITE, &io) < 0) {
        return -errno;
    }

    return 0;
}

static int read64_ioctl(int fd, uint64_t spa, uint64_t *value)
{
    struct tenstorrent_noc_io io = {0};

    io.argsz = sizeof(io);
    io.width = sizeof(uint64_t);
    io.addr = spa;
    if (ioctl(fd, TENSTORRENT_IOCTL_NOC_READ, &io) < 0) {
        return -errno;
    }

    *value = io.value;
    return 0;
}

static int bulk64_ioctl(int fd, unsigned long req, uint64_t spa, const void *buf, size_t len)
{
    struct tenstorrent_noc_block_io io = {0};

    io.argsz = sizeof(io);
    io.addr = spa;
    io.data_ptr = (uintptr_t)buf;
    io.data_len = len;
    if (ioctl(fd, req, &io) < 0) {
        return -errno;
    }

    return 0;
}

/*
 * Probe for the block ioctls without touching the device: a NULL buffer passes request validation
 * and then fails page pinning with EFAULT. Older kmds reject the unknown ioctl (EINVAL/ENOTTY) and
 * get the 64-bit scalar fallback.
 */
static int has_bulk(int fd)
{
    static int supported = -1;

    if (supported < 0) {
        supported = (bulk64_ioctl(fd, TENSTORRENT_IOCTL_DEV_NOC_WRITE_BLOCK, g_spa_base, NULL, 4) == -EFAULT);
    }
    return supported;
}

/* spa 8-aligned, len a multiple of 8. */
static int write_words64(int fd, uint64_t spa, const void *buf, size_t len)
{
    if (has_bulk(fd)) {
        return bulk64_ioctl(fd, TENSTORRENT_IOCTL_DEV_NOC_WRITE_BLOCK, spa, buf, len);
    }
    for (size_t off = 0; off < len; off += 8) {
        uint64_t v;
        int rc;

        memcpy(&v, (const uint8_t *)buf + off, sizeof(v));
        rc = write64_ioctl(fd, spa + off, v);
        if (rc) {
            return rc;
        }
    }
    return 0;
}

/* spa 8-aligned, len a multiple of 8. */
static int read_words64(int fd, uint64_t spa, void *buf, size_t len)
{
    if (has_bulk(fd)) {
        return bulk64_ioctl(fd, TENSTORRENT_IOCTL_DEV_NOC_READ_BLOCK, spa, buf, len);
    }
    for (size_t off = 0; off < len; off += 8) {
        uint64_t v;
        int rc = read64_ioctl(fd, spa + off, &v);

        if (rc) {
            return rc;
        }
        memcpy((uint8_t *)buf + off, &v, sizeof(v));
    }
    return 0;
}

static void print_progress_bar(off_t done, off_t total);

/*
 * Write len bytes (a multiple of 4) to 4-aligned spa: 32-bit writes for an unaligned first/last
 * word, 64-bit bulk writes in between. Optionally draws a progress bar.
 */
static int write_words(int fd, uint64_t spa, const uint8_t *data, size_t len, int progress)
{
    const size_t chunk = 64 * 1024;
    size_t done = 0;
    size_t mid;
    uint32_t w;
    int rc;

    if ((spa & 0x3ULL) || (len & 0x3u)) {
        return -EINVAL;
    }

    if ((spa & 0x7ULL) && len >= 4) {
        memcpy(&w, data, 4);
        rc = write32_ioctl(fd, spa, w);
        if (rc) {
            return rc;
        }
        done = 4;
    }

    mid = done + ((len - done) & ~(size_t)0x7);
    while (done < mid) {
        size_t n = (mid - done < chunk) ? (mid - done) : chunk;

        rc = write_words64(fd, spa + done, data + done, n);
        if (rc) {
            return rc;
        }
        done += n;
        if (progress) {
            print_progress_bar((off_t)done, (off_t)len);
        }
    }

    if (done < len) {
        memcpy(&w, data + done, 4);
        rc = write32_ioctl(fd, spa + done, w);
        if (rc) {
            return rc;
        }
    }
    if (progress) {
        print_progress_bar((off_t)len, (off_t)len);
    }
    return 0;
}

static int zero_words(int fd, uint64_t spa, size_t len)
{
    uint8_t *zeros = calloc(1, len ? len : 1);
    int rc;

    if (!zeros) {
        return -ENOMEM;
    }
    rc = write_words(fd, spa, zeros, len, 0);
    free(zeros);
    return rc;
}

/* Read bytes [spa, spa + len) via 8-aligned bulk reads. */
static int read_bytes(int fd, uint64_t spa, uint8_t *out, size_t len)
{
    uint64_t start = spa & ~0x7ULL;
    size_t span = (size_t)(((spa + len + 7) & ~0x7ULL) - start);
    uint8_t *tmp;
    int rc;

    if (len == 0) {
        return 0;
    }
    tmp = malloc(span);
    if (!tmp) {
        return -ENOMEM;
    }
    rc = read_words64(fd, start, tmp, span);
    if (!rc) {
        memcpy(out, tmp + (spa - start), len);
    }
    free(tmp);
    return rc;
}

/* Read a regular file into a malloc'd buffer zero-padded to a multiple of 4 bytes. */
static int read_image_file(const char *image_path, uint8_t **buf, size_t *size)
{
    struct stat st;
    size_t got = 0;
    uint8_t *data;
    int rc;
    int image_fd = open(image_path, O_RDONLY);

    if (image_fd < 0) {
        fprintf(stderr, "open image %s failed: %s\n", image_path, strerror(errno));
        return -errno;
    }
    if (fstat(image_fd, &st) < 0) {
        rc = -errno;
        fprintf(stderr, "stat image %s failed: %s\n", image_path, strerror(errno));
        close(image_fd);
        return rc;
    }
    if (!S_ISREG(st.st_mode)) {
        fprintf(stderr, "image %s is not a regular file\n", image_path);
        close(image_fd);
        return -EINVAL;
    }

    data = calloc(1, ((size_t)st.st_size + 3) & ~(size_t)3);
    if (!data && st.st_size) {
        close(image_fd);
        return -ENOMEM;
    }
    while (got < (size_t)st.st_size) {
        ssize_t n = read(image_fd, data + got, (size_t)st.st_size - got);

        if (n <= 0) {
            rc = n < 0 ? -errno : -EIO;
            fprintf(stderr, "read image %s failed at offset %zu: %s\n", image_path, got,
                    n < 0 ? strerror(errno) : "short read");
            free(data);
            close(image_fd);
            return rc;
        }
        got += (size_t)n;
    }

    close(image_fd);
    *buf = data;
    *size = got;
    return 0;
}

static int open_tt_dev(const char *path, uint16_t expected_device_id)
{
    int fd;
    struct tenstorrent_get_device_info info = {
        .in.output_size_bytes = sizeof(info.out),
    };

    fd = open(path, O_RDWR | O_APPEND);
    if (fd < 0) {
        return -errno;
    }

    if (ioctl(fd, TENSTORRENT_IOCTL_GET_DEVICE_INFO, &info) < 0) {
        int rc = -errno;
        close(fd);
        return rc;
    }

    if (info.out.vendor_id != TENSTORRENT_PCI_VENDOR_ID ||
        info.out.device_id != expected_device_id) {
        close(fd);
        return -ENODEV;
    }

    return fd;
}

enum topology {
    TOPO_NONE = 0,
    TOPO_MMK,
    TOPO_QSR1A1,
};

static int parse_topology_arg(const char *arg, enum topology *topo)
{
    if (arg == NULL) {
        return -EINVAL;
    }

    if (!strcasecmp(arg, "mmk")) {
        *topo = TOPO_MMK;
    } else if (!strcasecmp(arg, "qsr1a1")) {
        *topo = TOPO_QSR1A1;
    } else {
        return -EINVAL;
    }
    return 0;
}

static void usage(const char *prog)
{
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "  %s --sysbuild <sysbuild_dir> --topology <mmk|qsr1a1> [--skip-bl0p5-load] [device_id]\n", prog);
    fprintf(stderr, "  %s <k|m0|m1|m2|m3|q0> <0|1> [device_id]\n", prog);
    fprintf(stderr, "  %s <k|m0|m1|m2|m3|q0> --kbl1 <build_dir> [-r <hex_addr>] [device_id]\n", prog);
    fprintf(stderr, "  %s <k|m0|m1|m2|m3|q0> --blop5 --sysbuild <sysbuild_dir> --topology <mmk|qsr1a1> [device_id]\n", prog);
    fprintf(stderr, "  %s <k|m0|m1|m2|m3|q0> --bl0 [device_id]\n", prog);
    fprintf(stderr, "  k = SPA base 0x12020..., m0..m3 = SPA base 0x1300/0x1308/0x1310/0x1318..., m = m1, q0 = SPA base 0x19D8... (Q0)\n");
    fprintf(stderr, "  0 = hold SMC RISC-V in reset\n");
    fprintf(stderr, "  1 = release SMC RISC-V from reset\n");
    fprintf(stderr, "  --kbl1 = hold reset, load <build_dir>/zephyr/zephyr.bin, release reset\n");
    fprintf(stderr, "  -r <hex_addr> = (with -i/--kbl1 only) load/execute from this local address instead of the default\n");
    fprintf(stderr, "  --blop5 --sysbuild = boot blop5 using all images from a sysbuild directory\n");
    fprintf(stderr, "  --topology = required with --sysbuild; mmk (Mimir) or qsr1a1 (Quasar) image set\n");
    fprintf(stderr, "  --skip-bl0p5-load = (with --sysbuild) don't launch K BL0/BL0P5; start at the bundle-ready poll\n");
    fprintf(stderr, "  --bl0 = hold reset, wipe SRAM, set RESET_VECTOR[0] to 0xC0040000, release reset\n");
    fprintf(stderr, "Examples:\n");
    fprintf(stderr, "  %s k 0\n", prog);
    fprintf(stderr, "  %s k 1 3\n", prog);
    fprintf(stderr, "  %s m --kbl1 build\n", prog);
    fprintf(stderr, "  %s --sysbuild build --topology mmk\n", prog);
    fprintf(stderr, "  %s k --blop5 --sysbuild build --topology qsr1a1\n", prog);
}

static int parse_mode_arg(const char *arg)
{
    if (arg == NULL) {
        return -EINVAL;
    }

    if (!strcasecmp(arg, "k")) {
        g_spa_base = KER_SPA_BASE_K;
    } else if (!strcasecmp(arg, "m0")) {
        g_spa_base = KER_SPA_BASE_M0;
    } else if (!strcasecmp(arg, "m") || !strcasecmp(arg, "m1")) {
        g_spa_base = KER_SPA_BASE_M1;
    } else if (!strcasecmp(arg, "m2")) {
        g_spa_base = KER_SPA_BASE_M2;
    } else if (!strcasecmp(arg, "m3")) {
        g_spa_base = KER_SPA_BASE_M3;
    } else if (!strcasecmp(arg, "q0")) {
        g_spa_base = KER_SPA_BASE_Q0;
    } else {
        return -EINVAL;
    }
    return 0;
}

static uint64_t reset_ctrl_spa(void)
{
    return g_spa_base + KER_RESET_CTRL_OFFSET;
}

static uint64_t reset_vector0_spa(void)
{
    return g_spa_base + KER_RESET_VECTOR0_OFFSET;
}

static uint64_t local_addr_to_spa(uint64_t addr);

static int set_reset_vector(int fd, uint64_t reset_vector)
{
    int rc;
    uint32_t readback = 0;

    rc = write32_ioctl(fd, reset_vector0_spa(), (uint32_t)reset_vector);
    if (rc) {
        fprintf(stderr, "write RESET_VECTOR[0] (0x%012llx) failed: %s\n",
                (unsigned long long)reset_vector0_spa(), strerror(-rc));
        return rc;
    }

    rc = read32_ioctl(fd, reset_vector0_spa(), &readback);
    if (rc) {
        fprintf(stderr, "readback RESET_VECTOR[0] (0x%012llx) failed: %s\n",
                (unsigned long long)reset_vector0_spa(), strerror(-rc));
        return rc;
    }

    if (readback != (uint32_t)reset_vector) {
        fprintf(stderr, "RESET_VECTOR[0] readback mismatch: expected 0x%08x got 0x%08x\n",
                (uint32_t)reset_vector, readback);
        return -EIO;
    }

    printf("RESET_VECTOR[0] = 0x%08x (SPA=0x%012llx)\n",
           readback, (unsigned long long)reset_vector0_spa());
    return 0;
}

static int wipe_smc_sram(int fd)
{
    uint64_t base = local_addr_to_spa(KER_SMC_SRAM_BASE);
    uint64_t end = base + KER_SMC_SRAM_SIZE;
    uint8_t *zeros = calloc(1, KER_SMC_SRAM_SIZE);
    int rc;

    if (!zeros) {
        return -ENOMEM;
    }
    printf("Wiping SMC SRAM at SPA=0x%012llx..0x%012llx\n",
           (unsigned long long)base, (unsigned long long)(end - 1));
    rc = write_words64(fd, base, zeros, KER_SMC_SRAM_SIZE);
    if (rc) {
        fprintf(stderr, "wipe SRAM at SPA 0x%012llx failed: %s\n",
                (unsigned long long)base, strerror(-rc));
    }
    free(zeros);
    return rc;
}

static uint64_t scratch_spa(unsigned int idx)
{
    return g_spa_base + KER_SCRATCH_BASE_OFFSET + ((uint64_t)idx * KER_SCRATCH_STRIDE);
}

static int dump_post_reset_registers(int fd)
{
    int rc;
    uint32_t val;
    unsigned int i;

    usleep(1000000);
    printf("Post-reset register dump after 1s (base=0x%012llx):\n",
           (unsigned long long)g_spa_base);

    rc = read32_ioctl(fd, reset_ctrl_spa(), &val);
    if (rc) {
        fprintf(stderr, "read RESET_CTRL (0x%012llx) failed: %s\n",
                (unsigned long long)reset_ctrl_spa(), strerror(-rc));
        return rc;
    }
    printf("RESET_CTRL [0x%012llx] = 0x%08x\n",
           (unsigned long long)reset_ctrl_spa(), val);

    for (i = 0; i < KER_SCRATCH_DUMP_COUNT; i++) {
        uint64_t spa = scratch_spa(i);

        rc = read32_ioctl(fd, spa, &val);
        if (rc) {
            fprintf(stderr, "read SCRATCH_%u (0x%012llx) failed: %s\n",
                    i, (unsigned long long)spa, strerror(-rc));
            return rc;
        }
        printf("SCRATCH_%u  [0x%012llx] = 0x%08x\n",
               i, (unsigned long long)spa, val);
    }

    return 0;
}

static int clear_scratch_regs(int fd)
{
    unsigned int i;

    printf("Clearing %u scratch registers\n", KER_SCRATCH_CLEAR_COUNT);
    for (i = 0; i < KER_SCRATCH_CLEAR_COUNT; i++) {
        int rc = write32_ioctl(fd, scratch_spa(i), 0u);
        if (rc) {
            fprintf(stderr, "clear SCRATCH_%u (0x%012llx) failed: %s\n",
                    i, (unsigned long long)scratch_spa(i), strerror(-rc));
            return rc;
        }
    }
    return 0;
}

static int set_reset_state(int fd, uint32_t requested_state);

static int do_bl0_boot(int fd)
{
    int rc;

    rc = set_reset_state(fd, 0u);
    if (rc) return rc;

    rc = clear_scratch_regs(fd);
    if (rc) return rc;

    rc = wipe_smc_sram(fd);
    if (rc) return rc;

    rc = set_reset_vector(fd, KER_SMC_BL0_RESET_VECTOR);
    if (rc) return rc;

    return set_reset_state(fd, 1u);
}

static int set_reset_state(int fd, uint32_t requested_state)
{
    int rc;
    uint32_t old_val = 0;
    uint32_t new_val;
    uint32_t rb_val = 0;
    uint32_t bit_now;
    uint32_t mask = 1u << KER_RESET_CTRL_CORE0_RESET_N_BIT;

    rc = read32_ioctl(fd, reset_ctrl_spa(), &old_val);
    if (rc) {
        fprintf(stderr, "read RESET_CTRL (0x%012llx) failed: %s\n",
                (unsigned long long)reset_ctrl_spa(), strerror(-rc));
        return rc;
    }

    new_val = old_val;
    if (requested_state == 0u) {
        new_val &= ~mask;
    } else {
        new_val |= mask;
    }

    rc = write32_ioctl(fd, reset_ctrl_spa(), new_val);
    if (rc) {
        fprintf(stderr, "write RESET_CTRL (0x%012llx) failed: %s\n",
                (unsigned long long)reset_ctrl_spa(), strerror(-rc));
        return rc;
    }

    rc = read32_ioctl(fd, reset_ctrl_spa(), &rb_val);
    if (rc) {
        fprintf(stderr, "readback RESET_CTRL (0x%012llx) failed: %s\n",
                (unsigned long long)reset_ctrl_spa(), strerror(-rc));
        return rc;
    }

    bit_now = (rb_val >> KER_RESET_CTRL_CORE0_RESET_N_BIT) & 1u;
    printf("RESET_CTRL old=0x%08x new=0x%08x readback=0x%08x\n", old_val, new_val, rb_val);
    printf("core0_reset_n=%u (%s)\n", bit_now, bit_now ? "out of reset" : "held in reset");

    if (requested_state != bit_now) {
        fprintf(stderr, "ERROR: requested state=%u but readback bit=%u\n", requested_state, bit_now);
        return -EIO;
    }

    return 0;
}

static uint64_t local_addr_to_spa(uint64_t addr)
{
    if (addr >= KER_SMC_CORE_LOCAL_BASE && addr < KER_SMC_CORE_LOCAL_BASE + 0x1000000ULL) {
        return g_spa_base + (addr - KER_SMC_CORE_LOCAL_BASE);
    }
    return addr;
}

static void print_progress_bar(off_t done, off_t total)
{
    static const char fill[] = "########################################";
    const int width = (int)(sizeof(fill) - 1);
    int pct = (total > 0) ? (int)((done * 100) / total) : 100;
    int filled = (pct * width) / 100;

    printf("\r[%.*s%*s] %3d%%", filled, fill, width - filled, "", pct);
    fflush(stdout);
}

static int load_image_to_spa(int fd, const char *image_path, uint64_t load_spa)
{
    uint8_t *data = NULL;
    size_t size = 0;
    size_t padded;
    int rc;

    rc = read_image_file(image_path, &data, &size);
    if (rc) {
        return rc;
    }
    if (load_spa & 0x3ULL) {
        fprintf(stderr, "load SPA 0x%012llx is not 4-byte aligned\n", (unsigned long long)load_spa);
        free(data);
        return -EINVAL;
    }

    printf("Loading %zu bytes from %s to SPA=0x%012llx\n",
           size, image_path, (unsigned long long)load_spa);

    /* Preserve the target bytes past the end of the image in its last partial word. */
    padded = (size + 3) & ~(size_t)3;
    if (padded != size) {
        uint32_t old_word = 0;
        uint64_t spa = load_spa + padded - 4;

        rc = read32_ioctl(fd, spa, &old_word);
        if (rc) {
            fprintf(stderr, "read target word failed at SPA 0x%012llx: %s\n",
                    (unsigned long long)spa, strerror(-rc));
            free(data);
            return rc;
        }
        for (size_t i = size; i < padded; i++) {
            data[i] = (uint8_t)(old_word >> (8 * (i - (padded - 4))));
        }
    }

    rc = write_words(fd, load_spa, data, padded, 1);
    printf("\n");
    if (rc) {
        fprintf(stderr, "write image to SPA 0x%012llx failed: %s\n",
                (unsigned long long)load_spa, strerror(-rc));
    }
    free(data);
    return rc;
}

static int verify_image_at_spa(int fd, const char *image_path, uint64_t load_spa)
{
    uint8_t *expected = NULL;
    uint8_t *actual;
    size_t size = 0;
    int rc;

    rc = read_image_file(image_path, &expected, &size);
    if (rc) {
        return rc;
    }

    printf("Verifying %zu bytes at SPA=0x%012llx\n", size, (unsigned long long)load_spa);

    actual = malloc(size ? size : 1);
    if (!actual) {
        free(expected);
        return -ENOMEM;
    }
    rc = read_bytes(fd, load_spa, actual, size);
    if (rc) {
        fprintf(stderr, "read back from SPA 0x%012llx during verify failed: %s\n",
                (unsigned long long)load_spa, strerror(-rc));
        goto out;
    }

    for (size_t off = 0; off < size; off++) {
        if (actual[off] != expected[off]) {
            size_t word_off = off & ~(size_t)3;
            uint32_t exp_word = 0;
            uint32_t act_word = 0;

            for (size_t i = 0; i < 4 && word_off + i < size; i++) {
                exp_word |= (uint32_t)expected[word_off + i] << (8 * i);
                act_word |= (uint32_t)actual[word_off + i] << (8 * i);
            }
            fprintf(stderr,
                    "verify mismatch at SPA 0x%012llx (offset %zu): expected 0x%08x got 0x%08x\n",
                    (unsigned long long)(load_spa + word_off), word_off, exp_word, act_word);
            rc = -EIO;
            goto out;
        }
    }
    printf("Image verify passed\n");

out:
    free(actual);
    free(expected);
    return rc;
}

static int verify_image_in_smc(int fd, const char *image_path)
{
    return verify_image_at_spa(fd, image_path, local_addr_to_spa(KER_SMC_BL1_RESET_VECTOR));
}

static int poll_scratch_eq(int fd, uint64_t spa, uint32_t expected)
{
    uint32_t val = 0;
    int rc;
    uint32_t elapsed_us = 0;

    printf("Polling SPA=0x%012llx == 0x%08x...\n", (unsigned long long)spa, expected);
    for (;;) {
        rc = read32_ioctl(fd, spa, &val);
        if (rc) {
            fprintf(stderr, "read SPA 0x%012llx failed: %s\n",
                    (unsigned long long)spa, strerror(-rc));
            return rc;
        }
        if (val == expected) {
            printf("  -> 0x%08x\n", val);
            return 0;
        }
        usleep(1000000);
        elapsed_us += 1000000;
        //if (elapsed_us >= BUNDLE_POLL_TIMEOUT_US) {
        //    fprintf(stderr, "timeout: SPA 0x%012llx expected 0x%08x, got 0x%08x\n",
        //            (unsigned long long)spa, expected, val);
        //    return -ETIMEDOUT;
        //}
    }
}

static int poll_scratch_bit(int fd, uint64_t spa, uint32_t mask)
{
    uint32_t val = 0;
    int rc;
    uint32_t elapsed_us = 0;

    printf("Polling SPA=0x%012llx & 0x%08x...\n", (unsigned long long)spa, mask);
    for (;;) {
        rc = read32_ioctl(fd, spa, &val);
        if (rc) {
            fprintf(stderr, "read SPA 0x%012llx failed: %s\n",
                    (unsigned long long)spa, strerror(-rc));
            return rc;
        }
        if ((val & mask) == mask) {
            printf("  -> 0x%08x\n", val);
            return 0;
        }
        usleep(1000000);
        elapsed_us += 1000000;
        //if (elapsed_us >= BUNDLE_POLL_TIMEOUT_US) {
        //    fprintf(stderr, "timeout: SPA 0x%012llx mask 0x%08x, got 0x%08x\n",
        //            (unsigned long long)spa, mask, val);
        //    return -ETIMEDOUT;
        //}
    }
}

/*
 * Fake a BUN1 being present in SRAM loaded by SEP BL0
 */
static int write_bun1_to_staging(int fd)
{
    int rc;
    uint64_t staging_spa = local_addr_to_spa(KER_SMC_BL1_RESET_VECTOR);

    uint32_t hdr_words = (BUNDLE_MANIFEST_SIZE + BUNDLE_TOC_HDR_SIZE + 2u * BUNDLE_TOC_ENTRY_SIZE) / 4;
    uint32_t payload_len = BUNDLE_TOC_HDR_SIZE + 2u * BUNDLE_TOC_ENTRY_SIZE;

    printf("Staging BUN1 at SPA=0x%012llx: manifest+toc+entries=%u bytes\n",
           (unsigned long long)staging_spa,
           BUNDLE_MANIFEST_SIZE + BUNDLE_TOC_HDR_SIZE + 3u * BUNDLE_TOC_ENTRY_SIZE);

    /* Zero the header region so unset fields don't contain stale values */
    rc = zero_words(fd, staging_spa, hdr_words * 4u);
    if (rc) {
        fprintf(stderr, "zero staging header failed: %s\n", strerror(-rc));
        return rc;
    }
    uint64_t toc_spa = staging_spa + BUNDLE_MANIFEST_SIZE;

    /* Manifest: only payload_offset (int64 at +1160) is read by the BL0p5 loader */
    rc = write32_ioctl(fd, staging_spa + BUNDLE_MANIFEST_PAYLOAD_OFF_OFF, BUNDLE_MANIFEST_SIZE);
    if (rc) return rc;

    /* TOC header */
    rc = write32_ioctl(fd, toc_spa + 0, BUNDLE_TOC_ID); /* toc_identifier */
    if (rc) return rc;

    rc = write32_ioctl(fd, toc_spa + 4, BUNDLE_TOC_VERSION_MAJOR); /* major=1, minor=0 */
    if (rc) return rc;

    rc = write32_ioctl(fd, toc_spa + 8, payload_len); /* payload_length lo */
    if (rc) return rc;

    /* payload_length hi = 0 (already zeroed) */
    rc = write32_ioctl(fd, toc_spa + 16, 3u); /* image_count = 3 */
    if (rc) return rc;

    /*The rest doesn't really matter maybe...*/
    return 0;
}

/* One image described by a bundle TOC entry. */
struct bundle_img {
    uint32_t toc_index;    /* TOC slot; each slot must be used at most once */
    uint64_t type;         /* FW_BUNDLE_IMG_TYPE_* */
    const char *path;      /* host file to stage */
    uint64_t size;
    uint64_t load_addr;    /* core-local; 0 = leave the entry field zero */
    uint64_t entry_point;  /* core-local; 0 = leave the entry field zero */
    uint64_t offset;       /* payload-relative data offset, set by bundle_layout() */
};

#define BUNDLE_IMG_TYPE_BL1 (((uint64_t)BUNDLE_TOC_IMG_TYPE_BL1_HI << 32) | BUNDLE_TOC_IMG_TYPE_BL1_LO)
#define BUNDLE_IMG_TYPE_MIS (((uint64_t)BUNDLE_TOC_IMG_TYPE_MIS_HI << 32) | BUNDLE_TOC_IMG_TYPE_MIS_LO)

static uint64_t bundle_data_start(uint32_t entry_count)
{
    return BUNDLE_TOC_HDR_SIZE + (uint64_t)entry_count * BUNDLE_TOC_ENTRY_SIZE;
}

static uint32_t bundle_entry_count(const struct bundle_img *imgs, size_t n)
{
    uint32_t count = 0;

    for (size_t i = 0; i < n; i++) {
        if (imgs[i].toc_index + 1u > count) {
            count = imgs[i].toc_index + 1u;
        }
    }
    return count;
}

/*
 * Assign payload-relative data offsets in array order, starting right after the
 * TOC table. Each image starts on an `align` boundary. Returns the payload length.
 */
static uint64_t bundle_layout(struct bundle_img *imgs, size_t n, uint32_t entry_count,
                              uint64_t align)
{
    uint64_t off = bundle_data_start(entry_count);

    for (size_t i = 0; i < n; i++) {
        off = (off + align - 1) & ~(align - 1);
        imgs[i].offset = off;
        off += imgs[i].size;
    }
    return off;
}

/* Write a 64-bit value as two little-endian 32-bit words at spa. */
static int write_u64_pair(int fd, uint64_t spa, uint64_t value)
{
    int rc = write32_ioctl(fd, spa, (uint32_t)value);

    if (rc) return rc;
    return write32_ioctl(fd, spa + 4, (uint32_t)(value >> 32));
}

/*
 * Write manifest + TOC + entries + image data for one bundle.
 * The manifest sits at the BL1 reset vector (local); the TOC sits payload_rel bytes
 * past it. The bundle must end below max_end_local (local address).
 */
static int write_bundle(int fd, uint64_t payload_rel, struct bundle_img *imgs, size_t n,
                        uint64_t align, uint64_t max_end_local)
{
    uint64_t staging_spa = local_addr_to_spa(KER_SMC_BL1_RESET_VECTOR);
    uint32_t entry_count = bundle_entry_count(imgs, n);
    uint64_t payload_len;
    uint64_t payload_spa;
    int rc;

    payload_len = bundle_layout(imgs, n, entry_count, align);
    if (KER_SMC_BL1_RESET_VECTOR + payload_rel + payload_len > max_end_local) {
        fprintf(stderr, "bundle (%llu bytes at 0x%08llx) overruns limit 0x%08llx\n",
                (unsigned long long)payload_len,
                (unsigned long long)KER_SMC_BL1_RESET_VECTOR,
                (unsigned long long)max_end_local);
        return -ENOSPC;
    }

    payload_spa = staging_spa + payload_rel;
    printf("Staging bundle at SPA=0x%012llx: %u TOC entries, payload=%llu bytes\n",
           (unsigned long long)payload_spa, entry_count, (unsigned long long)payload_len);

    /* Clear manifest + TOC table so unused slots read as empty */
    rc = zero_words(fd, staging_spa, payload_rel + bundle_data_start(entry_count));
    if (rc) {
        fprintf(stderr, "zero staging header failed: %s\n", strerror(-rc));
        return rc;
    }

    rc = write_u64_pair(fd, staging_spa + BUNDLE_MANIFEST_PAYLOAD_OFF_OFF, payload_rel);
    if (rc) return rc;

    rc = write32_ioctl(fd, payload_spa + 0, BUNDLE_TOC_ID);
    if (rc) return rc;
    rc = write32_ioctl(fd, payload_spa + 4, BUNDLE_TOC_VERSION_MAJOR);
    if (rc) return rc;
    rc = write_u64_pair(fd, payload_spa + 8, payload_len);
    if (rc) return rc;
    rc = write_u64_pair(fd, payload_spa + 16, entry_count);
    if (rc) return rc;

    for (size_t i = 0; i < n; i++) {
        uint64_t e = payload_spa + BUNDLE_TOC_HDR_SIZE +
                     (uint64_t)imgs[i].toc_index * BUNDLE_TOC_ENTRY_SIZE;

        rc = write_u64_pair(fd, e + 0, imgs[i].type);
        if (rc) return rc;
        rc = write_u64_pair(fd, e + 8, imgs[i].offset);
        if (rc) return rc;
        rc = write_u64_pair(fd, e + 16, imgs[i].size);
        if (rc) return rc;
        if (imgs[i].load_addr) {
            rc = write_u64_pair(fd, e + 32, imgs[i].load_addr);
            if (rc) return rc;
        }
        if (imgs[i].entry_point) {
            rc = write_u64_pair(fd, e + 40, imgs[i].entry_point);
            if (rc) return rc;
        }
    }

    for (size_t i = 0; i < n; i++) {
        rc = load_image_to_spa(fd, imgs[i].path, payload_spa + imgs[i].offset);
        if (rc) return rc;
    }
    return 0;
}

/*
 * BUN2 (read by BL0P5): manifest at the BL1 reset vector, TOC right after it.
 * Must stay below BL0P5's load address, which BL0P5 is still executing from.
 */
static int write_bun2_to_staging(int fd, struct bundle_img *imgs, size_t n)
{
    return write_bundle(fd, BUNDLE_MANIFEST_SIZE, imgs, n, 4, KER_SMC_BL0P5_LOAD_ADDR);
}

/*
 * BUN3 (read by K-BL1). The K-MIS image is linked at KER_SMC_KMIS_LOAD_ADDR and is
 * jumped to in place, so it must be the first image (TOC index 0) and the TOC has to
 * end exactly where it starts. Must stay below BL1, which is still executing.
 */
static int write_mis_bundle_to_staging(int fd, struct bundle_img *imgs, size_t n)
{
    uint32_t entry_count = bundle_entry_count(imgs, n);
    uint64_t payload_rel;

    if (n == 0 || imgs[0].toc_index != 0) {
        fprintf(stderr, "BUN3: K-MIS must be TOC index 0\n");
        return -EINVAL;
    }

    payload_rel = KER_SMC_KMIS_LOAD_ADDR - KER_SMC_BL1_RESET_VECTOR - bundle_data_start(entry_count);
    printf("K-MIS placed at local=0x%08llx\n", (unsigned long long)KER_SMC_KMIS_LOAD_ADDR);
    return write_bundle(fd, payload_rel, imgs, n, 8, KER_SMC_BL1_EXEC_ADDR);
}

static int stat_image(const char *path, const char *what, uint64_t *size)
{
    struct stat st;

    if (stat(path, &st) < 0 || !S_ISREG(st.st_mode)) {
        fprintf(stderr, "cannot stat %s image %s: %s\n", what, path, strerror(errno));
        return -errno;
    }
    *size = (uint64_t)st.st_size;
    return 0;
}

/*
 * Boot blop5 with the sysbuild images. mbl1/mmis are the Mimir images. qbl1/qmis are
 * the Quasar images and are staged alongside them when non-NULL (qsr1a1 only).
 */
static int do_blop5_boot(int fd, const char *blop5_path, const char *bl1_path,
                         const char *mbl1_path, const char *d2d_path, const char *kmis_path,
                         const char *mmis_path, const char *qbl1_path, const char *qmis_path,
                         int skip_bl0p5_load)
{
    int rc;
    uint32_t val;
    uint64_t bl1_size, mbl1_size, d2d_size, kmis_size, mmis_size, qbl1_size = 0, qmis_size = 0;
    struct bundle_img bun2[4];
    struct bundle_img bun3[3];
    size_t bun2_n = 0;
    size_t bun3_n = 0;

    if ((rc = stat_image(bl1_path, "BL1", &bl1_size)) ||
        (rc = stat_image(mbl1_path, "mbl1", &mbl1_size)) ||
        (rc = stat_image(d2d_path, "d2d", &d2d_size)) ||
        (rc = stat_image(kmis_path, "kmis", &kmis_size)) ||
        (rc = stat_image(mmis_path, "mmis", &mmis_size))) {
        return rc;
    }
    if (qbl1_path && (rc = stat_image(qbl1_path, "qbl1", &qbl1_size))) {
        return rc;
    }
    if (qmis_path && (rc = stat_image(qmis_path, "qmis", &qmis_size))) {
        return rc;
    }

    /* BUN2 slots: 0 = K BL1, 1 = Mimir BL1, 2 = D2D, 4 = Quasar BL1 (3 is GDDR params) */
    bun2[bun2_n++] = (struct bundle_img){
        .toc_index = 0, .type = BUNDLE_IMG_TYPE_BL1, .path = bl1_path, .size = bl1_size,
        .load_addr = KER_SMC_BL1_EXEC_ADDR, .entry_point = KER_SMC_BL1_EXEC_ADDR,
    };
    bun2[bun2_n++] = (struct bundle_img){
        .toc_index = 1, .type = BUNDLE_IMG_TYPE_BL1, .path = mbl1_path, .size = mbl1_size,
        .load_addr = KER_SMC_BL1_EXEC_ADDR, .entry_point = KER_SMC_BL1_EXEC_ADDR,
    };
    bun2[bun2_n++] = (struct bundle_img){
        .toc_index = 2, .type = BUNDLE_IMG_TYPE_BL1, .path = d2d_path, .size = d2d_size,
    };
    if (qbl1_path) {
        bun2[bun2_n++] = (struct bundle_img){
            .toc_index = BUN2_QUASAR_BL1_TOC_INDEX, .type = BUNDLE_IMG_TYPE_BL1,
            .path = qbl1_path, .size = qbl1_size,
            .load_addr = KER_SMC_BL1_EXEC_ADDR, .entry_point = KER_SMC_BL1_EXEC_ADDR,
        };
    }

    /* BUN3 slots: 0 = K MIS (must be first), 1 = Mimir MIS, 2 = Quasar MIS */
    bun3[bun3_n++] = (struct bundle_img){
        .toc_index = 0, .type = BUNDLE_IMG_TYPE_MIS, .path = kmis_path, .size = kmis_size,
        .load_addr = KER_SMC_KMIS_LOAD_ADDR, .entry_point = KER_SMC_KMIS_LOAD_ADDR,
    };
    bun3[bun3_n++] = (struct bundle_img){
        .toc_index = 1, .type = BUNDLE_IMG_TYPE_MIS, .path = mmis_path, .size = mmis_size,
        .load_addr = KER_SMC_KMIS_LOAD_ADDR, .entry_point = KER_SMC_KMIS_LOAD_ADDR,
    };
    if (qmis_path) {
        bun3[bun3_n++] = (struct bundle_img){
            .toc_index = BUN3_QUASAR_MIS_TOC_INDEX, .type = BUNDLE_IMG_TYPE_MIS,
            .path = qmis_path, .size = qmis_size,
            .load_addr = KER_SMC_KMIS_LOAD_ADDR, .entry_point = KER_SMC_KMIS_LOAD_ADDR,
        };
    }

    if (skip_bl0p5_load) {
        printf("Skipping BL0P5 load; assuming it is already running\n");
        goto wait_for_bundle_ready;
    }

    /* Hold the core in reset */
    rc = set_reset_state(fd, 0u);
    if (rc) return rc;

    /* Aim the reset vector at BL0. This lets it setup up the SEP-safe region */
    rc = set_reset_vector(fd, KER_SMC_BL0_RESET_VECTOR);
    if (rc) return rc;

    /* Release reset; BL0 boots and initialises sep safe region*/
    rc = set_reset_state(fd, 1u);
    if (rc) return rc;

    /* SEP-BL0 (?) Wait for SEP-SRAM init */
    rc = poll_scratch_bit(fd, local_addr_to_spa(KER_SMC_SCRATCH_SMC_STATUS_TO_SEP),
                          SMC_SEP_STATUS_SRAM_INIT);

    rc = set_reset_state(fd, 0u);
    if (rc) return rc;

    /* Load BUN1 to its stage */
    rc = write_bun1_to_staging(fd);
    if (rc) return rc;

    /* Load BL0P5 to its execute location */
    rc = load_image_to_spa(fd, blop5_path, local_addr_to_spa(KER_SMC_BL0P5_LOAD_ADDR));
    if (rc) return rc;

    rc = verify_image_at_spa(fd, blop5_path, local_addr_to_spa(KER_SMC_BL0P5_LOAD_ADDR));
    if (rc) return rc;

    /* Aim the reset vector at BL0P5 */
    rc = set_reset_vector(fd, KER_SMC_BL0P5_LOAD_ADDR);
    if (rc) return rc;

    /* Release reset; BL0P5 boots and initialises the bun2 loader */
    rc = set_reset_state(fd, 1u);
    if (rc) return rc;

wait_for_bundle_ready:
    /* B) Wait for BL0P5 to signal it is ready to receive a bundle */
    printf("Waiting for BL0P5 bundle-ready signal...\n");
    rc = poll_scratch_eq(fd, local_addr_to_spa(KER_HOST_BOOT_STATE_LOCAL),
                         HOST_BOOT_STATE_WAIT_FOR_BUNDLE);
    if (rc) return rc;

    /* C+D) Write bundle manifest, TOC entries, and images into the staging area */
    rc = write_bun2_to_staging(fd, bun2, bun2_n);
    if (rc) return rc;

    /* Signal that the bundle is staged */
    rc = write32_ioctl(fd, local_addr_to_spa(KER_HOST_BOOT_STATE_LOCAL),
                       HOST_BOOT_STATE_BUNDLE_STAGED);
    if (rc) return rc;

    /* E) Wait for the bun2 loader to request validation */
    printf("Waiting for bun2 loader validation request...\n");
    rc = poll_scratch_bit(fd, local_addr_to_spa(KER_BUNDLE_VALIDATION_LOCAL),
                          BUNDLE_READY_FOR_VALIDATION_BIT);
    if (rc) return rc;

    /* F) Confirm the bundle is valid */
    rc = read32_ioctl(fd, local_addr_to_spa(KER_BUNDLE_VALIDATION_LOCAL), &val);
    if (rc) return rc;
    rc = write32_ioctl(fd, local_addr_to_spa(KER_BUNDLE_VALIDATION_LOCAL),
                       val | BUNDLE_VALIDATED_BIT);
    if (rc) return rc;

    /* K-MIS signals that it is ready for the next host bundle handshake. */
    printf("Waiting for K-MIS bundle-ready signal...\n");
    rc = poll_scratch_eq(fd, local_addr_to_spa(KER_HOST_BOOT_STATE_LOCAL),
                         HOST_BOOT_STATE_WAIT_FOR_BUNDLE);
    if (rc) return rc;

    /* BUN2 is consumed; reuse its staging area for the BUN3 MIS bundle. */
    rc = write_mis_bundle_to_staging(fd, bun3, bun3_n);
    if (rc) return rc;

    rc = write32_ioctl(fd, local_addr_to_spa(KER_HOST_BOOT_STATE_LOCAL),
                       HOST_BOOT_STATE_BUNDLE_STAGED);
    if (rc) return rc;

    printf("Waiting for BUN3 validation request...\n");
    rc = poll_scratch_bit(fd, local_addr_to_spa(KER_BUNDLE_VALIDATION_LOCAL),
                          BUNDLE_READY_FOR_VALIDATION_BIT);
    if (rc) return rc;

    rc = read32_ioctl(fd, local_addr_to_spa(KER_BUNDLE_VALIDATION_LOCAL), &val);
    if (rc) return rc;
    rc = write32_ioctl(fd, local_addr_to_spa(KER_BUNDLE_VALIDATION_LOCAL),
                       val | BUNDLE_VALIDATED_BIT);
    if (rc) return rc;

    printf("Handshake complete; MIS at should now be executing\n");

    return dump_post_reset_registers(fd);
}

/*
 * BL0 pre-boot for the sysbuild flow: wipe each chiplet's SRAM and park it
 * before K. The set of chiplets depends on the topology. K is always last.
 */
static int do_sysbuild_bl0_boot(int fd, enum topology topo)
{
    static const uint64_t mmk_bases[] = {
        KER_SPA_BASE_M0, KER_SPA_BASE_M1, KER_SPA_BASE_K,
    };
    static const uint64_t qsr1a1_bases[] = {
        KER_SPA_BASE_M1, KER_SPA_BASE_M2, KER_SPA_BASE_Q0, KER_SPA_BASE_K,
    };
    const uint64_t *bases = mmk_bases;
    size_t count = sizeof(mmk_bases) / sizeof(mmk_bases[0]);
    int rc;

    if (topo == TOPO_QSR1A1) {
        bases = qsr1a1_bases;
        count = sizeof(qsr1a1_bases) / sizeof(qsr1a1_bases[0]);
    }

    for (size_t i = 0; i < count; i++) {
        g_spa_base = bases[i];
        rc = do_bl0_boot(fd);
        if (rc) {
            return rc;
        }
    }
    return 0;
}

static char *make_image_path(const char *build_dir)
{
    size_t len = strlen(build_dir) + sizeof("/zephyr/zephyr.bin");
    char *path = malloc(len);

    if (!path) {
        fprintf(stderr, "out of memory\n");
        return NULL;
    }
    snprintf(path, len, "%s/zephyr/zephyr.bin", build_dir);
    return path;
}

static char *make_image_path_from_sysbuild(const char *sysbuild_dir, const char *image_name)
{
    const char *suffix = "/zephyr/zephyr.bin";
    size_t len = strlen(sysbuild_dir) + 1 + strlen(image_name) + strlen(suffix) + 1;
    char *path = malloc(len);

    if (!path) {
        fprintf(stderr, "out of memory\n");
        return NULL;
    }
    snprintf(path, len, "%s/%s%s", sysbuild_dir, image_name, suffix);
    return path;
}

static char *make_d2d_image_path_from_sysbuild(const char *sysbuild_dir)
{
    const char *suffix = "/grendel-sival-sdk/grendel_sival_sdk/firmware/libs/chip_init/d2d_fw/d2d_fw.bin";
    size_t len = strlen(sysbuild_dir) + 1 + strlen("bl1_keraunos") + strlen(suffix) + 1;
    char *path = malloc(len);

    if (!path) {
        fprintf(stderr, "out of memory\n");
        return NULL;
    }
    snprintf(path, len, "%s/%s%s", sysbuild_dir, "bl1_keraunos", suffix);

    /* The Sival SDK is now extracted into a shared cache (see cmake/sival_sdk.cmake)
     * rather than the build dir, so fall back to the newest cached copy. */
    if (access(path, R_OK) != 0) {
        const char *rel = "/grendel_sival_sdk/firmware/libs/chip_init/d2d_fw/d2d_fw.bin";
        const char *env = getenv("TT_SIVAL_SDK_CACHE");
        const char *home = getenv("HOME");
        char cache[PATH_MAX];
        char best[PATH_MAX] = "";
        time_t best_mtime = 0;
        DIR *dir;
        struct dirent *de;

        if (env) {
            snprintf(cache, sizeof(cache), "%s", env);
        } else if (home) {
            /* Under sudo, HOME is root's; use the invoking user's cache. */
            const char *sudo_user = getenv("SUDO_USER");
            struct passwd *pw = sudo_user ? getpwnam(sudo_user) : NULL;

            snprintf(cache, sizeof(cache), "%s/.cache/tt-sival-sdk",
                     pw ? pw->pw_dir : home);
        } else {
            return path;
        }

        dir = opendir(cache);
        if (!dir) {
            return path;
        }
        while ((de = readdir(dir)) != NULL) {
            char cand[PATH_MAX];
            struct stat st;

            if (de->d_name[0] == '.') {
                continue;
            }
            if (snprintf(cand, sizeof(cand), "%s/%s%s", cache, de->d_name, rel) >= (int)sizeof(cand)) {
                continue;
            }
            if (stat(cand, &st) == 0 && S_ISREG(st.st_mode) && st.st_mtime >= best_mtime) {
                best_mtime = st.st_mtime;
                snprintf(best, sizeof(best), "%s", cand);
            }
        }
        closedir(dir);

        if (best[0]) {
            char *resolved = strdup(best);
            if (resolved) {
                free(path);
                path = resolved;
            }
        }
    }
    return path;
}

int main(int argc, char **argv)
{
    char dev_path[64];
    long reset_state = -1;
    long dev_id = 0;
    char *endptr = NULL;
    char *image_path = NULL;
    char *blop5_path = NULL;
    char *mbl1_path = NULL;
    char *d2d_path = NULL;
    char *kmis_path = NULL;
    char *mmis_path = NULL;
    char *qbl1_path = NULL;
    char *qmis_path = NULL;
    char *sysbuild_path = NULL;
    char *topology_arg = NULL;
    enum topology topology = TOPO_NONE;
    int image_mode = 0;
    int blop5_mode = 0;
    int verify = 0;
    int scratch = 0;
    int bl0_mode = 0;
    int direct_sysbuild_mode = 0;
    int skip_bl0p5_load = 0;
    char *reset_vector_arg = NULL;
    uint64_t custom_reset_vector = 0;
    int have_custom_reset_vector = 0;
    int fd;
    int rc;

    /* Pre-scan: extract --topology <mmk|qsr1a1> or --topology=<mmk|qsr1a1> from anywhere */
    for (int topo_i = 1; topo_i < argc; topo_i++) {
        if (!strncmp(argv[topo_i], "--topology=", sizeof("--topology=") - 1)) {
            topology_arg = argv[topo_i] + sizeof("--topology=") - 1;
            for (int topo_j = topo_i; topo_j < argc - 1; topo_j++) {
                argv[topo_j] = argv[topo_j + 1];
            }
            argc -= 1;
            break;
        }
        if (!strcmp(argv[topo_i], "--topology")) {
            if (topo_i + 1 >= argc) {
                fprintf(stderr, "--topology requires a value (mmk or qsr1a1)\n");
                usage(argv[0]);
                return 2;
            }
            topology_arg = argv[topo_i + 1];
            for (int topo_j = topo_i; topo_j < argc - 2; topo_j++) {
                argv[topo_j] = argv[topo_j + 2];
            }
            argc -= 2;
            break;
        }
    }

    if (argc < 3) {
        usage(argv[0]);
        return 2;
    }

    direct_sysbuild_mode = !strcmp(argv[1], "--sysbuild");
    if (direct_sysbuild_mode) {
        blop5_mode = 1;
    } else {
        rc = parse_mode_arg(argv[1]);
    }
    if (!direct_sysbuild_mode && rc) {
        fprintf(stderr, "Invalid mode '%s'. Expected 'k', 'm0', 'm1', 'm2', 'm3' or 'q0'.\n", argv[1]);
        usage(argv[0]);
        return 2;
    }

    /* Pre-scan: extract --blop5 before mode-specific argument parsing */
    for (int blop5_i = 2; blop5_i < argc; blop5_i++) {
        if (!strcmp(argv[blop5_i], "--blop5")) {
            blop5_mode = 1;
            for (int blop5_j = blop5_i; blop5_j < argc - 1; blop5_j++) {
                argv[blop5_j] = argv[blop5_j + 1];
            }
            argc -= 1;
            break;
        }
    }

    for (int skip_i = 1; skip_i < argc; skip_i++) {
        if (!strcmp(argv[skip_i], "--skip-bl0p5-load")) {
            skip_bl0p5_load = 1;
            for (int skip_j = skip_i; skip_j < argc - 1; skip_j++) {
                argv[skip_j] = argv[skip_j + 1];
            }
            argc -= 1;
            break;
        }
    }

    /* Pre-scan: extract --sysbuild <path> before mode-specific argument parsing */
    for (int sysbuild_i = direct_sysbuild_mode ? 1 : 2; sysbuild_i < argc; sysbuild_i++) {
        if (!strcmp(argv[sysbuild_i], "--sysbuild")) {
            if (sysbuild_i + 1 >= argc) {
                fprintf(stderr, "--sysbuild requires a path argument\n");
                usage(argv[0]);
                return 2;
            }
            sysbuild_path = argv[sysbuild_i + 1];
            for (int sysbuild_j = sysbuild_i; sysbuild_j < argc - 2; sysbuild_j++) {
                argv[sysbuild_j] = argv[sysbuild_j + 2];
            }
            argc -= 2;
            break;
        }
    }

    /* Pre-scan: extract -r <hex_addr> before mode-specific argument parsing (only valid with -i/--kbl1) */
    for (int rv_i = 2; rv_i < argc; rv_i++) {
        if (!strcmp(argv[rv_i], "-r")) {
            if (rv_i + 1 >= argc) {
                fprintf(stderr, "-r requires a hex address argument\n");
                usage(argv[0]);
                return 2;
            }
            reset_vector_arg = argv[rv_i + 1];
            for (int rv_j = rv_i; rv_j < argc - 2; rv_j++) {
                argv[rv_j] = argv[rv_j + 2];
            }
            argc -= 2;
            break;
        }
    }

    if ((!direct_sysbuild_mode && argc < 2) || argc > 4) {
        usage(argv[0]);
        return 2;
    }

    if (blop5_mode) {
        image_mode = 1;
        int first_device_arg = direct_sysbuild_mode ? 1 : 2;
        int expected_argc = direct_sysbuild_mode ? 1 : 2;
        if (argc != expected_argc && argc != expected_argc + 1) {
            usage(argv[0]);
            return 2;
        }
        if (argc == expected_argc + 1) {
            endptr = NULL;
            dev_id = strtol(argv[first_device_arg], &endptr, 0);
            if (endptr == argv[first_device_arg] || *endptr != '\0' || dev_id < 0 || dev_id > 255) {
                fprintf(stderr, "Invalid device_id: %s\n", argv[first_device_arg]);
                return 2;
            }
        }
    } else if (!strcmp(argv[2], "--bl0")) {
        bl0_mode = 1;
        if (argc != 3 && argc != 4) {
            usage(argv[0]);
            return 2;
        }
        if (argc == 4) {
            endptr = NULL;
            dev_id = strtol(argv[3], &endptr, 0);
            if (endptr == argv[3] || *endptr != '\0' || dev_id < 0 || dev_id > 255) {
                fprintf(stderr, "Invalid device_id: %s\n", argv[3]);
                return 2;
            }
        }
    } else if (!strcmp(argv[2], "--kbl1") ||
               (!blop5_mode && !strcmp(argv[2], "-i"))) {
        image_mode = 1;
        if (argc != 4 && argc != 5) {
            usage(argv[0]);
            return 2;
        }
        image_path = argv[3];
        if (argc == 5) {
            endptr = NULL;
            dev_id = strtol(argv[4], &endptr, 0);
            if (endptr == argv[4] || *endptr != '\0' || dev_id < 0 || dev_id > 255) {
                fprintf(stderr, "Invalid device_id: %s\n", argv[4]);
                return 2;
            }
        }
    } else if (!strcmp(argv[2], "-i") && blop5_mode) {
        fprintf(stderr, "-i is not valid with --blop5; use --kbl1 <kbl1_build_dir>\n");
        usage(argv[0]);
        return 2;
    } else if (!strcmp(argv[2], "-v")) {
        verify = 1;
        if (argc != 4 && argc != 5) {
            usage(argv[0]);
            return 2;
        }
        image_path = argv[3];
        if (argc == 5) {
            endptr = NULL;
            dev_id = strtol(argv[4], &endptr, 0);
            if (endptr == argv[4] || *endptr != '\0' || dev_id < 0 || dev_id > 255)
            {
                fprintf(stderr, "Invalid device_id: %s\n", argv[4]);
                return 2;
            }
        }
    } else if (!strcmp(argv[2], "-s")) {
        scratch = 1;
    } else {
        if (argc != 3 && argc != 4) {
            usage(argv[0]);
            return 2;
        }
        reset_state = strtol(argv[2], &endptr, 0);
        if (endptr == argv[2] || *endptr != '\0' || (reset_state != 0 && reset_state != 1)) {
            fprintf(stderr, "Invalid reset state: %s\n", argv[2]);
            usage(argv[0]);
            return 2;
        }
        if (argc == 4) {
            endptr = NULL;
            dev_id = strtol(argv[3], &endptr, 0);
            if (endptr == argv[3] || *endptr != '\0' || dev_id < 0 || dev_id > 255) {
                fprintf(stderr, "Invalid device_id: %s\n", argv[3]);
                return 2;
            }
        }
    }

    if (skip_bl0p5_load && !blop5_mode) {
        fprintf(stderr, "--skip-bl0p5-load is only valid with --sysbuild\n");
        usage(argv[0]);
        return 2;
    }

    if (blop5_mode && !sysbuild_path) {
        fprintf(stderr, "--blop5 requires --sysbuild <sysbuild_dir>\n");
        usage(argv[0]);
        return 2;
    }

    if (sysbuild_path) {
        if (!topology_arg) {
            fprintf(stderr, "--sysbuild requires --topology <mmk|qsr1a1>\n");
            usage(argv[0]);
            return 2;
        }
        if (parse_topology_arg(topology_arg, &topology)) {
            fprintf(stderr, "Invalid topology '%s'. Expected 'mmk' or 'qsr1a1'.\n", topology_arg);
            usage(argv[0]);
            return 2;
        }
    } else if (topology_arg) {
        fprintf(stderr, "--topology is only valid with --sysbuild\n");
        usage(argv[0]);
        return 2;
    }

    if (reset_vector_arg) {
        if (!image_mode || blop5_mode) {
            fprintf(stderr, "-r <hex_addr> is only valid with -i/--kbl1\n");
            usage(argv[0]);
            return 2;
        }
        endptr = NULL;
        custom_reset_vector = strtoull(reset_vector_arg, &endptr, 16);
        if (endptr == reset_vector_arg || *endptr != '\0') {
            fprintf(stderr, "Invalid reset vector: %s\n", reset_vector_arg);
            return 2;
        }
        have_custom_reset_vector = 1;
    }

    if (image_path) {
        char *p = make_image_path(image_path);
        if (!p) return 1;
        image_path = p;
    }
    if (blop5_mode) {
        /* Mimir images are staged for both topologies; Quasar images are added for qsr1a1 */
        blop5_path = make_image_path_from_sysbuild(sysbuild_path, "bl0p5_keraunos");
        image_path = make_image_path_from_sysbuild(sysbuild_path, "bl1_keraunos");
        mbl1_path = make_image_path_from_sysbuild(sysbuild_path, "bl1_mimir");
        d2d_path = make_d2d_image_path_from_sysbuild(sysbuild_path);
        kmis_path = make_image_path_from_sysbuild(sysbuild_path, "mis");
        mmis_path = make_image_path_from_sysbuild(sysbuild_path, "mis_mimir");
        if (topology == TOPO_QSR1A1) {
            qbl1_path = make_image_path_from_sysbuild(sysbuild_path, "bl1_quasar");
            qmis_path = make_image_path_from_sysbuild(sysbuild_path, "mis_quasar");
        }
        if (!blop5_path || !image_path || !mbl1_path || !d2d_path || !kmis_path || !mmis_path ||
            (topology == TOPO_QSR1A1 && (!qbl1_path || !qmis_path))) {
            return 1;
        }
    }

    /* Check every sysbuild image before touching hardware: the BL0 pre-boot resets chiplets */
    if (blop5_mode) {
        uint64_t sz;
        const struct { const char *path; const char *what; } imgs[] = {
            { blop5_path, "bl0p5" }, { image_path, "bl1" }, { mbl1_path, "mbl1" },
            { d2d_path, "d2d" }, { kmis_path, "kmis" }, { mmis_path, "mmis" },
            { qbl1_path, "qbl1" }, { qmis_path, "qmis" },
        };
        for (size_t i = 0; i < sizeof(imgs) / sizeof(imgs[0]); i++) {
            if (imgs[i].path && (rc = stat_image(imgs[i].path, imgs[i].what, &sz)) != 0) {
                return 1;
            }
        }
    }

    snprintf(dev_path, sizeof(dev_path), "/dev/tenstorrent/%ld", dev_id);
    fd = open_tt_dev(dev_path, KERAUNOS_PCI_DEVICE_ID);
    if (fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", dev_path, strerror(-fd));
        return 1;
    }

    if (direct_sysbuild_mode) {
        if (!skip_bl0p5_load) {
            rc = do_sysbuild_bl0_boot(fd, topology);
            if (rc) {
                close(fd);
                return 1;
            }
        }

        g_spa_base = KER_SPA_BASE_K;
        rc = do_blop5_boot(fd, blop5_path, image_path, mbl1_path, d2d_path,
                   kmis_path, mmis_path, qbl1_path, qmis_path, skip_bl0p5_load);
        if (rc) {
            close(fd);
            return 1;
        }
    } else if (image_mode) {
        if (blop5_mode) {
            rc = do_blop5_boot(fd, blop5_path, image_path, mbl1_path, d2d_path,
                               kmis_path, mmis_path, qbl1_path, qmis_path, skip_bl0p5_load);
            if (rc) {
                close(fd);
                return 1;
            }
        } else {
            uint64_t effective_vector = have_custom_reset_vector ? custom_reset_vector
                                                                  : KER_SMC_BL1_RESET_VECTOR;

            rc = set_reset_state(fd, 0u);
            if (rc) {
                close(fd);
                return 1;
            }

            rc = load_image_to_spa(fd, image_path, local_addr_to_spa(effective_vector));
            if (rc) {
                close(fd);
                return 1;
            }

            rc = verify_image_at_spa(fd, image_path, local_addr_to_spa(effective_vector));
            if (rc) {
                close(fd);
                return 1;
            }

            rc = set_reset_vector(fd, effective_vector);
            if (rc) {
                close(fd);
                return 1;
            }

            rc = set_reset_state(fd, 1u);
            if (rc) {
                close(fd);
                return 1;
            }

            rc = dump_post_reset_registers(fd);
            if (rc) {
                close(fd);
                return 1;
            }
        }
    } else if (bl0_mode) {
        rc = do_bl0_boot(fd);
        if (rc) {
            close(fd);
            return 1;
        }
    } else if (verify) {
        rc = verify_image_in_smc(fd, image_path);
        if (rc)
        {
            close(fd);
            return 1;
        }
    } else if (scratch) {
        rc = dump_post_reset_registers(fd);
        if (rc)
        {
            close(fd);
            return 1;
        }
    } else {
        rc = set_reset_state(fd, (uint32_t)reset_state);
        if (rc) {
            close(fd);
            return 1;
        }

        if (reset_state == 1) {
            rc = dump_post_reset_registers(fd);
            if (rc) {
                close(fd);
                return 1;
            }
        }
    }

    close(fd);
    return 0;
}
