// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only
//
// Keraunos reset helper via TENSTORRENT_IOCTL_KER_READ32/_KER_WRITE32.
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
//

#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <linux/types.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

#define TENSTORRENT_IOCTL_MAGIC 0xFA
#define TENSTORRENT_IOCTL_GET_DEVICE_INFO _IO(TENSTORRENT_IOCTL_MAGIC, 0)
#define TENSTORRENT_IOCTL_KER_READ32      _IO(TENSTORRENT_IOCTL_MAGIC, 16)
#define TENSTORRENT_IOCTL_KER_WRITE32     _IO(TENSTORRENT_IOCTL_MAGIC, 17)

#define TENSTORRENT_PCI_VENDOR_ID 0x1e52
#define KERAUNOS_PCI_DEVICE_ID    0xfeed

#define KER_SMC_CORE_LOCAL_BASE 0xC0000000ULL
#define KER_SPA_BASE_K          0x1202000000ULL
#define KER_SPA_BASE_M1         0x1308000000ULL
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
#define MIMIR_SMC_MIS_LOAD_ADDR         (KER_SPA_BASE_M1 + \
                                         (KER_SMC_KMIS_LOAD_ADDR - KER_SMC_CORE_LOCAL_BASE))
/* Scratch registers used for the BL0P5 <-> host handshake (local addresses) */
#define KER_HOST_BOOT_STATE_LOCAL        0xC0010160ULL  /* SCRATCH[12] */
#define KER_BUNDLE_VALIDATION_LOCAL      0xC0010150ULL  /* SCRATCH[10] */
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
/* Offset from payload start (= toc base) to image[0] in a 2-entry TOC */
#define BUNDLE_IMG_PAYLOAD_OFFSET        (BUNDLE_TOC_HDR_SIZE + BUNDLE_TOC_ENTRY_SIZE)
#define BUNDLE_IMG0_PAYLOAD_OFFSET       (BUNDLE_TOC_HDR_SIZE + 2u * BUNDLE_TOC_ENTRY_SIZE)
#define BUN3_PAYLOAD_OFFSET              (KER_SMC_KMIS_LOAD_ADDR - KER_SMC_BL1_RESET_VECTOR - BUNDLE_IMG0_PAYLOAD_OFFSET)
#define BUNDLE_POLL_TIMEOUT_US           10000000u  /* 10 s */

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

struct tenstorrent_ker_read32 {
    __u32 argsz;
    __u32 flags;
    __u64 addr;
    __u32 value;
    __u32 reserved;
};

struct tenstorrent_ker_write32 {
    __u32 argsz;
    __u32 flags;
    __u64 addr;
    __u32 value;
    __u32 reserved;
};

static uint64_t g_spa_base = KER_SPA_BASE_K;

static int read32_ioctl(int fd, uint64_t spa, uint32_t *value)
{
    struct tenstorrent_ker_read32 io = {0};

    io.argsz = sizeof(io);
    io.addr = spa;
    if (ioctl(fd, TENSTORRENT_IOCTL_KER_READ32, &io) < 0) {
        return -errno;
    }

    *value = io.value;
    return 0;
}

static int write32_ioctl(int fd, uint64_t spa, uint32_t value)
{
    struct tenstorrent_ker_write32 io = {0};

    io.argsz = sizeof(io);
    io.addr = spa;
    io.value = value;
    if (ioctl(fd, TENSTORRENT_IOCTL_KER_WRITE32, &io) < 0) {
        return -errno;
    }

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

static void usage(const char *prog)
{
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "  %s --sysbuild <sysbuild_dir> [device_id]\n", prog);
    fprintf(stderr, "  %s <k|m> <0|1> [device_id]\n", prog);
    fprintf(stderr, "  %s <k|m> --kbl1 <build_dir> [device_id]\n", prog);
    fprintf(stderr, "  %s <k|m> --blop5 --sysbuild <sysbuild_dir> [device_id]\n", prog);
    fprintf(stderr, "  %s <k|m> --bl0 [device_id]\n", prog);
    fprintf(stderr, "  k = SPA base 0x12020..., m = SPA base 0x13000...\n");
    fprintf(stderr, "  0 = hold SMC RISC-V in reset\n");
    fprintf(stderr, "  1 = release SMC RISC-V from reset\n");
    fprintf(stderr, "  --kbl1 = hold reset, load <build_dir>/zephyr/zephyr.bin, release reset\n");
    fprintf(stderr, "  --blop5 --sysbuild = boot blop5 using all images from a sysbuild directory\n");
    fprintf(stderr, "  --bl0 = hold reset, wipe SRAM, set RESET_VECTOR[0] to 0xC0040000, release reset\n");
    fprintf(stderr, "Examples:\n");
    fprintf(stderr, "  %s k 0\n", prog);
    fprintf(stderr, "  %s k 1 3\n", prog);
    fprintf(stderr, "  %s m --kbl1 build\n", prog);
    fprintf(stderr, "  %s k --blop5 --sysbuild build\n", prog);
}

static int parse_mode_arg(const char *arg)
{
    if (arg == NULL || arg[0] == '\0' || arg[1] != '\0') {
        return -EINVAL;
    }

    switch (tolower((unsigned char)arg[0])) {
    case 'k':
        g_spa_base = KER_SPA_BASE_K;
        return 0;
    case 'm':
        g_spa_base = KER_SPA_BASE_M1;
        return 0;
    default:
        return -EINVAL;
    }
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
    uint32_t readback;

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

static int set_bl1_reset_vector(int fd)
{
    return set_reset_vector(fd, KER_SMC_BL1_RESET_VECTOR);
}

static int wipe_smc_sram(int fd)
{
    uint64_t base = local_addr_to_spa(KER_SMC_SRAM_BASE);
    uint64_t end = base + KER_SMC_SRAM_SIZE;

    printf("Wiping SMC SRAM at SPA=0x%012llx..0x%012llx\n",
           (unsigned long long)base, (unsigned long long)(end - 1));
    for (uint64_t spa = base; spa < end; spa += sizeof(uint32_t)) {
        int rc = write32_ioctl(fd, spa, 0u);
        if (rc) {
            fprintf(stderr, "write SRAM zero at SPA 0x%012llx failed: %s\n",
                    (unsigned long long)spa, strerror(-rc));
            return rc;
        }
    }
    return 0;
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
    uint32_t old_val;
    uint32_t new_val;
    uint32_t rb_val;
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

static int load_image_to_spa(int fd, const char *image_path, uint64_t load_spa)
{
    int rc;
    int image_fd;
    struct stat st;
    off_t offset = 0;

    image_fd = open(image_path, O_RDONLY);
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

    printf("Loading %lld bytes from %s to SPA=0x%012llx\n",
           (long long)st.st_size, image_path, (unsigned long long)load_spa);

    while (offset < st.st_size) {
        uint8_t bytes[4] = {0, 0, 0, 0};
        uint32_t word;
        ssize_t want = (st.st_size - offset >= 4) ? 4 : (ssize_t)(st.st_size - offset);
        ssize_t got = read(image_fd, bytes, (size_t)want);

        if (got < 0) {
            rc = -errno;
            fprintf(stderr, "read image %s failed at offset %lld: %s\n",
                    image_path, (long long)offset, strerror(errno));
            close(image_fd);
            return rc;
        }
        if (got != want) {
            fprintf(stderr, "short read from image %s at offset %lld\n", image_path, (long long)offset);
            close(image_fd);
            return -EIO;
        }

        if (got < 4) {
            uint32_t old_word = 0;
            rc = read32_ioctl(fd, load_spa + (uint64_t)offset, &old_word);
            if (rc) {
                fprintf(stderr, "read target word failed at SPA 0x%012llx: %s\n",
                        (unsigned long long)(load_spa + (uint64_t)offset), strerror(-rc));
                close(image_fd);
                return rc;
            }
            bytes[2] = (got > 2) ? bytes[2] : (uint8_t)((old_word >> 16) & 0xffu);
            bytes[3] = (got > 3) ? bytes[3] : (uint8_t)((old_word >> 24) & 0xffu);
            if (got < 2) {
                bytes[1] = (uint8_t)((old_word >> 8) & 0xffu);
            }
        }

        word = (uint32_t)bytes[0] |
               ((uint32_t)bytes[1] << 8) |
               ((uint32_t)bytes[2] << 16) |
               ((uint32_t)bytes[3] << 24);

        rc = write32_ioctl(fd, load_spa + (uint64_t)offset, word);
        if (rc) {
            fprintf(stderr, "write target word failed at SPA 0x%012llx: %s\n",
                    (unsigned long long)(load_spa + (uint64_t)offset), strerror(-rc));
            close(image_fd);
            return rc;
        }

        offset += got;
    }

    close(image_fd);
    return 0;
}

static int load_image_to_smc(int fd, const char *image_path)
{
    uint64_t load_spa = local_addr_to_spa(KER_SMC_BL1_RESET_VECTOR);

    printf("Using reset-vector default local=0x%08llx -> load SPA=0x%012llx\n",
           (unsigned long long)KER_SMC_BL1_RESET_VECTOR,
           (unsigned long long)load_spa);
    return load_image_to_spa(fd, image_path, load_spa);
}

static int verify_image_at_spa(int fd, const char *image_path, uint64_t load_spa)
{
    int rc;
    int image_fd;
    struct stat st;
    off_t offset = 0;

    image_fd = open(image_path, O_RDONLY);
    if (image_fd < 0) {
        fprintf(stderr, "open image %s for verify failed: %s\n", image_path, strerror(errno));
        return -errno;
    }

    if (fstat(image_fd, &st) < 0) {
        rc = -errno;
        fprintf(stderr, "stat image %s for verify failed: %s\n", image_path, strerror(errno));
        close(image_fd);
        return rc;
    }

    if (!S_ISREG(st.st_mode)) {
        fprintf(stderr, "image %s is not a regular file\n", image_path);
        close(image_fd);
        return -EINVAL;
    }

    printf("Verifying %lld bytes at SPA=0x%012llx\n",
           (long long)st.st_size, (unsigned long long)load_spa);

    while (offset < st.st_size) {
        uint8_t bytes[4] = {0, 0, 0, 0};
        uint32_t expected_word;
        uint32_t actual_word = 0;
        ssize_t want = (st.st_size - offset >= 4) ? 4 : (ssize_t)(st.st_size - offset);
        ssize_t got = read(image_fd, bytes, (size_t)want);

        if (got < 0) {
            rc = -errno;
            fprintf(stderr, "read image %s failed at offset %lld during verify: %s\n",
                    image_path, (long long)offset, strerror(errno));
            close(image_fd);
            return rc;
        }
        if (got != want) {
            fprintf(stderr, "short read from image %s at offset %lld during verify\n",
                    image_path, (long long)offset);
            close(image_fd);
            return -EIO;
        }

        rc = read32_ioctl(fd, load_spa + (uint64_t)offset, &actual_word);
        if (rc) {
            fprintf(stderr, "read target word failed at SPA 0x%012llx during verify: %s\n",
                    (unsigned long long)(load_spa + (uint64_t)offset), strerror(-rc));
            close(image_fd);
            return rc;
        }

        if (got < 4) {
            bytes[2] = (got > 2) ? bytes[2] : (uint8_t)((actual_word >> 16) & 0xffu);
            bytes[3] = (got > 3) ? bytes[3] : (uint8_t)((actual_word >> 24) & 0xffu);
            if (got < 2) {
                bytes[1] = (uint8_t)((actual_word >> 8) & 0xffu);
            }
        }

        expected_word = (uint32_t)bytes[0] |
                        ((uint32_t)bytes[1] << 8) |
                        ((uint32_t)bytes[2] << 16) |
                        ((uint32_t)bytes[3] << 24);

        if (actual_word != expected_word) {
            fprintf(stderr,
                    "verify mismatch at SPA 0x%012llx (offset %lld): expected 0x%08x got 0x%08x\n",
                    (unsigned long long)(load_spa + (uint64_t)offset),
                    (long long)offset, expected_word, actual_word);
            close(image_fd);
            return -EIO;
        }

        offset += got;
    }

    close(image_fd);
    printf("Image verify passed\n");
    return 0;
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
        usleep(10000);
        elapsed_us += 10000;
        if (elapsed_us >= BUNDLE_POLL_TIMEOUT_US) {
            fprintf(stderr, "timeout: SPA 0x%012llx expected 0x%08x, got 0x%08x\n",
                    (unsigned long long)spa, expected, val);
            return -ETIMEDOUT;
        }
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
        usleep(10000);
        elapsed_us += 10000;
        if (elapsed_us >= BUNDLE_POLL_TIMEOUT_US) {
            fprintf(stderr, "timeout: SPA 0x%012llx mask 0x%08x, got 0x%08x\n",
                    (unsigned long long)spa, mask, val);
            return -ETIMEDOUT;
        }
    }
}

/*
 * Write a minimal fw_bundle_manifest + fw_bundle_toc + fw_bundle_toc_entry[0..1] + images
 * into the staging area so the bun2 loader can parse and copy it.
 *
 * Staging area layout (all offsets from staging base):
 *   [0 .. 1183]    fw_bundle_manifest  (only payload_offset at +1160 matters)
 *   [1184 .. 1215] fw_bundle_toc header (32 bytes)
 *   [1216 .. 1431] fw_bundle_toc_entry[0]: kbl1 (216 bytes)
 *   [1432 .. 1647] fw_bundle_toc_entry[1]: mbl1 (216 bytes)
 *   [1648 .. ]     kbl1 image data, then mbl1 image data
 */
static int write_bundle_to_staging(int fd, const char *bl1_path, off_t image_size,
                                   const char *mbl1_path, off_t mbl1_size)
{
    int rc;
    uint32_t i;
    /* BL0P5 expects the bundle manifest at 0xC0067000. */
    uint64_t staging_spa = local_addr_to_spa(KER_SMC_BL1_RESET_VECTOR);
    uint32_t hdr_words = (BUNDLE_MANIFEST_SIZE + BUNDLE_TOC_HDR_SIZE + 2u * BUNDLE_TOC_ENTRY_SIZE) / 4;
    uint32_t payload_len = BUNDLE_TOC_HDR_SIZE + 2u * BUNDLE_TOC_ENTRY_SIZE + (uint32_t)image_size
                            + (uint32_t)mbl1_size;
    uint64_t toc_spa    = staging_spa + BUNDLE_MANIFEST_SIZE;
    uint64_t entry_spa  = toc_spa + BUNDLE_TOC_HDR_SIZE;
    uint64_t entry1_spa = entry_spa + BUNDLE_TOC_ENTRY_SIZE;
    uint64_t mbl1_offset = BUNDLE_IMG0_PAYLOAD_OFFSET + (uint64_t)image_size;

    printf("Staging bundle at SPA=0x%012llx: manifest+toc+entries=%u bytes, kbl1=%lld bytes\n",
           (unsigned long long)staging_spa,
           BUNDLE_MANIFEST_SIZE + BUNDLE_TOC_HDR_SIZE + 2u * BUNDLE_TOC_ENTRY_SIZE,
           (long long)image_size);

    /* Zero the header region so unset fields don't contain stale values */
    for (i = 0; i < hdr_words; i++) {
        rc = write32_ioctl(fd, staging_spa + (uint64_t)i * 4, 0u);
        if (rc) {
            fprintf(stderr, "zero staging offset %u failed: %s\n", i * 4, strerror(-rc));
            return rc;
        }
    }

    /* Manifest: only payload_offset (int64 at +1160) is read by the bun2 loader */
    rc = write32_ioctl(fd, staging_spa + BUNDLE_MANIFEST_PAYLOAD_OFF_OFF, BUNDLE_MANIFEST_SIZE);
    if (rc) return rc;
    /* high 32 bits = 0 (positive offset, already zeroed) */

    /* TOC header */
    rc = write32_ioctl(fd, toc_spa +  0, BUNDLE_TOC_ID);           /* toc_identifier */
    if (rc) return rc;
    rc = write32_ioctl(fd, toc_spa +  4, BUNDLE_TOC_VERSION_MAJOR); /* major=1, minor=0 */
    if (rc) return rc;
    rc = write32_ioctl(fd, toc_spa +  8, payload_len);              /* payload_length lo */
    if (rc) return rc;
    /* payload_length hi = 0 (already zeroed) */
    rc = write32_ioctl(fd, toc_spa + 16, 2u);                       /* image_count = 2 */
    if (rc) return rc;
    /* image_count hi + reserved = 0 (already zeroed) */

    /* TOC entry[0] */
    rc = write32_ioctl(fd, entry_spa +  0, BUNDLE_TOC_IMG_TYPE_BL1_LO); /* type lo */
    if (rc) return rc;
    rc = write32_ioctl(fd, entry_spa +  4, BUNDLE_TOC_IMG_TYPE_BL1_HI); /* type hi */
    if (rc) return rc;
    rc = write32_ioctl(fd, entry_spa +  8, BUNDLE_IMG0_PAYLOAD_OFFSET); /* offset lo (from payload start) */
    if (rc) return rc;
    /* offset hi = 0 */
    rc = write32_ioctl(fd, entry_spa + 16, (uint32_t)image_size);        /* length lo */
    if (rc) return rc;
    rc = write32_ioctl(fd, entry_spa + 20, (uint32_t)((uint64_t)image_size >> 32)); /* length hi */
    if (rc) return rc;
    /* version at +24 = 0 */
    rc = write32_ioctl(fd, entry_spa + 32, (uint32_t)KER_SMC_BL1_EXEC_ADDR);       /* load_addr lo */
    if (rc) return rc;
    /* load_addr hi = 0 */
    rc = write32_ioctl(fd, entry_spa + 40, (uint32_t)KER_SMC_BL1_EXEC_ADDR);       /* entry_point lo */
    if (rc) return rc;
    /* entry_point hi = 0 */

    /* TOC entry[1]: mbl1 (image data follows kbl1) */
    rc = write32_ioctl(fd, entry1_spa +  0, BUNDLE_TOC_IMG_TYPE_BL1_LO); /* type lo */
    if (rc) return rc;
    rc = write32_ioctl(fd, entry1_spa +  4, BUNDLE_TOC_IMG_TYPE_BL1_HI); /* type hi */
    if (rc) return rc;
    rc = write32_ioctl(fd, entry1_spa +  8, (uint32_t)mbl1_offset);      /* offset lo */
    if (rc) return rc;
    rc = write32_ioctl(fd, entry1_spa + 12, (uint32_t)(mbl1_offset >> 32)); /* offset hi */
    if (rc) return rc;
    rc = write32_ioctl(fd, entry1_spa + 16, (uint32_t)mbl1_size);           /* length lo */
    if (rc) return rc;
    rc = write32_ioctl(fd, entry1_spa + 20, (uint32_t)((uint64_t)mbl1_size >> 32)); /* length hi */
    if (rc) return rc;
    /* version at +24 = 0 */
    rc = write32_ioctl(fd, entry1_spa + 32, (uint32_t)KER_SMC_BL1_EXEC_ADDR); /* load_addr lo */
    if (rc) return rc;
    /* load_addr hi = 0 */
    rc = write32_ioctl(fd, entry1_spa + 40, (uint32_t)KER_SMC_BL1_EXEC_ADDR); /* entry_point lo */
    if (rc) return rc;
    /* entry_point hi = 0 */

    /* Write kbl1 image after both TOC entries */
    rc = load_image_to_spa(fd, bl1_path,
                           staging_spa + BUNDLE_MANIFEST_SIZE + BUNDLE_IMG0_PAYLOAD_OFFSET);
    if (rc) return rc;

    /* Write mbl1 image immediately after kbl1 */
    if (mbl1_path) {
        rc = load_image_to_spa(fd, mbl1_path,
                               staging_spa + BUNDLE_MANIFEST_SIZE + mbl1_offset);
        if (rc) return rc;
    }

    return 0;
}

static int write_mis_bundle_to_staging(int fd, const char *kmis_path, off_t kmis_size,
                                       const char *mmis_path, off_t mmis_size)
{
    int rc;
    uint32_t i;
    uint64_t staging_spa = local_addr_to_spa(KER_SMC_BL1_RESET_VECTOR);
    uint64_t toc_spa = staging_spa + BUN3_PAYLOAD_OFFSET;
    uint64_t entry_spa = toc_spa + BUNDLE_TOC_HDR_SIZE;
    uint64_t entry1_spa = entry_spa + BUNDLE_TOC_ENTRY_SIZE;
    uint64_t mmis_offset = BUNDLE_IMG0_PAYLOAD_OFFSET +
                           (((uint64_t)kmis_size + 7u) & ~7u);
    uint32_t header_words = (BUN3_PAYLOAD_OFFSET + BUNDLE_TOC_HDR_SIZE +
                             2u * BUNDLE_TOC_ENTRY_SIZE) / 4u;
    uint32_t payload_len = BUNDLE_TOC_HDR_SIZE + 2u * BUNDLE_TOC_ENTRY_SIZE +
                           (uint32_t)mmis_offset + (uint32_t)mmis_size;

    printf("Staging BUN3 at SPA=0x%012llx with K-MIS at local=0x%08llx\n",
           (unsigned long long)staging_spa,
           (unsigned long long)KER_SMC_KMIS_LOAD_ADDR);

    for (i = 0; i < header_words; i++) {
        rc = write32_ioctl(fd, staging_spa + (uint64_t)i * 4u, 0u);
        if (rc) {
            fprintf(stderr, "zero BUN3 header offset %u failed: %s\n", i * 4u,
                    strerror(-rc));
            return rc;
        }
    }

    rc = write32_ioctl(fd, staging_spa + BUNDLE_MANIFEST_PAYLOAD_OFF_OFF,
                       BUN3_PAYLOAD_OFFSET);
    if (rc) return rc;

    rc = write32_ioctl(fd, toc_spa + 0, BUNDLE_TOC_ID);
    if (rc) return rc;
    rc = write32_ioctl(fd, toc_spa + 4, BUNDLE_TOC_VERSION_MAJOR);
    if (rc) return rc;
    rc = write32_ioctl(fd, toc_spa + 8, payload_len);
    if (rc) return rc;
    rc = write32_ioctl(fd, toc_spa + 16, 2u);
    if (rc) return rc;

    /* TOC entry[0]: K-MIS is already in place at its linked address. */
    rc = write32_ioctl(fd, entry_spa + 0, BUNDLE_TOC_IMG_TYPE_MIS_LO);
    if (rc) return rc;
    rc = write32_ioctl(fd, entry_spa + 4, BUNDLE_TOC_IMG_TYPE_MIS_HI);
    if (rc) return rc;
    rc = write32_ioctl(fd, entry_spa + 8, BUNDLE_IMG0_PAYLOAD_OFFSET);
    if (rc) return rc;
    rc = write32_ioctl(fd, entry_spa + 16, (uint32_t)kmis_size);
    if (rc) return rc;
    rc = write32_ioctl(fd, entry_spa + 20, (uint32_t)((uint64_t)kmis_size >> 32));
    if (rc) return rc;
    rc = write32_ioctl(fd, entry_spa + 32, (uint32_t)KER_SMC_KMIS_LOAD_ADDR);
    if (rc) return rc;
    rc = write32_ioctl(fd, entry_spa + 40, (uint32_t)KER_SMC_KMIS_LOAD_ADDR);
    if (rc) return rc;

    /* TOC entry[1]: M-MIS is copied to Mimir by K-MIS. */
    rc = write32_ioctl(fd, entry1_spa + 0, BUNDLE_TOC_IMG_TYPE_MIS_LO);
    if (rc) return rc;
    rc = write32_ioctl(fd, entry1_spa + 4, BUNDLE_TOC_IMG_TYPE_MIS_HI);
    if (rc) return rc;
    rc = write32_ioctl(fd, entry1_spa + 8, (uint32_t)mmis_offset);
    if (rc) return rc;
    rc = write32_ioctl(fd, entry1_spa + 12, (uint32_t)(mmis_offset >> 32));
    if (rc) return rc;
    rc = write32_ioctl(fd, entry1_spa + 16, (uint32_t)mmis_size);
    if (rc) return rc;
    rc = write32_ioctl(fd, entry1_spa + 20, (uint32_t)((uint64_t)mmis_size >> 32));
    if (rc) return rc;
    rc = write32_ioctl(fd, entry1_spa + 32, (uint32_t)MIMIR_SMC_MIS_LOAD_ADDR);
    if (rc) return rc;
    rc = write32_ioctl(fd, entry1_spa + 36,
                       (uint32_t)(MIMIR_SMC_MIS_LOAD_ADDR >> 32));
    if (rc) return rc;
    rc = write32_ioctl(fd, entry1_spa + 40, (uint32_t)KER_SMC_KMIS_LOAD_ADDR);
    if (rc) return rc;

    rc = load_image_to_spa(fd, kmis_path,
                           staging_spa + BUN3_PAYLOAD_OFFSET + BUNDLE_IMG0_PAYLOAD_OFFSET);
    if (rc) return rc;

    return load_image_to_spa(fd, mmis_path,
                             staging_spa + BUN3_PAYLOAD_OFFSET + mmis_offset);
}

static int do_blop5_boot(int fd, const char *blop5_path, const char *bl1_path,
                         const char *mbl1_path, const char *kmis_path,
                         const char *mmis_path)
{
    int rc;
    uint32_t val;
    struct stat st;
    struct stat mbl1_st = {0};
    struct stat kmis_st = {0};
    struct stat mmis_st = {0};

    if (stat(bl1_path, &st) < 0 || !S_ISREG(st.st_mode)) {
        fprintf(stderr, "cannot stat BL1 image %s: %s\n", bl1_path, strerror(errno));
        return -errno;
    }

    if (mbl1_path && (stat(mbl1_path, &mbl1_st) < 0 || !S_ISREG(mbl1_st.st_mode))) {
        fprintf(stderr, "cannot stat mbl1 image %s: %s\n", mbl1_path, strerror(errno));
        return -errno;
    }

    if (stat(kmis_path, &kmis_st) < 0 || !S_ISREG(kmis_st.st_mode)) {
        fprintf(stderr, "cannot stat kmis image %s: %s\n", kmis_path, strerror(errno));
        return -errno;
    }

    if (stat(mmis_path, &mmis_st) < 0 || !S_ISREG(mmis_st.st_mode)) {
        fprintf(stderr, "cannot stat mmis image %s: %s\n", mmis_path, strerror(errno));
        return -errno;
    }

    /* Hold the core in reset */
    rc = set_reset_state(fd, 0u);
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

    /* B) Wait for BL0P5 to signal it is ready to receive a bundle */
    printf("Waiting for BL0P5 bundle-ready signal...\n");
    rc = poll_scratch_eq(fd, local_addr_to_spa(KER_HOST_BOOT_STATE_LOCAL),
                         HOST_BOOT_STATE_WAIT_FOR_BUNDLE);
    if (rc) return rc;

    /* C+D) Write bundle manifest, TOC entries, and images into the staging area */
    rc = write_bundle_to_staging(fd, bl1_path, st.st_size,
                                 mbl1_path, mbl1_path ? mbl1_st.st_size : 0);
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
    rc = write_mis_bundle_to_staging(fd, kmis_path, kmis_st.st_size,
                                     mmis_path, mmis_st.st_size);
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

int main(int argc, char **argv)
{
    char dev_path[64];
    long reset_state = -1;
    long dev_id = 0;
    char *endptr = NULL;
    char *image_path = NULL;
    char *blop5_path = NULL;
    char *mbl1_path = NULL;
    char *kmis_path = NULL;
    char *mmis_path = NULL;
    char *sysbuild_path = NULL;
    int image_mode = 0;
    int blop5_mode = 0;
    int verify = 0;
    int scratch = 0;
    int bl0_mode = 0;
    int direct_sysbuild_mode = 0;
    int fd;
    int rc;

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
        fprintf(stderr, "Invalid mode '%s'. Expected 'k' or 'm'.\n", argv[1]);
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

    if (blop5_mode && !sysbuild_path) {
        fprintf(stderr, "--blop5 requires --sysbuild <sysbuild_dir>\n");
        usage(argv[0]);
        return 2;
    }

    if (image_path) {
        char *p = make_image_path(image_path);
        if (!p) return 1;
        image_path = p;
    }
    if (blop5_mode) {
        blop5_path = make_image_path_from_sysbuild(sysbuild_path, "bl0p5_keraunos");
        image_path = make_image_path_from_sysbuild(sysbuild_path, "bl1_keraunos");
        mbl1_path = make_image_path_from_sysbuild(sysbuild_path, "bl1_mimir");
        kmis_path = make_image_path_from_sysbuild(sysbuild_path, "mis");
        mmis_path = make_image_path_from_sysbuild(sysbuild_path, "mis_mimir");
        if (!blop5_path || !image_path || !mbl1_path || !kmis_path || !mmis_path) return 1;
    }

    snprintf(dev_path, sizeof(dev_path), "/dev/tenstorrent/%ld", dev_id);
    fd = open_tt_dev(dev_path, KERAUNOS_PCI_DEVICE_ID);
    if (fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", dev_path, strerror(-fd));
        return 1;
    }

    if (direct_sysbuild_mode) {
        g_spa_base = KER_SPA_BASE_M1;
        rc = do_bl0_boot(fd);
        if (rc) {
            close(fd);
            return 1;
        }

        g_spa_base = KER_SPA_BASE_K;
        rc = do_bl0_boot(fd);
        if (rc) {
            close(fd);
            return 1;
        }

        rc = do_blop5_boot(fd, blop5_path, image_path, mbl1_path, kmis_path, mmis_path);
        if (rc) {
            close(fd);
            return 1;
        }
    } else if (image_mode) {
        if (blop5_mode) {
            rc = do_blop5_boot(fd, blop5_path, image_path, mbl1_path, kmis_path, mmis_path);
            if (rc) {
                close(fd);
                return 1;
            }
        } else {
            rc = set_reset_state(fd, 0u);
            if (rc) {
                close(fd);
                return 1;
            }

            rc = load_image_to_smc(fd, image_path);
            if (rc) {
                close(fd);
                return 1;
            }

            rc = verify_image_in_smc(fd, image_path);
            if (rc) {
                close(fd);
                return 1;
            }

            rc = set_bl1_reset_vector(fd);
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
