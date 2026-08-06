// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only
//
// Keraunos reset helper via TENSTORRENT_IOCTL_KER_READ32/_KER_WRITE32.
//
// Behavior:
//   keraunos_reset 0 -> hold SMC RISC-V core in reset
//   keraunos_reset 1 -> release SMC RISC-V core from reset
//   keraunos_reset -i <image.bin> -> hold reset, load image, release reset
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

#define KER_RESET_CTRL_SPA 0x1202010020ULL
#define KER_RESET_VECTOR0_SPA 0x1202010000ULL
#define KER_SMC_CORE_LOCAL_BASE 0xC0000000ULL
#define KER_SMC_PCIE_SPA_BASE   0x1202000000ULL
/* KeraunosSmcCpu_ResetCtrl_reg_t.core0_reset_n_n0_scan */
#define KER_RESET_CTRL_CORE0_RESET_N_BIT 0u

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
    fprintf(stderr, "  %s <0|1> [device_id]\n", prog);
    fprintf(stderr, "  %s -i <image.bin> [device_id]\n", prog);
    fprintf(stderr, "  0 = hold SMC RISC-V in reset\n");
    fprintf(stderr, "  1 = release SMC RISC-V from reset\n");
    fprintf(stderr, "  -i = hold reset, load image, release reset\n");
    fprintf(stderr, "Examples:\n");
    fprintf(stderr, "  %s 0\n", prog);
    fprintf(stderr, "  %s 1\n", prog);
    fprintf(stderr, "  %s 1 3\n", prog);
    fprintf(stderr, "  %s -i build/zephyr/zephyr.bin\n", prog);
}

static int set_reset_state(int fd, uint32_t requested_state)
{
    int rc;
    uint32_t old_val;
    uint32_t new_val;
    uint32_t rb_val;
    uint32_t bit_now;
    uint32_t mask = 1u << KER_RESET_CTRL_CORE0_RESET_N_BIT;

    rc = read32_ioctl(fd, KER_RESET_CTRL_SPA, &old_val);
    if (rc) {
        fprintf(stderr, "read RESET_CTRL (0x%012llx) failed: %s\n",
                (unsigned long long)KER_RESET_CTRL_SPA, strerror(-rc));
        return rc;
    }

    new_val = old_val;
    if (requested_state == 0u) {
        new_val &= ~mask;
    } else {
        new_val |= mask;
    }

    rc = write32_ioctl(fd, KER_RESET_CTRL_SPA, new_val);
    if (rc) {
        fprintf(stderr, "write RESET_CTRL (0x%012llx) failed: %s\n",
                (unsigned long long)KER_RESET_CTRL_SPA, strerror(-rc));
        return rc;
    }

    rc = read32_ioctl(fd, KER_RESET_CTRL_SPA, &rb_val);
    if (rc) {
        fprintf(stderr, "readback RESET_CTRL (0x%012llx) failed: %s\n",
                (unsigned long long)KER_RESET_CTRL_SPA, strerror(-rc));
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
        return KER_SMC_PCIE_SPA_BASE + (addr - KER_SMC_CORE_LOCAL_BASE);
    }
    return addr;
}

static int load_image_to_smc(int fd, const char *image_path)
{
    int rc;
    int image_fd;
    struct stat st;
    off_t offset = 0;
    uint32_t reset_vector_local;
    uint64_t load_spa;

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

    rc = read32_ioctl(fd, KER_RESET_VECTOR0_SPA, &reset_vector_local);
    if (rc) {
        fprintf(stderr, "read RESET_VECTOR0 (0x%012llx) failed: %s\n",
                (unsigned long long)KER_RESET_VECTOR0_SPA, strerror(-rc));
        close(image_fd);
        return rc;
    }

    load_spa = local_addr_to_spa((uint64_t)reset_vector_local);
    printf("RESET_VECTOR0 local=0x%08x -> load SPA=0x%012llx\n",
           reset_vector_local, (unsigned long long)load_spa);
    printf("Loading %lld bytes from %s\n", (long long)st.st_size, image_path);

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

int main(int argc, char **argv)
{
    char dev_path[64];
    long reset_state = -1;
    long dev_id = 0;
    char *endptr = NULL;
    const char *image_path = NULL;
    int image_mode = 0;
    int fd;
    int rc;

    if (argc < 2 || argc > 4) {
        usage(argv[0]);
        return 2;
    }

    if (!strcmp(argv[1], "-i")) {
        image_mode = 1;
        if (argc != 3 && argc != 4) {
            usage(argv[0]);
            return 2;
        }
        image_path = argv[2];
        if (argc == 4) {
            endptr = NULL;
            dev_id = strtol(argv[3], &endptr, 0);
            if (endptr == argv[3] || *endptr != '\0' || dev_id < 0 || dev_id > 255) {
                fprintf(stderr, "Invalid device_id: %s\n", argv[3]);
                return 2;
            }
        }
    } else {
        if (argc != 2 && argc != 3) {
            usage(argv[0]);
            return 2;
        }
        reset_state = strtol(argv[1], &endptr, 0);
        if (endptr == argv[1] || *endptr != '\0' || (reset_state != 0 && reset_state != 1)) {
            fprintf(stderr, "Invalid reset state: %s\n", argv[1]);
            usage(argv[0]);
            return 2;
        }
        if (argc == 3) {
            endptr = NULL;
            dev_id = strtol(argv[2], &endptr, 0);
            if (endptr == argv[2] || *endptr != '\0' || dev_id < 0 || dev_id > 255) {
                fprintf(stderr, "Invalid device_id: %s\n", argv[2]);
                return 2;
            }
        }
    }

    snprintf(dev_path, sizeof(dev_path), "/dev/tenstorrent/%ld", dev_id);
    fd = open_tt_dev(dev_path, KERAUNOS_PCI_DEVICE_ID);
    if (fd < 0) {
        fprintf(stderr, "open %s failed: %s\n", dev_path, strerror(-fd));
        return 1;
    }

    if (image_mode) {
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

        rc = set_reset_state(fd, 1u);
        if (rc) {
            close(fd);
            return 1;
        }
    } else {
        rc = set_reset_state(fd, (uint32_t)reset_state);
        if (rc) {
            close(fd);
            return 1;
        }
    }

    close(fd);
    return 0;
}
