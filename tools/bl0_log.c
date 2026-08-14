// SPDX-License-Identifier: GPL-2.0-only
//
// Passive SMC BL0 status-ring reader over the KER_READ32 ioctl.
//
// Build:
//   gcc -O2 -Wall -Wextra -Werror -o bl0_log bl0_log.c
//
// Run:
//   ./bl0_log <k|m> [device_id]
//   ./bl0_log <k|m> [device_id] --follow --interval-ms <milliseconds>

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#define TENSTORRENT_IOCTL_MAGIC 0xFA
#define TENSTORRENT_IOCTL_GET_DEVICE_INFO _IO(TENSTORRENT_IOCTL_MAGIC, 0)
#define TENSTORRENT_IOCTL_KER_READ32 _IO(TENSTORRENT_IOCTL_MAGIC, 16)

#define TENSTORRENT_PCI_VENDOR_ID 0x1e52
#define KERAUNOS_PCI_DEVICE_ID 0xfeed

#define KER_SPA_BASE 0x1202000000ULL
#define MIMIR_SPA_BASE 0x1300000000ULL
#define SMC_SRAM_BASE 0xC0060000ULL
#define SMC_SRAM_SIZE 0x100000ULL
#define SMC_STATUS_BUFFER_ADDR (SMC_SRAM_BASE + SMC_SRAM_SIZE - 2 * 0x80C)
#define SMC_RING_CAPACITY 512u
#define SMC_RING_HEADER_WORDS 3u
#define SMC_RING_ENTRY_OFFSET (SMC_RING_HEADER_WORDS * sizeof(uint32_t))
#define SMC_RING_ENTRY_COUNT 512u
#define SMC_RING_ENTRY_STRIDE sizeof(uint32_t)
#define DEFAULT_INTERVAL_MS 100u

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

static uint64_t g_spa_base;

static int read32_ioctl(int fd, uint64_t spa, uint32_t *value)
{
    struct tenstorrent_ker_read32 request = {0};

    request.argsz = sizeof(request);
    request.addr = spa;
    if (ioctl(fd, TENSTORRENT_IOCTL_KER_READ32, &request) < 0) {
        return -errno;
    }
    *value = request.value;
    return 0;
}

static int open_tt_dev(long device_id)
{
    char path[64];
    int fd;
    struct tenstorrent_get_device_info info = {
        .in.output_size_bytes = sizeof(info.out),
    };

    snprintf(path, sizeof(path), "/dev/tenstorrent/%ld", device_id);
    fd = open(path, O_RDONLY | O_APPEND);
    if (fd < 0) {
        return -errno;
    }
    if (ioctl(fd, TENSTORRENT_IOCTL_GET_DEVICE_INFO, &info) < 0) {
        int rc = -errno;
        close(fd);
        return rc;
    }
    if (info.out.vendor_id != TENSTORRENT_PCI_VENDOR_ID ||
        info.out.device_id != KERAUNOS_PCI_DEVICE_ID) {
        close(fd);
        return -ENODEV;
    }
    return fd;
}

static uint64_t status_buffer_spa(void)
{
    return g_spa_base + (SMC_STATUS_BUFFER_ADDR - 0xC0000000ULL);
}

static const char *message_type_name(uint8_t type)
{
    switch (type) {
    case 0x01:
        return "STATUS";
    case 0x08:
        return "WARNING";
    case 0x0F:
        return "ERROR";
    default:
        return "CUSTOM";
    }
}

static const char *status_value_name(uint16_t value)
{
    switch (value) {
    case 0x0001:
        return "ROM_STARTED/STATUS_INIT";
    case 0x0010:
        return "BOOT_START";
    case 0x0020:
        return "RECOVERY_MODE";
    case 0x0021:
        return "PRIMARY_MODE";
    case 0x0022:
        return "SECONDARY_MODE";
    case 0x0025:
        return "INVALID_SECURITY_MODE";
    case 0x0030:
        return "OCCP_INIT_FAILED";
    case 0x0031:
        return "OCCP_READY";
    case 0x0040:
        return "COORDINATION_ACTIVE";
    case 0x0050:
        return "BOOT_COMPLETE";
    case 0x00ff:
        return "UNEXPECTED_EXIT";
    case 0x0100:
        return "OCCP_CMD_READ_ERROR";
    case 0x0101:
        return "OCCP_CMD_UNKNOWN";
    case 0x0110:
        return "OCCP_CMD_FAILED";
    case 0x0120:
        return "OCCP_READ_OVERFLOW";
    case 0x0121:
        return "OCCP_READ_ACCESS_DENIED";
    case 0x0130:
        return "OCCP_WRITE_OVERFLOW";
    case 0x0131:
        return "OCCP_WRITE_ACCESS_DENIED";
    case 0x0140:
        return "OCCP_VALIDATE_SECURITY";
    case 0x0141:
        return "OCCP_VALIDATE_ADDRESS_FAILED";
    case 0x0200:
        return "OCCP_JUMP_EXECUTED";
    case 0x0201:
        return "OCCP_JUMP_SECURITY";
    case 0x0202:
        return "OCCP_JUMP_READ_FAILED";
    default:
        return "";
    }
}

static void print_entry(uint32_t entry, uint32_t index)
{
    uint8_t type = (uint8_t)(entry >> 24);
    uint8_t firmware_id = (uint8_t)(entry >> 16);
    uint16_t value = (uint16_t)entry;
    const char *name = status_value_name(value);

    printf("[%u] 0x%08x fw=0x%02x type=%s value=0x%04x%s%s\n",
           index, entry, firmware_id, message_type_name(type), value,
           name[0] ? " " : "", name);
}

static int read_ring_header(int fd, uint32_t *head, uint32_t *tail, uint32_t *capacity)
{
    uint64_t base = status_buffer_spa();
    int rc;

    rc = read32_ioctl(fd, base + 0, head);
    if (rc) return rc;
    rc = read32_ioctl(fd, base + 4, tail);
    if (rc) return rc;
    rc = read32_ioctl(fd, base + 8, capacity);
    if (rc) return rc;

    if (*capacity != SMC_RING_CAPACITY || *head >= SMC_RING_CAPACITY || *tail >= SMC_RING_CAPACITY) {
        return -EPROTO;
    }
    return 0;
}

static int print_pending(int fd, uint32_t *observed_tail)
{
    uint32_t head;
    uint32_t tail;
    uint32_t capacity;
    uint32_t verify_head;
    uint32_t index;
    int rc;

    rc = read_ring_header(fd, &head, &tail, &capacity);
    if (rc) return rc;
    if (head == tail) {
        *observed_tail = tail;
        return 0;
    }

    index = tail;
    while (index != head) {
        uint32_t entry = 0;
        uint64_t address = status_buffer_spa() + SMC_RING_ENTRY_OFFSET +
                           index * SMC_RING_ENTRY_STRIDE;

        rc = read32_ioctl(fd, address, &entry);
        if (rc) return rc;
        print_entry(entry, index);
        index = (index + 1) % capacity;
    }

    rc = read_ring_header(fd, &verify_head, &tail, &capacity);
    if (rc) return rc;
    if (verify_head != head) {
        fprintf(stderr, "warning: status ring changed during snapshot; poll again\n");
    }
    *observed_tail = tail;
    return 0;
}

static void sleep_ms(unsigned int milliseconds)
{
    struct timespec delay = {
        .tv_sec = milliseconds / 1000u,
        .tv_nsec = (long)(milliseconds % 1000u) * 1000000L,
    };
    nanosleep(&delay, NULL);
}

static void usage(const char *program)
{
    fprintf(stderr, "Usage: %s <k|m> [device_id]\n", program);
    fprintf(stderr, "       %s <k|m> [device_id] --follow [--interval-ms <milliseconds>]\n", program);
}

int main(int argc, char **argv)
{
    long device_id = 0;
    unsigned int interval_ms = DEFAULT_INTERVAL_MS;
    int follow = 0;
    int fd;
    int rc;
    char *end;
    uint32_t observed_tail = 0;

    if (argc < 2 || argc > 5 || (argv[1][0] != 'k' && argv[1][0] != 'K' &&
                                  argv[1][0] != 'm' && argv[1][0] != 'M')) {
        usage(argv[0]);
        return 2;
    }
    g_spa_base = (argv[1][0] == 'm' || argv[1][0] == 'M') ? MIMIR_SPA_BASE : KER_SPA_BASE;

    for (int argument = 2; argument < argc; argument++) {
        if (strcmp(argv[argument], "--follow") == 0) {
            follow = 1;
        } else if (strcmp(argv[argument], "--interval-ms") == 0 && argument + 1 < argc) {
            unsigned long parsed = strtoul(argv[++argument], &end, 0);
            if (*argv[argument] == '\0' || *end != '\0' || parsed > 60000u) {
                usage(argv[0]);
                return 2;
            }
            interval_ms = (unsigned int)parsed;
        } else if (argument == 2) {
            device_id = strtol(argv[argument], &end, 0);
            if (*argv[argument] == '\0' || *end != '\0' || device_id < 0 || device_id > 255) {
                usage(argv[0]);
                return 2;
            }
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    fd = open_tt_dev(device_id);
    if (fd < 0) {
        fprintf(stderr, "open /dev/tenstorrent/%ld failed: %s\n", device_id, strerror(-fd));
        return 1;
    }

    printf("Reading SMC BL0 status ring at SPA 0x%012llx (%s)\n",
           (unsigned long long)status_buffer_spa(),
           g_spa_base == MIMIR_SPA_BASE ? "Mimir" : "Keraunos");
    do {
        rc = print_pending(fd, &observed_tail);
        if (rc) {
            fprintf(stderr, "status ring read failed: %s\n", strerror(-rc));
            close(fd);
            return 1;
        }
        if (follow) {
            sleep_ms(interval_ms);
        }
    } while (follow);

    close(fd);
    return 0;
}
