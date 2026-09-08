// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only
//
// Keraunos VUART console via TENSTORRENT_IOCTL_KER_READ32/_KER_WRITE32.
//
// Firmware contract:
//   - SMC cpu_ctrl SCRATCH_2 stores a 32-bit pointer to a VUART descriptor.
//   - Descriptor layout follows struct tt_vuart (same as BH tooling).
//
// Build:
//   gcc -O2 -Wall -Wextra -o keraunos_vuart_console keraunos_vuart_console.c
//
// Run:
//   ./keraunos_vuart_console <device_id>
//   ./keraunos_vuart_console --helper <device_id>
//

#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <linux/types.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#define TENSTORRENT_IOCTL_MAGIC 0xFA
#define TENSTORRENT_IOCTL_GET_DEVICE_INFO _IO(TENSTORRENT_IOCTL_MAGIC, 0)
#define TENSTORRENT_IOCTL_KER_READ32      _IO(TENSTORRENT_IOCTL_MAGIC, 16)
#define TENSTORRENT_IOCTL_KER_WRITE32     _IO(TENSTORRENT_IOCTL_MAGIC, 17)

#define TENSTORRENT_PCI_VENDOR_ID 0x1e52
#define KERAUNOS_PCI_DEVICE_ID    0xfeed

#define KER_SMC_CORE_LOCAL_BASE 0xC0000000ULL
#define KER_SPA_BASE_K       0x1202000000ULL
#define KER_SPA_BASE_M1      0x1308000000ULL
#define KER_SCRATCH2_OFFSET  0x10110ULL
#define KER_SCRATCH0_OFFSET  0x10100ULL
#define KER_VUART_MAGIC       0x775e21a1u
#define KER_VUART_MAX_CAP     8192u
#define KER_VUART_POLL_US     10000

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

struct tt_vuart_desc {
	uint32_t magic;
	uint32_t rx_cap;
	uint32_t rx_head;
	uint32_t rx_tail;
	uint32_t tx_cap;
	uint32_t tx_head;
	uint32_t tx_oflow;
	uint32_t tx_tail;
	uint32_t version;
};

struct host_state {
	int fd;
	uint64_t desc_spa;
	struct tt_vuart_desc d;
};

static struct termios g_term_old;
static int g_term_saved;
static int g_stdin_tty;
static int g_stdin_flags_saved;
static int g_stdin_flags;
static volatile sig_atomic_t g_stop;
static uint64_t g_spa_base = KER_SPA_BASE_K;

static uint32_t buf_size(uint32_t head, uint32_t tail)
{
	return tail - head;
}

static uint32_t buf_space(uint32_t head, uint32_t tail, uint32_t cap)
{
	return cap - buf_size(head, tail);
}

static uint64_t scratch0_spa(void)
{
	return g_spa_base + KER_SCRATCH0_OFFSET;
}

static uint64_t scratch2_spa(void)
{
	return g_spa_base + KER_SCRATCH2_OFFSET;
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

static int read_u8_ioctl(int fd, uint64_t spa, uint8_t *value)
{
	uint64_t word_spa = spa & ~0x3ULL;
	uint32_t word = 0;
	uint32_t shift = (uint32_t)((spa & 0x3ULL) * 8ULL);
	int rc;

	rc = read32_ioctl(fd, word_spa, &word);
	if (rc) {
		return rc;
	}

	*value = (uint8_t)((word >> shift) & 0xffu);
	return 0;
}

static int write_u8_ioctl(int fd, uint64_t spa, uint8_t value)
{
	uint64_t word_spa = spa & ~0x3ULL;
	uint32_t word = 0;
	uint32_t shift = (uint32_t)((spa & 0x3ULL) * 8ULL);
	uint32_t mask = 0xffu << shift;
	int rc;

	rc = read32_ioctl(fd, word_spa, &word);
	if (rc) {
		return rc;
	}

	word = (word & ~mask) | ((uint32_t)value << shift);
	return write32_ioctl(fd, word_spa, word);
}

static int read_desc(struct host_state *hs)
{
	int rc;
	uint32_t words[9];
	size_t i;

	for (i = 0; i < 9; i++) {
		rc = read32_ioctl(hs->fd, hs->desc_spa + (i * 4ULL), &words[i]);
		if (rc) {
			return rc;
		}
	}

	hs->d.magic = words[0];
	hs->d.rx_cap = words[1];
	hs->d.rx_head = words[2];
	hs->d.rx_tail = words[3];
	hs->d.tx_cap = words[4];
	hs->d.tx_head = words[5];
	hs->d.tx_oflow = words[6];
	hs->d.tx_tail = words[7];
	hs->d.version = words[8];
	return 0;
}

static int write_rx_tail(struct host_state *hs, uint32_t val)
{
	return write32_ioctl(hs->fd, hs->desc_spa + (3ULL * 4ULL), val);
}

static int write_tx_head(struct host_state *hs, uint32_t val)
{
	return write32_ioctl(hs->fd, hs->desc_spa + (5ULL * 4ULL), val);
}

static uint64_t tx_buf_spa(const struct host_state *hs)
{
	return hs->desc_spa + (9ULL * 4ULL);
}

static uint64_t rx_buf_spa(const struct host_state *hs)
{
	return tx_buf_spa(hs) + hs->d.tx_cap;
}

static int drain_device_tx(struct host_state *hs)
{
	int rc;
	uint32_t head;
	uint32_t tail;
	uint32_t cap;

	rc = read_desc(hs);
	if (rc) {
		return rc;
	}

	if (hs->d.magic != KER_VUART_MAGIC) {
		return -EINVAL;
	}

	head = hs->d.tx_head;
	tail = hs->d.tx_tail;
	cap = hs->d.tx_cap;

	while (buf_size(head, tail) > 0) {
		uint32_t idx = head % cap;
		unsigned char ch;

		rc = read_u8_ioctl(hs->fd, tx_buf_spa(hs) + idx, &ch);
		if (rc) {
			return rc;
		}
		if (write(STDOUT_FILENO, &ch, 1) < 0) {
			return -errno;
		}

		head++;
	}

	if (head != hs->d.tx_head) {
		rc = write_tx_head(hs, head);
		if (rc) {
			return rc;
		}
	}

	return 0;
}

static int send_host_char(struct host_state *hs, unsigned char ch)
{
	int rc;
	uint32_t head;
	uint32_t tail;
	uint32_t cap;
	uint32_t idx;

	rc = read_desc(hs);
	if (rc) {
		return rc;
	}
	if (hs->d.magic != KER_VUART_MAGIC) {
		return -EINVAL;
	}

	head = hs->d.rx_head;
	tail = hs->d.rx_tail;
	cap = hs->d.rx_cap;

	if (buf_space(head, tail, cap) == 0) {
		return -EAGAIN;
	}

	idx = tail % cap;
	rc = write_u8_ioctl(hs->fd, rx_buf_spa(hs) + idx, ch);
	if (rc) {
		return rc;
	}

	tail++;
	return write_rx_tail(hs, tail);
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

static int set_terminal_raw(void)
{
	struct termios t;

	g_stdin_tty = isatty(STDIN_FILENO);
	if (!g_stdin_tty) {
		return 0;
	}

	if (tcgetattr(STDIN_FILENO, &g_term_old) < 0) {
		return -errno;
	}
	g_term_saved = 1;

	t = g_term_old;
	cfmakeraw(&t);
	if (tcsetattr(STDIN_FILENO, TCSANOW, &t) < 0) {
		return -errno;
	}

	g_stdin_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
	if (g_stdin_flags < 0) {
		return -errno;
	}
	g_stdin_flags_saved = 1;
	if (fcntl(STDIN_FILENO, F_SETFL, g_stdin_flags | O_NONBLOCK) < 0) {
		return -errno;
	}

	return 0;
}

static void restore_terminal(void)
{
	if (g_stdin_flags_saved) {
		(void)fcntl(STDIN_FILENO, F_SETFL, g_stdin_flags);
		g_stdin_flags_saved = 0;
	}

	if (g_term_saved) {
		tcsetattr(STDIN_FILENO, TCSAFLUSH, &g_term_old);
		g_term_saved = 0;
	}
}

static void handle_signal(int sig)
{
	(void)sig;
	g_stop = 1;
}

static void usage(const char *prog)
{
	fprintf(stderr, "Usage:\n");
	fprintf(stderr, "  %s <k|m> <device_id>\n", prog);
	fprintf(stderr, "  %s <k|m> --helper <device_id>\n", prog);
	fprintf(stderr, "  %s <k|m> -H <device_id>\n", prog);
	fprintf(stderr, "Examples:\n");
	fprintf(stderr, "  %s k 0\n", prog);
	fprintf(stderr, "  %s m --helper 0\n", prog);
}

static int dump_scratch_helper(int fd)
{
	uint32_t scratch0;
	uint32_t scratch2;
	uint64_t desc_spa;
	int rc;

	rc = read32_ioctl(fd, scratch0_spa(), &scratch0);
	if (rc) {
		return rc;
	}

	rc = read32_ioctl(fd, scratch2_spa(), &scratch2);
	if (rc) {
		return rc;
	}

	printf("SCRATCH_0 [0x%012llx] = 0x%08x\n",
	       (unsigned long long)scratch0_spa(), scratch0);
	printf("SCRATCH_2 [0x%012llx] = 0x%08x\n",
	       (unsigned long long)scratch2_spa(), scratch2);

	desc_spa = (scratch2 >= KER_SMC_CORE_LOCAL_BASE)
			   ? (g_spa_base + ((uint64_t)scratch2 - KER_SMC_CORE_LOCAL_BASE))
			   : (uint64_t)scratch2;
	printf("DESC_SPA  [converted]  = 0x%012llx\n", (unsigned long long)desc_spa);

	return 0;
}

int main(int argc, char **argv)
{
	char dev_path[64];
	const char *dev_arg = NULL;
	long dev_id;
	char *endptr = NULL;
	struct host_state hs = {0};
	int ctrl_a_pressed = 0;
	int helper_mode = 0;
	int retcode = 0;
	int rc;

	if (signal(SIGINT, handle_signal) == SIG_ERR) {
		fprintf(stderr, "failed to install SIGINT handler: %s\n", strerror(errno));
		return 1;
	}

	if (argc < 3 || argc > 4) {
		usage(argv[0]);
		return 2;
	}

	rc = parse_mode_arg(argv[1]);
	if (rc) {
		fprintf(stderr, "Invalid mode '%s'. Expected 'k' or 'm'.\n", argv[1]);
		usage(argv[0]);
		return 2;
	}

	if (argc == 3) {
		dev_arg = argv[2];
	} else if (argc == 4 && (!strcmp(argv[2], "--helper") || !strcmp(argv[2], "-H"))) {
		helper_mode = 1;
		dev_arg = argv[3];
	} else {
		usage(argv[0]);
		return 2;
	}

	dev_id = strtol(dev_arg, &endptr, 0);
	if (endptr == dev_arg || *endptr != '\0' || dev_id < 0 || dev_id > 255) {
		fprintf(stderr, "Invalid device_id: %s\n", dev_arg);
		return 2;
	}

	snprintf(dev_path, sizeof(dev_path), "/dev/tenstorrent/%ld", dev_id);
	hs.fd = open_tt_dev(dev_path, KERAUNOS_PCI_DEVICE_ID);
	if (hs.fd < 0) {
		fprintf(stderr, "open %s failed: %s\n", dev_path, strerror(-hs.fd));
		return 1;
	}

	if (helper_mode) {
		rc = dump_scratch_helper(hs.fd);
		if (rc) {
			fprintf(stderr, "helper read failed: %s\n", strerror(-rc));
			close(hs.fd);
			return 1;
		}
		close(hs.fd);
		return 0;
	}

	{
		uint32_t ptr32;

		rc = read32_ioctl(hs.fd, scratch2_spa(), &ptr32);
		hs.desc_spa = (ptr32 >= KER_SMC_CORE_LOCAL_BASE)
				    ? (g_spa_base + ((uint64_t)ptr32 - KER_SMC_CORE_LOCAL_BASE))
				    : (uint64_t)ptr32;
	}
	if (rc) {
		fprintf(stderr, "read scratch2 failed: %s\n", strerror(-rc));
		close(hs.fd);
		return 1;
	}
	if (hs.desc_spa == 0 || hs.desc_spa == 0xffffffffu) {
		fprintf(stderr, "scratch2 has invalid VUART pointer: 0x%08x\n", (uint32_t)hs.desc_spa);
		close(hs.fd);
		return 1;
	}

	rc = read_desc(&hs);
	if (rc) {
		fprintf(stderr, "read VUART descriptor @0x%llx failed: %s\n",
			(unsigned long long)hs.desc_spa, strerror(-rc));
		close(hs.fd);
		return 1;
	}

	if (hs.d.magic != KER_VUART_MAGIC) {
		fprintf(stderr, "bad VUART magic at 0x%llx: got 0x%08x expected 0x%08x\n",
			(unsigned long long)hs.desc_spa, hs.d.magic, KER_VUART_MAGIC);
		close(hs.fd);
		return 1;
	}
	if (hs.d.tx_cap == 0 || hs.d.rx_cap == 0) {
		fprintf(stderr, "invalid VUART caps tx=%u rx=%u\n", hs.d.tx_cap, hs.d.rx_cap);
		close(hs.fd);
		return 1;
	}

	fprintf(stderr,
		"Initial pointers: tx_head=%u tx_tail=%u rx_head=%u rx_tail=%u tx_oflow=%u\n",
		hs.d.tx_head, hs.d.tx_tail, hs.d.rx_head, hs.d.rx_tail, hs.d.tx_oflow);

	fprintf(stderr,
		"Keraunos VUART: desc=0x%llx tx_cap=%u rx_cap=%u version=0x%08x\n"
		"Ctrl-C to exit.\n",
		(unsigned long long)hs.desc_spa, hs.d.tx_cap, hs.d.rx_cap, hs.d.version);

	rc = set_terminal_raw();
	if (rc) {
		fprintf(stderr, "failed to set terminal raw mode: %s\n", strerror(-rc));
		close(hs.fd);
		return 1;
	}

	fprintf(stderr, "Press Ctrl-C or Ctrl-a,x to quit\n");

	for (; !g_stop;) {
		unsigned char ch;
		ssize_t n = -1;

		if (g_stdin_tty) {
			n = read(STDIN_FILENO, &ch, 1);
		}

		rc = drain_device_tx(&hs);
		if (rc && rc != -EAGAIN) {
			fprintf(stderr, "VUART read failed: %s\n", strerror(-rc));
			retcode = 1;
			break;
		}

		if (n > 0) {
			if (ctrl_a_pressed) {
				if (ch == 'x' || ch == 'X') {
					break;
				}
				ctrl_a_pressed = 0;
			} else if (ch == 0x01) {
				ctrl_a_pressed = 1;
				continue;
			} else if (ch == 0x03) {
				break;
			} else {
				rc = send_host_char(&hs, ch);
				if (rc && rc != -EAGAIN) {
					fprintf(stderr, "VUART write failed: %s\n", strerror(-rc));
					retcode = 1;
					break;
				}
			}
		} else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			fprintf(stderr, "stdin read failed: %s\n", strerror(errno));
			retcode = 1;
			break;
		}

		usleep(KER_VUART_POLL_US);
	}

	restore_terminal();
	close(hs.fd);
	return retcode;
}
