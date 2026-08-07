// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only
//
// Keraunos message queue utility via TENSTORRENT_IOCTL_KER_READ32/_KER_WRITE32.
//
// Firmware contract:
//   - SMC cpu_ctrl SCRATCH_3 stores a 32-bit pointer to the message queue header.
//   - Message queue contains request and response queues for inter-processor communication.
//
// Build:
//   gcc -O2 -Wall -Wextra -o keraunos_msg keraunos_msg.c
//
// Run:
//   ./keraunos_msg <up to 8 words>
//   ./keraunos_msg <word1> [word2] ... [word8]
//
// Examples:
//   ./keraunos_msg 0x90                        # Send test message (MSG_TYPE_TEST)
//   ./keraunos_msg 0x90 0x1234 0x5678          # Send test message with payload
//   ./keraunos_msg 0xfe 0 0 0 0 0 0 0          # Send SCRATCH_ONLY message

#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <linux/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <inttypes.h>

#define TENSTORRENT_IOCTL_MAGIC 0xFA
#define TENSTORRENT_IOCTL_GET_DEVICE_INFO _IO(TENSTORRENT_IOCTL_MAGIC, 0)
#define TENSTORRENT_IOCTL_KER_READ32      _IO(TENSTORRENT_IOCTL_MAGIC, 16)
#define TENSTORRENT_IOCTL_KER_WRITE32     _IO(TENSTORRENT_IOCTL_MAGIC, 17)

#define TENSTORRENT_PCI_VENDOR_ID 0x1e52
#define KERAUNOS_PCI_DEVICE_ID    0xfeed

#define SMC_SPA_BASE_K 0x1202000000ULL
#define SMC_SPA_BASE_M 0x1300000000ULL

/* SMC CPU Control SCRATCH registers */
#define SMC_CPUCTRL_SCRATCH_BASE_OFFSET 0x10100ULL
#define SMC_CPUCTRL_SCRATCH_STRIDE 0x8

/* Mailbox registers for messaging */
#define SMC_MBOX_BASE_OFFSET        0x18000ULL
#define SMC_MBOX_CHANNEL_STRIDE     0x800
#define SMC_MBOX_TX_CHAN            0
#define SMC_MBOX_WRITE_DATA_OFFSET  0x00

static uint64_t g_spa_base = SMC_SPA_BASE_K;

/* Message queue constants */
#define MSG_QUEUE_SIZE              4
#define MSG_QUEUE_POINTER_WRAP      (2 * MSG_QUEUE_SIZE)
#define REQUEST_MSG_LEN             8
#define RESPONSE_MSG_LEN            8

/* Timeouts */
#define MSG_QUEUE_POLL_TIMEOUT_MS   1000
#define MSG_QUEUE_POLL_INTERVAL_US  5000

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

struct message_queue_header {
	/* 16B for CPU writes, ARC reads */
	uint32_t request_queue_wptr;
	uint32_t response_queue_rptr;
	uint32_t unused_1;
	uint32_t unused_2;

	/* 16B for ARC writes, CPU reads */
	uint32_t request_queue_rptr;
	uint32_t response_queue_wptr;
	uint32_t last_serial;
	uint32_t unused_3;
};

struct arc_msg {
	uint32_t header;
	uint32_t payload[7];
};

struct host_state {
	int fd;
	uint64_t queue_header_spa;
	struct message_queue_header mq_header;
};

static uint64_t smc_cpuctrl_scratch_spa(unsigned int idx)
{
	return g_spa_base + SMC_CPUCTRL_SCRATCH_BASE_OFFSET +
	       ((uint64_t)idx * SMC_CPUCTRL_SCRATCH_STRIDE);
}

static uint64_t smc_mbox_write_data_spa(unsigned int chan)
{
	return g_spa_base + SMC_MBOX_BASE_OFFSET +
	       ((uint64_t)chan * SMC_MBOX_CHANNEL_STRIDE) + SMC_MBOX_WRITE_DATA_OFFSET;
}

static int parse_mode_arg(const char *arg)
{
	if (arg == NULL || arg[0] == '\0' || arg[1] != '\0') {
		return -EINVAL;
	}

	switch (tolower((unsigned char)arg[0])) {
	case 'k':
		g_spa_base = SMC_SPA_BASE_K;
		return 0;
	case 'm':
		g_spa_base = SMC_SPA_BASE_M;
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

static int write64_ioctl(int fd, uint64_t spa, uint64_t value)
{
	int rc;

	rc = write32_ioctl(fd, spa, (uint32_t)(value & 0xffffffffULL));
	if (rc) {
		return rc;
	}

	return write32_ioctl(fd, spa + 4, (uint32_t)((value >> 32) & 0xffffffffULL));
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
	fprintf(stderr, "  %s <k|m> <word0> [word1] ... [word7]\n", prog);
	fprintf(stderr, "\n");
	fprintf(stderr, "Send up to 8 words as message data to firmware.\n");
	fprintf(stderr, "k uses SPA base 0x12020..., m uses SPA base 0x13000....\n");
	fprintf(stderr, "Device is fixed to /dev/tenstorrent/0.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Examples:\n");
	fprintf(stderr, "  %s k 0x90                            # Send message type 0x90\n", prog);
	fprintf(stderr, "  %s m 0x90 0x1234 0x5678              # Send with payload\n", prog);
	fprintf(stderr, "  %s k 0xfe 0 0 0 0 0 0 0              # SCRATCH_ONLY\n", prog);
}

static int read_mq_header(struct host_state *hs)
{
	int rc;
	uint32_t val = 0;
	uint32_t i;

	for (i = 0; i < 8; i++) {
		rc = read32_ioctl(hs->fd, hs->queue_header_spa + (i * 4), &val);
		if (rc) {
			return rc;
		}
		((uint32_t *)&hs->mq_header)[i] = val;
	}

	return 0;
}

static int push_request(struct host_state *hs, const struct arc_msg *msg)
{
	int rc;
	uint32_t wptr;
	uint32_t rptr;
	uint32_t num_occupied;
	uint32_t slot;
	uint64_t request_base_spa;
	uint64_t msg_spa;
	uint32_t i;
	uint64_t timeout_us;
	uint64_t elapsed_us = 0;

	/* Read current pointers */
	rc = read_mq_header(hs);
	if (rc) {
		fprintf(stderr, "Failed to read message queue header: %s\n", strerror(-rc));
		return rc;
	}

	wptr = hs->mq_header.request_queue_wptr;
	rptr = hs->mq_header.request_queue_rptr;

	/* Wait for space in queue (timeout after 1 second) */
	timeout_us = 1000000;
	while (elapsed_us < timeout_us) {
		num_occupied = (wptr - rptr) % MSG_QUEUE_POINTER_WRAP;
		if (num_occupied < MSG_QUEUE_SIZE) {
			break;
		}

		usleep(MSG_QUEUE_POLL_INTERVAL_US);
		elapsed_us += MSG_QUEUE_POLL_INTERVAL_US;

		rc = read_mq_header(hs);
		if (rc) {
			fprintf(stderr, "Failed to read message queue header: %s\n", strerror(-rc));
			return rc;
		}
		wptr = hs->mq_header.request_queue_wptr;
		rptr = hs->mq_header.request_queue_rptr;
	}

	if (num_occupied >= MSG_QUEUE_SIZE) {
		fprintf(stderr, "Timeout waiting for space in request queue\n");
		return -ETIMEDOUT;
	}

	/* Calculate slot and SPA for this message */
	slot = wptr % MSG_QUEUE_SIZE;
	request_base_spa = hs->queue_header_spa + 32; /* Skip header */
	msg_spa = request_base_spa + (slot * sizeof(struct arc_msg));

	/* Write message to queue */
	for (i = 0; i < 8; i++) {
		uint32_t value = (i == 0) ? msg->header : msg->payload[i - 1];
		rc = write32_ioctl(hs->fd, msg_spa + (i * 4), value);
		if (rc) {
			fprintf(stderr, "Failed to write message data: %s\n", strerror(-rc));
			return rc;
		}
	}

	/* Update write pointer */
	wptr = (wptr + 1) % MSG_QUEUE_POINTER_WRAP;
	hs->mq_header.request_queue_wptr = wptr;
	rc = write32_ioctl(hs->fd, hs->queue_header_spa, wptr);
	if (rc) {
		fprintf(stderr, "Failed to update write pointer: %s\n", strerror(-rc));
		return rc;
	}

	return 0;
}

static int pop_response(struct host_state *hs, struct arc_msg *msg)
{
	int rc;
	uint32_t rptr;
	uint32_t wptr;
	uint32_t num_occupied;
	uint32_t slot;
	uint64_t response_base_spa;
	uint64_t msg_spa;
	uint32_t i;
	uint64_t timeout_us;
	uint64_t elapsed_us = 0;

	/* Read current pointers */
	rc = read_mq_header(hs);
	if (rc) {
		fprintf(stderr, "Failed to read message queue header: %s\n", strerror(-rc));
		return rc;
	}

	rptr = hs->mq_header.response_queue_rptr;
	wptr = hs->mq_header.response_queue_wptr;

	/* Wait for response (timeout after 1 second) */
	timeout_us = 1000000;
	while (elapsed_us < timeout_us) {
		num_occupied = (wptr - rptr) % MSG_QUEUE_POINTER_WRAP;
		if (num_occupied > 0) {
			break;
		}

		usleep(MSG_QUEUE_POLL_INTERVAL_US);
		elapsed_us += MSG_QUEUE_POLL_INTERVAL_US;

		rc = read_mq_header(hs);
		if (rc) {
			fprintf(stderr, "Failed to read message queue header: %s\n", strerror(-rc));
			return rc;
		}
		rptr = hs->mq_header.response_queue_rptr;
		wptr = hs->mq_header.response_queue_wptr;
	}

	if (num_occupied == 0) {
		fprintf(stderr, "Timeout waiting for response\n");
		return -ETIMEDOUT;
	}

	/* Calculate slot and SPA for response */
	slot = rptr % MSG_QUEUE_SIZE;
	response_base_spa = hs->queue_header_spa + 32 + (MSG_QUEUE_SIZE * sizeof(struct arc_msg));
	msg_spa = response_base_spa + (slot * sizeof(struct arc_msg));

	/* Read response message */
	memset(msg, 0, sizeof(*msg));
	for (i = 0; i < 8; i++) {
		uint32_t value = 0;
		rc = read32_ioctl(hs->fd, msg_spa + (i * 4), &value);
		if (rc) {
			fprintf(stderr, "Failed to read response data: %s\n", strerror(-rc));
			return rc;
		}

		if (i == 0) {
			msg->header = value;
		} else {
			msg->payload[i - 1] = value;
		}
	}

	/* Update read pointer */
	rptr = (rptr + 1) % MSG_QUEUE_POINTER_WRAP;
	hs->mq_header.response_queue_rptr = rptr;
	rc = write32_ioctl(hs->fd, hs->queue_header_spa + 4, rptr);
	if (rc) {
		fprintf(stderr, "Failed to update read pointer: %s\n", strerror(-rc));
		return rc;
	}

	return 0;
}

static int poke_mailbox(int fd)
{
	int rc;

	/* Write to mailbox TX channel to notify firmware */
	rc = write64_ioctl(fd, smc_mbox_write_data_spa(SMC_MBOX_TX_CHAN), 0);
	if (rc) {
		fprintf(stderr, "Failed to poke mailbox: %s\n", strerror(-rc));
		return rc;
	}

	return 0;
}

int main(int argc, char **argv)
{
	struct host_state hs;
	struct arc_msg request = {0};
	struct arc_msg response = {0};
	int32_t word_count = 0;
	int rc;
	int i;
	char device_path[64];
	uint32_t parsed_val;
	char *endptr;

	if (argc < 3) {
		usage(argv[0]);
		return 1;
	}

	rc = parse_mode_arg(argv[1]);
	if (rc) {
		fprintf(stderr, "Invalid mode '%s'. Expected 'k' or 'm'.\n", argv[1]);
		usage(argv[0]);
		return 1;
	}

	/* Parse arguments */
	for (i = 2; i < argc; i++) {
		errno = 0;
		parsed_val = (uint32_t)strtoul(argv[i], &endptr, 0);

		if (errno != 0 || endptr == argv[i] || *endptr != '\0') {
			fprintf(stderr, "Invalid argument: %s\n", argv[i]);
			usage(argv[0]);
			return 1;
		}

		if (word_count >= (int32_t)REQUEST_MSG_LEN) {
			fprintf(stderr, "Too many message words (max %d)\n", REQUEST_MSG_LEN);
			return 1;
		}

		if (word_count == 0) {
			request.header = parsed_val;
		} else {
			request.payload[word_count - 1] = parsed_val;
		}
		word_count++;
	}

	if (word_count == 0) {
		fprintf(stderr, "No message data provided\n");
		usage(argv[0]);
		return 1;
	}

	/* Construct device path */
	snprintf(device_path, sizeof(device_path), "/dev/tenstorrent/0");

	/* Open device */
	hs.fd = open_tt_dev(device_path, KERAUNOS_PCI_DEVICE_ID);
	if (hs.fd < 0) {
		fprintf(stderr, "Failed to open device %s: %s\n", device_path, strerror(-hs.fd));
		return 1;
	}

	/* Get message queue info pointer from SCRATCH_3 */
	uint32_t mq_info_ptr = 0;
	uint32_t mq_base_ptr = 0;
	rc = read32_ioctl(hs.fd, smc_cpuctrl_scratch_spa(3), &mq_info_ptr);
	if (rc) {
		fprintf(stderr, "Failed to read SCRATCH_3: %s\n", strerror(-rc));
		close(hs.fd);
		return 1;
	}

	if (mq_info_ptr == 0) {
		fprintf(stderr, "Message queue not initialized (SCRATCH_3 = 0)\n");
		close(hs.fd);
		return 1;
	}

	/* Convert message_queue_info pointer to SPA */
	uint64_t mq_info_spa = g_spa_base + (mq_info_ptr & 0xfffffffULL);

	/* Read message_queue_info[0] which contains pointer to message_queues */
	rc = read32_ioctl(hs.fd, mq_info_spa, &mq_base_ptr);
	if (rc) {
		fprintf(stderr, "Failed to read message_queue_info[0]: %s\n", strerror(-rc));
		close(hs.fd);
		return 1;
	}

	if (mq_base_ptr == 0) {
		fprintf(stderr, "Message queue base pointer is NULL\n");
		close(hs.fd);
		return 1;
	}

	/* Convert message_queues base pointer to SPA */
	hs.queue_header_spa = g_spa_base + (mq_base_ptr & 0xfffffffULL);

	printf("Message queue info SPA: 0x%012" PRIx64 "\n", mq_info_spa);
	printf("Message queue header SPA: 0x%012" PRIx64 "\n", hs.queue_header_spa);
	printf("Sending %d word(s):\n", word_count);
	printf("  header: 0x%08x\n", request.header);
	for (i = 0; i < word_count - 1; i++) {
		printf("  payload[%d]: 0x%08x\n", i, request.payload[i]);
	}

	/* Push request to queue */
	rc = push_request(&hs, &request);
	if (rc) {
		fprintf(stderr, "Failed to push request: %s\n", strerror(-rc));
		close(hs.fd);
		return 1;
	}

	printf("Request queued, poking mailbox...\n");

	/* Poke mailbox to wake up firmware */
	rc = poke_mailbox(hs.fd);
	if (rc) {
		fprintf(stderr, "Failed to poke mailbox: %s\n", strerror(-rc));
		close(hs.fd);
		return 1;
	}

	/* Wait for and read response */
	printf("Waiting for response...\n");
	rc = pop_response(&hs, &response);
	if (rc) {
		fprintf(stderr, "Failed to get response: %s\n", strerror(-rc));
		close(hs.fd);
		return 1;
	}

	printf("Response received:\n");
	printf("  header: 0x%08x\n", response.header);
	for (i = 0; i < (int)RESPONSE_MSG_LEN - 1; i++) {
		printf("  payload[%d]: 0x%08x\n", i, response.payload[i]);
	}

	close(hs.fd);
	return 0;
}
