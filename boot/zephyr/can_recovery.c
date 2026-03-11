/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/sections.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "bootutil/bootutil.h"
#include "bootutil/bootutil_log.h"
#include "bootutil/bootutil_public.h"
#include "sysflash/sysflash.h"

BOOT_LOG_MODULE_DECLARE(mcuboot);

#define CAN_RECOVERY_PROTO_VERSION 1U
#define CAN_RECOVERY_MAGIC 0x43524e42U
#define CAN_RECOVERY_FRAME_SOF  0xA0U
#define CAN_RECOVERY_FRAME_DATA 0xB0U
#define CAN_RECOVERY_TID_MASK 0x0FU
#define CAN_RECOVERY_MSG_MAX 320U
#define CAN_RECOVERY_SEND_TIMEOUT_MS 100
#define CAN_RECOVERY_RX_POLL_MS 50
#define CAN_RECOVERY_WRITE_HDR_LEN 8U

enum can_recovery_msg_type {
	CAN_RECOVERY_HELLO_REQ = 1,
	CAN_RECOVERY_HELLO_RSP = 2,
	CAN_RECOVERY_START_REQ = 3,
	CAN_RECOVERY_START_RSP = 4,
	CAN_RECOVERY_WRITE_REQ = 5,
	CAN_RECOVERY_WRITE_RSP = 6,
	CAN_RECOVERY_FINALIZE_REQ = 7,
	CAN_RECOVERY_FINALIZE_RSP = 8,
	CAN_RECOVERY_RESET_REQ = 9,
	CAN_RECOVERY_RESET_RSP = 10,
};

enum can_recovery_status {
	CAN_RECOVERY_STATUS_OK = 0,
	CAN_RECOVERY_STATUS_BAD_STATE = 1,
	CAN_RECOVERY_STATUS_BAD_ARG = 2,
	CAN_RECOVERY_STATUS_BAD_OFFSET = 3,
	CAN_RECOVERY_STATUS_BAD_LENGTH = 4,
	CAN_RECOVERY_STATUS_BAD_CHUNK_CRC = 5,
	CAN_RECOVERY_STATUS_BAD_IMAGE_CRC = 6,
	CAN_RECOVERY_STATUS_FLASH = 7,
	CAN_RECOVERY_STATUS_CAN = 8,
	CAN_RECOVERY_STATUS_TOO_LARGE = 9,
	CAN_RECOVERY_STATUS_INTERNAL = 10,
};

struct can_recovery_retained {
	uint32_t magic;
	uint8_t boot_attempts;
	uint8_t reserved[3];
};

struct can_recovery_session {
	bool started;
	uint32_t image_size;
	uint32_t image_crc32;
	uint32_t next_offset;
};

struct can_recovery_rx_assembly {
	bool active;
	uint8_t transfer_id;
	uint8_t msg_type;
	uint16_t expected_len;
	uint16_t received_len;
	uint8_t data[CAN_RECOVERY_MSG_MAX];
};

struct can_recovery_hello_rsp {
	uint8_t proto_version;
	uint8_t status;
	uint8_t failed_boots;
	uint8_t reserved;
	uint32_t slot_size;
	uint16_t max_chunk;
} __packed;

struct can_recovery_start_req {
	uint32_t image_size;
	uint32_t image_crc32;
} __packed;

struct can_recovery_start_rsp {
	uint8_t status;
	uint8_t reserved[3];
	uint32_t slot_size;
	uint16_t max_chunk;
} __packed;

struct can_recovery_write_rsp {
	uint8_t status;
	uint8_t reserved[3];
	uint32_t next_offset;
} __packed;

struct can_recovery_finalize_rsp {
	uint8_t status;
	uint8_t pending;
	uint16_t reserved;
	uint32_t image_size;
} __packed;

BUILD_ASSERT(DT_HAS_CHOSEN(zephyr_canbus), "MCUboot CAN recovery requires zephyr,canbus");
CAN_MSGQ_DEFINE(can_recovery_rx_msgq, 32);

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
static int can_recovery_filter_id = -1;
static struct can_recovery_session can_recovery_session;
static struct can_recovery_rx_assembly can_recovery_rx;
static struct can_recovery_retained can_recovery_retained __noinit;
static uint8_t can_recovery_tx_tid;

static uint16_t can_recovery_crc16(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFFU;

	for (size_t i = 0; i < len; ++i) {
		crc ^= (uint16_t)data[i] << 8;
		for (int bit = 0; bit < 8; ++bit) {
			if ((crc & 0x8000U) != 0U) {
				crc = (uint16_t)((crc << 1) ^ 0x1021U);
			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

static uint32_t can_recovery_crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
	crc = ~crc;

	for (size_t i = 0; i < len; ++i) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; ++bit) {
			crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)-(int32_t)(crc & 1U));
		}
	}

	return ~crc;
}

static bool can_recovery_boot_is_risky(void)
{
	int swap_type = boot_swap_type();

	return (swap_type == BOOT_SWAP_TYPE_TEST) || (swap_type == BOOT_SWAP_TYPE_REVERT);
}

static void can_recovery_retained_init(void)
{
	if (can_recovery_retained.magic != CAN_RECOVERY_MAGIC) {
		can_recovery_retained.magic = CAN_RECOVERY_MAGIC;
		can_recovery_retained.boot_attempts = 0U;
	}

	if (!can_recovery_boot_is_risky()) {
		can_recovery_retained.boot_attempts = 0U;
	}
}

static bool can_recovery_failed_boot_threshold_hit(void)
{
	can_recovery_retained_init();
	return can_recovery_retained.boot_attempts >= CONFIG_BOOT_CAN_RECOVERY_FAILED_BOOT_THRESHOLD;
}

static void can_recovery_clear_boot_attempts(void)
{
	can_recovery_retained.magic = CAN_RECOVERY_MAGIC;
	can_recovery_retained.boot_attempts = 0U;
}

void boot_can_recovery_note_boot_attempt(void)
{
	can_recovery_retained_init();

	if (!can_recovery_boot_is_risky()) {
		return;
	}

	if (can_recovery_retained.boot_attempts < UINT8_MAX) {
		can_recovery_retained.boot_attempts++;
	}
}

static uint32_t can_recovery_slot_capacity(const struct flash_area *secondary)
{
	uint32_t slot_size = flash_area_get_size(secondary);

	if (slot_size <= CONFIG_BOOT_CAN_RECOVERY_SLOT_RESERVE) {
		return 0U;
	}

	return slot_size - CONFIG_BOOT_CAN_RECOVERY_SLOT_RESERVE;
}

static int can_recovery_init_bus(void)
{
	struct can_filter filter = {
		.id = CONFIG_BOOT_CAN_RECOVERY_RX_ID,
		.mask = CAN_STD_ID_MASK,
		.flags = 0U,
	};
	int rc;

	if (!device_is_ready(can_dev)) {
		BOOT_LOG_ERR("CAN recovery device not ready");
		return -ENODEV;
	}

	rc = can_set_mode(can_dev, CAN_MODE_NORMAL);
	if ((rc != 0) && (rc != -EALREADY)) {
		BOOT_LOG_ERR("CAN set mode failed: %d", rc);
		return rc;
	}

	rc = can_start(can_dev);
	if ((rc != 0) && (rc != -EALREADY)) {
		BOOT_LOG_ERR("CAN start failed: %d", rc);
		return rc;
	}

	if (can_recovery_filter_id < 0) {
		can_recovery_filter_id = can_add_rx_filter_msgq(can_dev, &can_recovery_rx_msgq, &filter);
		if (can_recovery_filter_id < 0) {
			BOOT_LOG_ERR("CAN add filter failed: %d", can_recovery_filter_id);
			return can_recovery_filter_id;
		}
	}

	return 0;
}

static int can_recovery_send_frame(const uint8_t *data, size_t len)
{
	struct can_frame frame = {
		.id = CONFIG_BOOT_CAN_RECOVERY_TX_ID,
		.flags = 0U,
		.dlc = can_bytes_to_dlc(len),
	};

	memcpy(frame.data, data, len);
	return can_send(can_dev, &frame, K_MSEC(CAN_RECOVERY_SEND_TIMEOUT_MS), NULL, NULL);
}

static int can_recovery_send_message(uint8_t msg_type, const void *payload, uint16_t payload_len)
{
	const uint8_t *bytes = payload;
	uint8_t frame[8];
	uint8_t tid = can_recovery_tx_tid++ & CAN_RECOVERY_TID_MASK;
	uint16_t offset = 0U;
	uint16_t first = MIN(payload_len, (uint16_t)4U);
	int rc;

	frame[0] = CAN_RECOVERY_FRAME_SOF | tid;
	frame[1] = msg_type;
	sys_put_le16(payload_len, &frame[2]);
	memset(&frame[4], 0, sizeof(frame) - 4);
	if (first > 0U) {
		memcpy(&frame[4], bytes, first);
		offset = first;
	}

	rc = can_recovery_send_frame(frame, 4U + first);
	if (rc != 0) {
		return rc;
	}

	while (offset < payload_len) {
		uint8_t chunk = MIN((uint16_t)7U, (uint16_t)(payload_len - offset));

		frame[0] = CAN_RECOVERY_FRAME_DATA | tid;
		memset(&frame[1], 0, sizeof(frame) - 1);
		memcpy(&frame[1], &bytes[offset], chunk);
		rc = can_recovery_send_frame(frame, 1U + chunk);
		if (rc != 0) {
			return rc;
		}
		offset += chunk;
	}

	return 0;
}

static int can_recovery_poll_message(int32_t timeout_ms, uint8_t *msg_type,
				     uint8_t *buffer, uint16_t *buffer_len)
{
	int64_t deadline = (timeout_ms >= 0) ? (k_uptime_get() + timeout_ms) : 0;

	while (true) {
		struct can_frame frame;
		int32_t remaining = timeout_ms;
		int rc;

		if (timeout_ms >= 0) {
			int64_t now = k_uptime_get();

			remaining = (deadline > now) ? (int32_t)(deadline - now) : 0;
		}

		rc = k_msgq_get(&can_recovery_rx_msgq, &frame,
			       (timeout_ms < 0) ? K_FOREVER : K_MSEC(remaining));
		if (rc != 0) {
			return rc;
		}

		uint8_t data_len = can_dlc_to_bytes(frame.dlc);
		if (data_len == 0U) {
			continue;
		}

		uint8_t frame_type = frame.data[0] & 0xF0U;
		uint8_t transfer_id = frame.data[0] & CAN_RECOVERY_TID_MASK;

		if (frame_type == CAN_RECOVERY_FRAME_SOF) {
			if (data_len < 4U) {
				continue;
			}

			can_recovery_rx.active = true;
			can_recovery_rx.transfer_id = transfer_id;
			can_recovery_rx.msg_type = frame.data[1];
			can_recovery_rx.expected_len = sys_get_le16(&frame.data[2]);
			can_recovery_rx.received_len = 0U;

			if (can_recovery_rx.expected_len > sizeof(can_recovery_rx.data)) {
				can_recovery_rx.active = false;
				continue;
			}

			uint16_t count = MIN((uint16_t)(data_len - 4U), can_recovery_rx.expected_len);
			if (count > 0U) {
				memcpy(can_recovery_rx.data, &frame.data[4], count);
				can_recovery_rx.received_len = count;
			}
		} else if ((frame_type == CAN_RECOVERY_FRAME_DATA) &&
			   can_recovery_rx.active &&
			   (can_recovery_rx.transfer_id == transfer_id)) {
			uint16_t space = can_recovery_rx.expected_len - can_recovery_rx.received_len;
			uint16_t count = MIN((uint16_t)(data_len - 1U), space);

			if (count > 0U) {
				memcpy(&can_recovery_rx.data[can_recovery_rx.received_len], &frame.data[1], count);
				can_recovery_rx.received_len += count;
			}
		} else {
			continue;
		}

		if (can_recovery_rx.active &&
		    (can_recovery_rx.received_len == can_recovery_rx.expected_len)) {
			*msg_type = can_recovery_rx.msg_type;
			*buffer_len = can_recovery_rx.expected_len;
			memcpy(buffer, can_recovery_rx.data, can_recovery_rx.expected_len);
			can_recovery_rx.active = false;
			return 0;
		}
	}
}

static int can_recovery_erase_slot(const struct flash_area *secondary)
{
	return flash_area_erase(secondary, 0, flash_area_get_size(secondary));
}

static int can_recovery_write_slot(const struct flash_area *secondary, uint32_t offset,
				   const uint8_t *data, uint16_t len)
{
	const struct device *flash_dev = flash_area_get_device(secondary);
	const struct flash_parameters *params = flash_get_parameters(flash_dev);
	uint32_t write_block = MAX(params->write_block_size, 1U);
	uint16_t padded_len;
	uint8_t scratch[CONFIG_BOOT_CAN_RECOVERY_MAX_CHUNK + 16];

	if ((offset % write_block) != 0U) {
		return -EINVAL;
	}

	if ((len % write_block) == 0U) {
		return flash_area_write(secondary, offset, data, len);
	}

	padded_len = (uint16_t)ROUND_UP(len, write_block);
	if (padded_len > sizeof(scratch)) {
		return -EINVAL;
	}

	memset(scratch, flash_area_erased_val(secondary), padded_len);
	memcpy(scratch, data, len);
	return flash_area_write(secondary, offset, scratch, padded_len);
}

static int can_recovery_crc_slot(const struct flash_area *secondary, uint32_t image_size,
				 uint32_t *crc_out)
{
	uint8_t scratch[128];
	uint32_t crc = 0U;
	uint32_t offset = 0U;

	while (offset < image_size) {
		uint32_t chunk = MIN((uint32_t)sizeof(scratch), image_size - offset);
		int rc = flash_area_read(secondary, offset, scratch, chunk);

		if (rc != 0) {
			return rc;
		}

		crc = can_recovery_crc32_update(crc, scratch, chunk);
		offset += chunk;
	}

	*crc_out = crc;
	return 0;
}

static int can_recovery_handle_hello(const struct flash_area *secondary)
{
	struct can_recovery_hello_rsp rsp = {
		.proto_version = CAN_RECOVERY_PROTO_VERSION,
		.status = CAN_RECOVERY_STATUS_OK,
		.failed_boots = can_recovery_retained.boot_attempts,
		.reserved = 0U,
		.slot_size = can_recovery_slot_capacity(secondary),
		.max_chunk = CONFIG_BOOT_CAN_RECOVERY_MAX_CHUNK,
	};

	return can_recovery_send_message(CAN_RECOVERY_HELLO_RSP, &rsp, sizeof(rsp));
}

static int can_recovery_handle_start(const struct flash_area *secondary,
				     const uint8_t *payload, uint16_t payload_len)
{
	const struct can_recovery_start_req *req = (const struct can_recovery_start_req *)payload;
	struct can_recovery_start_rsp rsp = {
		.status = CAN_RECOVERY_STATUS_OK,
		.slot_size = can_recovery_slot_capacity(secondary),
		.max_chunk = CONFIG_BOOT_CAN_RECOVERY_MAX_CHUNK,
	};
	int rc;

	if (payload_len != sizeof(*req)) {
		rsp.status = CAN_RECOVERY_STATUS_BAD_LENGTH;
		return can_recovery_send_message(CAN_RECOVERY_START_RSP, &rsp, sizeof(rsp));
	}

	if ((req->image_size == 0U) || (req->image_size > rsp.slot_size)) {
		rsp.status = CAN_RECOVERY_STATUS_TOO_LARGE;
		return can_recovery_send_message(CAN_RECOVERY_START_RSP, &rsp, sizeof(rsp));
	}

	rc = can_recovery_erase_slot(secondary);
	if (rc != 0) {
		rsp.status = CAN_RECOVERY_STATUS_FLASH;
		return can_recovery_send_message(CAN_RECOVERY_START_RSP, &rsp, sizeof(rsp));
	}

	can_recovery_session.started = true;
	can_recovery_session.image_size = req->image_size;
	can_recovery_session.image_crc32 = req->image_crc32;
	can_recovery_session.next_offset = 0U;
	can_recovery_clear_boot_attempts();
	return can_recovery_send_message(CAN_RECOVERY_START_RSP, &rsp, sizeof(rsp));
}

static int can_recovery_handle_write(const struct flash_area *secondary,
				     const uint8_t *payload, uint16_t payload_len)
{
	struct can_recovery_write_rsp rsp = { 0 };
	uint32_t offset;
	uint16_t len;
	uint16_t crc;
	int rc;

	if (!can_recovery_session.started) {
		rsp.status = CAN_RECOVERY_STATUS_BAD_STATE;
		return can_recovery_send_message(CAN_RECOVERY_WRITE_RSP, &rsp, sizeof(rsp));
	}

	if (payload_len < CAN_RECOVERY_WRITE_HDR_LEN) {
		rsp.status = CAN_RECOVERY_STATUS_BAD_LENGTH;
		return can_recovery_send_message(CAN_RECOVERY_WRITE_RSP, &rsp, sizeof(rsp));
	}

	offset = sys_get_le32(payload);
	len = sys_get_le16(payload + 4);
	crc = sys_get_le16(payload + 6);

	if ((len == 0U) || (len > CONFIG_BOOT_CAN_RECOVERY_MAX_CHUNK) ||
	    (payload_len != (uint16_t)(CAN_RECOVERY_WRITE_HDR_LEN + len))) {
		rsp.status = CAN_RECOVERY_STATUS_BAD_LENGTH;
		return can_recovery_send_message(CAN_RECOVERY_WRITE_RSP, &rsp, sizeof(rsp));
	}

	if (offset != can_recovery_session.next_offset) {
		rsp.status = CAN_RECOVERY_STATUS_BAD_OFFSET;
		rsp.next_offset = can_recovery_session.next_offset;
		return can_recovery_send_message(CAN_RECOVERY_WRITE_RSP, &rsp, sizeof(rsp));
	}

	if ((offset + len) > can_recovery_session.image_size) {
		rsp.status = CAN_RECOVERY_STATUS_TOO_LARGE;
		rsp.next_offset = can_recovery_session.next_offset;
		return can_recovery_send_message(CAN_RECOVERY_WRITE_RSP, &rsp, sizeof(rsp));
	}

	if (can_recovery_crc16(payload + CAN_RECOVERY_WRITE_HDR_LEN, len) != crc) {
		rsp.status = CAN_RECOVERY_STATUS_BAD_CHUNK_CRC;
		rsp.next_offset = can_recovery_session.next_offset;
		return can_recovery_send_message(CAN_RECOVERY_WRITE_RSP, &rsp, sizeof(rsp));
	}

	rc = can_recovery_write_slot(secondary, offset, payload + CAN_RECOVERY_WRITE_HDR_LEN, len);
	if (rc != 0) {
		rsp.status = CAN_RECOVERY_STATUS_FLASH;
		rsp.next_offset = can_recovery_session.next_offset;
		return can_recovery_send_message(CAN_RECOVERY_WRITE_RSP, &rsp, sizeof(rsp));
	}

	can_recovery_session.next_offset += len;
	rsp.status = CAN_RECOVERY_STATUS_OK;
	rsp.next_offset = can_recovery_session.next_offset;
	return can_recovery_send_message(CAN_RECOVERY_WRITE_RSP, &rsp, sizeof(rsp));
}

static int can_recovery_handle_finalize(const struct flash_area *secondary,
					const uint8_t *payload, uint16_t payload_len)
{
	const struct can_recovery_start_req *req = (const struct can_recovery_start_req *)payload;
	struct can_recovery_finalize_rsp rsp = { 0 };
	uint32_t readback_crc = 0U;
	int rc;

	if (!can_recovery_session.started) {
		rsp.status = CAN_RECOVERY_STATUS_BAD_STATE;
		return can_recovery_send_message(CAN_RECOVERY_FINALIZE_RSP, &rsp, sizeof(rsp));
	}

	if (payload_len != sizeof(*req)) {
		rsp.status = CAN_RECOVERY_STATUS_BAD_LENGTH;
		return can_recovery_send_message(CAN_RECOVERY_FINALIZE_RSP, &rsp, sizeof(rsp));
	}

	if ((req->image_size != can_recovery_session.image_size) ||
	    (req->image_crc32 != can_recovery_session.image_crc32) ||
	    (can_recovery_session.next_offset != can_recovery_session.image_size)) {
		rsp.status = CAN_RECOVERY_STATUS_BAD_STATE;
		rsp.image_size = can_recovery_session.next_offset;
		return can_recovery_send_message(CAN_RECOVERY_FINALIZE_RSP, &rsp, sizeof(rsp));
	}

	rc = can_recovery_crc_slot(secondary, req->image_size, &readback_crc);
	if (rc != 0) {
		rsp.status = CAN_RECOVERY_STATUS_FLASH;
		return can_recovery_send_message(CAN_RECOVERY_FINALIZE_RSP, &rsp, sizeof(rsp));
	}

	if (readback_crc != req->image_crc32) {
		rsp.status = CAN_RECOVERY_STATUS_BAD_IMAGE_CRC;
		return can_recovery_send_message(CAN_RECOVERY_FINALIZE_RSP, &rsp, sizeof(rsp));
	}

	rc = boot_set_pending(0);
	if (rc != 0) {
		rsp.status = CAN_RECOVERY_STATUS_INTERNAL;
		return can_recovery_send_message(CAN_RECOVERY_FINALIZE_RSP, &rsp, sizeof(rsp));
	}

	can_recovery_session.started = false;
	can_recovery_clear_boot_attempts();
	rsp.status = CAN_RECOVERY_STATUS_OK;
	rsp.pending = 1U;
	rsp.image_size = req->image_size;
	return can_recovery_send_message(CAN_RECOVERY_FINALIZE_RSP, &rsp, sizeof(rsp));
}

static int can_recovery_handle_reset(void)
{
	uint8_t status = CAN_RECOVERY_STATUS_OK;
	int rc = can_recovery_send_message(CAN_RECOVERY_RESET_RSP, &status, sizeof(status));

	if (rc == 0) {
		k_msleep(20);
		sys_reboot(SYS_REBOOT_COLD);
	}

	return rc;
}

static void can_recovery_server_loop(void)
{
	const struct flash_area *secondary = NULL;
	uint8_t msg_type;
	uint16_t payload_len;
	uint8_t payload[CAN_RECOVERY_MSG_MAX];
	int rc = flash_area_open(FLASH_AREA_IMAGE_SECONDARY(0), &secondary);

	if (rc != 0) {
		BOOT_LOG_ERR("Cannot open secondary slot: %d", rc);
		return;
	}

	BOOT_LOG_INF("Entering CAN recovery on %s", can_dev->name);

	while (true) {
		rc = can_recovery_poll_message(-1, &msg_type, payload, &payload_len);
		if (rc != 0) {
			continue;
		}

		switch (msg_type) {
		case CAN_RECOVERY_HELLO_REQ:
			(void)can_recovery_handle_hello(secondary);
			break;
		case CAN_RECOVERY_START_REQ:
			(void)can_recovery_handle_start(secondary, payload, payload_len);
			break;
		case CAN_RECOVERY_WRITE_REQ:
			(void)can_recovery_handle_write(secondary, payload, payload_len);
			break;
		case CAN_RECOVERY_FINALIZE_REQ:
			(void)can_recovery_handle_finalize(secondary, payload, payload_len);
			break;
		case CAN_RECOVERY_RESET_REQ:
			(void)can_recovery_handle_reset();
			break;
		default:
			break;
		}
	}
}

void boot_can_recovery_check(void)
{
	const struct flash_area *secondary = NULL;
	uint8_t msg_type;
	uint16_t payload_len;
	uint8_t payload[CAN_RECOVERY_MSG_MAX];

	if (can_recovery_init_bus() != 0) {
		return;
	}

	if (can_recovery_failed_boot_threshold_hit()) {
		BOOT_LOG_WRN("CAN recovery forced after %u failed risky boots",
			     can_recovery_retained.boot_attempts);
		can_recovery_server_loop();
		return;
	}

	if (can_recovery_poll_message(CONFIG_BOOT_CAN_RECOVERY_WINDOW_MS, &msg_type,
				      payload, &payload_len) != 0) {
		return;
	}

	if (msg_type != CAN_RECOVERY_HELLO_REQ) {
		return;
	}

	if (flash_area_open(FLASH_AREA_IMAGE_SECONDARY(0), &secondary) == 0) {
		(void)can_recovery_handle_hello(secondary);
	}

	can_recovery_server_loop();
}
