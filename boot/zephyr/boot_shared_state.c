#include "boot_shared_state.h"

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#define BOOT_SHARED_STATE_MAGIC 0x42535452u
#define BOOT_SHARED_STATE_VERSION 1u

struct boot_shared_state_storage {
	uint32_t magic;
	uint32_t version;
	uint32_t crc32;
	struct boot_shared_state_payload payload;
};

#if DT_HAS_CHOSEN(zephyr_boot_shared_state)
#define BOOT_SHARED_STATE_NODE DT_CHOSEN(zephyr_boot_shared_state)
#define BOOT_SHARED_STATE_ADDR DT_REG_ADDR(BOOT_SHARED_STATE_NODE)
#define BOOT_SHARED_STATE_SIZE DT_REG_SIZE(BOOT_SHARED_STATE_NODE)
#define BOOT_SHARED_STATE_SUPPORTED 1
BUILD_ASSERT(BOOT_SHARED_STATE_SIZE >= sizeof(struct boot_shared_state_storage),
	     "boot shared state region is too small");
#else
#define BOOT_SHARED_STATE_SUPPORTED 0
#endif

static uint32_t boot_shared_state_crc32(const uint8_t *data, size_t len)
{
	uint32_t crc = 0U;

	crc = ~crc;
	for (size_t i = 0; i < len; ++i) {
		crc ^= data[i];
		for (int bit = 0; bit < 8; ++bit) {
			crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)-(int32_t)(crc & 1U));
		}
	}

	return ~crc;
}

#if BOOT_SHARED_STATE_SUPPORTED
static volatile struct boot_shared_state_storage *boot_shared_state_ptr(void)
{
	return (volatile struct boot_shared_state_storage *)BOOT_SHARED_STATE_ADDR;
}

static void boot_shared_state_copy_out(struct boot_shared_state_storage *dst)
{
	volatile const struct boot_shared_state_storage *src = boot_shared_state_ptr();
	uint8_t *dst_bytes = (uint8_t *)dst;
	volatile const uint8_t *src_bytes = (volatile const uint8_t *)src;

	for (size_t i = 0; i < sizeof(*dst); ++i) {
		dst_bytes[i] = src_bytes[i];
	}
}

static void boot_shared_state_copy_in(const struct boot_shared_state_storage *src)
{
	volatile struct boot_shared_state_storage *dst = boot_shared_state_ptr();
	const uint8_t *src_bytes = (const uint8_t *)src;
	volatile uint8_t *dst_bytes = (volatile uint8_t *)dst;

	for (size_t i = 0; i < sizeof(*src); ++i) {
		dst_bytes[i] = src_bytes[i];
	}
}

static bool boot_shared_state_is_valid_storage(const struct boot_shared_state_storage *state)
{
	uint32_t crc;

	if ((state->magic != BOOT_SHARED_STATE_MAGIC) ||
	    (state->version != BOOT_SHARED_STATE_VERSION)) {
		return false;
	}

	crc = boot_shared_state_crc32((const uint8_t *)&state->payload, sizeof(state->payload));
	return crc == state->crc32;
}

static void boot_shared_state_make_storage(struct boot_shared_state_storage *dst,
					       const struct boot_shared_state_payload *payload)
{
	memset(dst, 0, sizeof(*dst));
	dst->magic = BOOT_SHARED_STATE_MAGIC;
	dst->version = BOOT_SHARED_STATE_VERSION;
	dst->payload = *payload;
	dst->crc32 = boot_shared_state_crc32((const uint8_t *)&dst->payload, sizeof(dst->payload));
}

static int boot_shared_state_read_current(struct boot_shared_state_storage *state)
{
	boot_shared_state_copy_out(state);
	if (!boot_shared_state_is_valid_storage(state)) {
		return -ENOENT;
	}

	return 0;
}
#endif

bool boot_shared_state_is_supported(void)
{
	return BOOT_SHARED_STATE_SUPPORTED;
}

int boot_shared_state_read(struct boot_shared_state_payload *out)
{
#if !BOOT_SHARED_STATE_SUPPORTED
	ARG_UNUSED(out);
	return -ENODEV;
#else
	struct boot_shared_state_storage state;
	int rc;

	if (out == NULL) {
		return -EINVAL;
	}

	rc = boot_shared_state_read_current(&state);
	if (rc != 0) {
		memset(out, 0, sizeof(*out));
		return rc;
	}

	*out = state.payload;
	return 0;
#endif
}

int boot_shared_state_write(const struct boot_shared_state_payload *payload)
{
#if !BOOT_SHARED_STATE_SUPPORTED
	ARG_UNUSED(payload);
	return -ENODEV;
#else
	struct boot_shared_state_storage state;

	if (payload == NULL) {
		return -EINVAL;
	}

	boot_shared_state_make_storage(&state, payload);
	boot_shared_state_copy_in(&state);
	return 0;
#endif
}

int boot_shared_state_clear(void)
{
#if !BOOT_SHARED_STATE_SUPPORTED
	return -ENODEV;
#else
	struct boot_shared_state_storage empty = { 0 };

	boot_shared_state_copy_in(&empty);
	return 0;
#endif
}

bool boot_shared_state_consume_bootloader_request(uint32_t *window_s)
{
#if !BOOT_SHARED_STATE_SUPPORTED
	ARG_UNUSED(window_s);
	return false;
#else
	struct boot_shared_state_payload payload;

	if (boot_shared_state_read(&payload) != 0) {
		return false;
	}

	if ((payload.flags & BOOT_SHARED_STATE_FLAG_REQUEST_BOOTLOADER) == 0U) {
		return false;
	}

	if (window_s != NULL) {
		*window_s = payload.bootloader_window_s;
	}

	payload.flags &= ~BOOT_SHARED_STATE_FLAG_REQUEST_BOOTLOADER;
	payload.bootloader_window_s = 0U;
	(void)boot_shared_state_write(&payload);
	return true;
#endif
}

int boot_shared_state_request_bootloader(uint32_t window_s)
{
#if !BOOT_SHARED_STATE_SUPPORTED
	ARG_UNUSED(window_s);
	return -ENODEV;
#else
	struct boot_shared_state_payload payload;

	if (window_s == 0U) {
		return -EINVAL;
	}

	if (boot_shared_state_read(&payload) != 0) {
		memset(&payload, 0, sizeof(payload));
	}

	payload.flags |= BOOT_SHARED_STATE_FLAG_REQUEST_BOOTLOADER;
	payload.bootloader_window_s = window_s;
	return boot_shared_state_write(&payload);
#endif
}

uint32_t boot_shared_state_get_failed_boots(void)
{
#if !BOOT_SHARED_STATE_SUPPORTED
	return 0U;
#else
	struct boot_shared_state_payload payload;

	if (boot_shared_state_read(&payload) != 0) {
		return 0U;
	}

	return payload.failed_boots;
#endif
}

int boot_shared_state_set_failed_boots(uint32_t count)
{
#if !BOOT_SHARED_STATE_SUPPORTED
	ARG_UNUSED(count);
	return -ENODEV;
#else
	struct boot_shared_state_payload payload;

	if (boot_shared_state_read(&payload) != 0) {
		memset(&payload, 0, sizeof(payload));
	}

	payload.failed_boots = count;
	return boot_shared_state_write(&payload);
#endif
}

int boot_shared_state_increment_failed_boots(uint32_t *new_count)
{
#if !BOOT_SHARED_STATE_SUPPORTED
	ARG_UNUSED(new_count);
	return -ENODEV;
#else
	struct boot_shared_state_payload payload;

	if (boot_shared_state_read(&payload) != 0) {
		memset(&payload, 0, sizeof(payload));
	}

	if (payload.failed_boots < UINT32_MAX) {
		payload.failed_boots++;
	}

	if (new_count != NULL) {
		*new_count = payload.failed_boots;
	}

	return boot_shared_state_write(&payload);
#endif
}
