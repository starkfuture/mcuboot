#ifndef BOOT_SHARED_STATE_H_
#define BOOT_SHARED_STATE_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BOOT_SHARED_STATE_FLAG_REQUEST_BOOTLOADER BIT(0)

struct boot_shared_state_payload {
	uint32_t flags;
	uint32_t bootloader_window_s;
	uint32_t failed_boots;
	uint32_t reserved;
};

bool boot_shared_state_is_supported(void);
int boot_shared_state_read(struct boot_shared_state_payload *out);
int boot_shared_state_write(const struct boot_shared_state_payload *state);
int boot_shared_state_clear(void);
bool boot_shared_state_consume_bootloader_request(uint32_t *window_s);
int boot_shared_state_request_bootloader(uint32_t window_s);
uint32_t boot_shared_state_get_failed_boots(void);
int boot_shared_state_set_failed_boots(uint32_t count);
int boot_shared_state_increment_failed_boots(uint32_t *new_count);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_SHARED_STATE_H_ */
