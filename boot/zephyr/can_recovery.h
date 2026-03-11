/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MCUBOOT_CAN_RECOVERY_H_
#define MCUBOOT_CAN_RECOVERY_H_

#ifdef __cplusplus
extern "C" {
#endif

void boot_can_recovery_check(void);
void boot_can_recovery_note_boot_attempt(void);

#ifdef __cplusplus
}
#endif

#endif /* MCUBOOT_CAN_RECOVERY_H_ */
