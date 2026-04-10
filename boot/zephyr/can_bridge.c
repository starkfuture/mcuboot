/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include "bootutil/bootutil_log.h"

BOOT_LOG_MODULE_DECLARE(mcuboot);

#define BOOT_CAN_BRIDGE_RX_QUEUE_LEN 32
#define BOOT_CAN_BRIDGE_SERVICE_PERIOD_MS 5
#define BOOT_CAN_BRIDGE_INVALID_FILTER_ID (-1)

#define BOOT_CAN_BRIDGE_BUS_A_CHOSEN stark_mcuboot_can_bridge_a
#define BOOT_CAN_BRIDGE_BUS_B_CHOSEN stark_mcuboot_can_bridge_b

#if DT_HAS_CHOSEN(BOOT_CAN_BRIDGE_BUS_A_CHOSEN) && DT_HAS_CHOSEN(BOOT_CAN_BRIDGE_BUS_B_CHOSEN)
#define BOOT_CAN_BRIDGE_DT_READY 1
#define BOOT_CAN_BRIDGE_BUS_A_NODE DT_CHOSEN(BOOT_CAN_BRIDGE_BUS_A_CHOSEN)
#define BOOT_CAN_BRIDGE_BUS_B_NODE DT_CHOSEN(BOOT_CAN_BRIDGE_BUS_B_CHOSEN)
#else
#define BOOT_CAN_BRIDGE_DT_READY 0
#endif

CAN_MSGQ_DEFINE(boot_can_bridge_bus_a_rx_msgq, BOOT_CAN_BRIDGE_RX_QUEUE_LEN);
CAN_MSGQ_DEFINE(boot_can_bridge_bus_b_rx_msgq, BOOT_CAN_BRIDGE_RX_QUEUE_LEN);

struct boot_can_bridge_bus {
	const struct device *dev;
	struct k_msgq *rx_msgq;
	int std_filter_id;
	int ext_filter_id;
};

struct boot_can_bridge_ctx {
	struct boot_can_bridge_bus bus_a;
	struct boot_can_bridge_bus bus_b;
	bool started;
};

#if BOOT_CAN_BRIDGE_DT_READY
static struct boot_can_bridge_ctx boot_can_bridge_ctx = {
	.bus_a = {
		.dev = DEVICE_DT_GET(BOOT_CAN_BRIDGE_BUS_A_NODE),
		.rx_msgq = &boot_can_bridge_bus_a_rx_msgq,
		.std_filter_id = BOOT_CAN_BRIDGE_INVALID_FILTER_ID,
		.ext_filter_id = BOOT_CAN_BRIDGE_INVALID_FILTER_ID,
	},
	.bus_b = {
		.dev = DEVICE_DT_GET(BOOT_CAN_BRIDGE_BUS_B_NODE),
		.rx_msgq = &boot_can_bridge_bus_b_rx_msgq,
		.std_filter_id = BOOT_CAN_BRIDGE_INVALID_FILTER_ID,
		.ext_filter_id = BOOT_CAN_BRIDGE_INVALID_FILTER_ID,
	},
};
#endif

static bool boot_can_bridge_should_exclude(const struct can_frame *frame)
{
	if ((frame->flags & CAN_FRAME_IDE) != 0U) {
		return false;
	}

	return (frame->id == CONFIG_BOOT_CAN_RECOVERY_RX_ID) ||
	       (frame->id == CONFIG_BOOT_CAN_RECOVERY_TX_ID) ||
	       (frame->id == CONFIG_BOOT_CAN_BRIDGE_DIAG_UART_RX_ID) ||
	       (frame->id == CONFIG_BOOT_CAN_BRIDGE_DIAG_UART_TX_ID);
}

static int boot_can_bridge_prepare_bus(const struct device *dev)
{
	enum can_state state;
	int rc;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	rc = can_get_state(dev, &state, NULL);
	if ((rc == 0) && (state != CAN_STATE_STOPPED)) {
		return 0;
	}

	rc = can_set_mode(dev, CAN_MODE_NORMAL);
	if ((rc != 0) && (rc != -EALREADY)) {
		return rc;
	}

	rc = can_start(dev);
	if ((rc != 0) && (rc != -EALREADY)) {
		return rc;
	}

	return 0;
}

static int boot_can_bridge_add_filters(struct boot_can_bridge_bus *bus)
{
	static const struct can_filter std_filter = {
		.id = 0U,
		.mask = 0U,
		.flags = 0U,
	};
	static const struct can_filter ext_filter = {
		.id = 0U,
		.mask = 0U,
		.flags = CAN_FILTER_IDE,
	};

	bus->std_filter_id = can_add_rx_filter_msgq(bus->dev, bus->rx_msgq, &std_filter);
	if (bus->std_filter_id < 0) {
		return bus->std_filter_id;
	}

	bus->ext_filter_id = can_add_rx_filter_msgq(bus->dev, bus->rx_msgq, &ext_filter);
	if (bus->ext_filter_id < 0) {
		can_remove_rx_filter(bus->dev, bus->std_filter_id);
		bus->std_filter_id = BOOT_CAN_BRIDGE_INVALID_FILTER_ID;
		return bus->ext_filter_id;
	}

	return 0;
}

static void boot_can_bridge_remove_filters(struct boot_can_bridge_bus *bus)
{
	if (bus->std_filter_id >= 0) {
		can_remove_rx_filter(bus->dev, bus->std_filter_id);
		bus->std_filter_id = BOOT_CAN_BRIDGE_INVALID_FILTER_ID;
	}

	if (bus->ext_filter_id >= 0) {
		can_remove_rx_filter(bus->dev, bus->ext_filter_id);
		bus->ext_filter_id = BOOT_CAN_BRIDGE_INVALID_FILTER_ID;
	}

	k_msgq_purge(bus->rx_msgq);
}

static void boot_can_bridge_forward(struct boot_can_bridge_bus *src,
				    struct boot_can_bridge_bus *dst)
{
	struct can_frame frame;

	while (k_msgq_get(src->rx_msgq, &frame, K_NO_WAIT) == 0) {
		if (boot_can_bridge_should_exclude(&frame)) {
			continue;
		}

		(void)can_send(dst->dev, &frame, K_NO_WAIT, NULL, NULL);
	}
}

int boot_can_bridge_start(void)
{
#if !BOOT_CAN_BRIDGE_DT_READY
	return -ENODEV;
#else
	int rc;

	if (boot_can_bridge_ctx.started) {
		return 0;
	}

	if (DT_SAME_NODE(BOOT_CAN_BRIDGE_BUS_A_NODE, BOOT_CAN_BRIDGE_BUS_B_NODE)) {
		return -EINVAL;
	}

	rc = boot_can_bridge_prepare_bus(boot_can_bridge_ctx.bus_a.dev);
	if (rc != 0) {
		return rc;
	}

	rc = boot_can_bridge_prepare_bus(boot_can_bridge_ctx.bus_b.dev);
	if (rc != 0) {
		return rc;
	}

	rc = boot_can_bridge_add_filters(&boot_can_bridge_ctx.bus_a);
	if (rc != 0) {
		return rc;
	}

	rc = boot_can_bridge_add_filters(&boot_can_bridge_ctx.bus_b);
	if (rc != 0) {
		boot_can_bridge_remove_filters(&boot_can_bridge_ctx.bus_a);
		return rc;
	}

	boot_can_bridge_ctx.started = true;
	BOOT_LOG_INF("CAN bridge active on %s <-> %s",
		     boot_can_bridge_ctx.bus_a.dev->name,
		     boot_can_bridge_ctx.bus_b.dev->name);

	return 0;
#endif
}

void boot_can_bridge_stop(void)
{
#if BOOT_CAN_BRIDGE_DT_READY
	if (!boot_can_bridge_ctx.started) {
		return;
	}

	boot_can_bridge_remove_filters(&boot_can_bridge_ctx.bus_a);
	boot_can_bridge_remove_filters(&boot_can_bridge_ctx.bus_b);
	boot_can_bridge_ctx.started = false;
#endif
}

void boot_can_bridge_pump(void)
{
#if BOOT_CAN_BRIDGE_DT_READY
	if (!boot_can_bridge_ctx.started) {
		return;
	}

	boot_can_bridge_forward(&boot_can_bridge_ctx.bus_a, &boot_can_bridge_ctx.bus_b);
	boot_can_bridge_forward(&boot_can_bridge_ctx.bus_b, &boot_can_bridge_ctx.bus_a);
	k_msleep(BOOT_CAN_BRIDGE_SERVICE_PERIOD_MS);
#endif
}
