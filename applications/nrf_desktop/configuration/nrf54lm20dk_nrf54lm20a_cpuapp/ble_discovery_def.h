/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/* This configuration file defines discovery parameters for BLE central.
 * Used only by ble_discovery module.
 */

/* This structure enforces the header file is included only once in the build.
 * Violating this requirement triggers a multiple definition error at link time.
 */
const struct {} ble_discovery_include_once;

#include "ble_event.h"

struct bt_peripheral {
	uint16_t pid;
	enum peer_type peer_type;
};

/* Framework (USB-IF VID), as in the Daisy keyboard's DIS PnP ID. */
static const uint16_t vendor_vid = 0x32AC;

/* Peripherals parameters. */
static const struct bt_peripheral bt_peripherals[] = {
	{
		/* Daisy: Framework Wireless Touchpad Keyboard */
		.pid = 0x0034,
		.peer_type = PEER_TYPE_KEYBOARD,
	},
};
