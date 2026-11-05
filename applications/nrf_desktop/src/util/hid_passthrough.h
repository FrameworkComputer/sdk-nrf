/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _HID_PASSTHROUGH_H_
#define _HID_PASSTHROUGH_H_

/**
 * @brief HID passthrough
 * @defgroup hid_passthrough HID passthrough
 * @{
 *
 * Interface between the USB state module and the HID passthrough module. In
 * passthrough mode, every USB HID instance exposes the report map of a HID
 * Service of the connected peripherals, instead of the nRF Desktop report
 * map. Instances are identified by their index (devicetree order).
 *
 * The functions are called from the USB stack context.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Feature report of a passthrough interface.
 *
 * Defined in the board's hid_passthrough_def.h.
 */
struct hid_passthrough_feature {
	/** Report ID. */
	uint8_t id;
	/** Report size, without the report ID. */
	uint16_t size;
	/** Cached value, seeded with the peripheral's default. Host GET_REPORT
	 *  requests are answered from it. Values that fit in a single ATT read
	 *  are refreshed from a peripheral when it connects.
	 */
	uint8_t *data;
	/** Mode state that the USB host owns. It is never refreshed from the
	 *  peripheral, it is written to every peripheral when it connects, and
	 *  it returns to reset_val when the USB host goes away.
	 */
	bool host_owned;
	/** Value of a host-owned (single byte) report after a USB reset. */
	uint8_t reset_val;
};

/** @brief Passthrough interface: a USB HID instance.
 *
 * Defined in the board's hid_passthrough_def.h, one per USB HID instance, in
 * devicetree order.
 */
struct hid_passthrough_iface {
	/** Report map exposed over USB: a copy of the peripheral's report map. */
	const uint8_t *report_map;
	size_t report_map_size;
	/** A HID Service is forwarded to this interface if its report map has a
	 *  top level collection with this usage.
	 */
	uint8_t usage_page;
	uint8_t usage;
	/** Input report sent as the boot report in HID boot protocol (8 bytes,
	 *  boot keyboard layout). 0 if none.
	 */
	uint8_t boot_report_id;
	/** Input from one peripheral at a time, handed over between gestures.
	 *  Used for a touchpad shared by several keyboards.
	 */
	bool exclusive;
	struct hid_passthrough_feature *features;
	size_t feature_cnt;
};

/** @brief Get the report map of a USB HID instance.
 *
 * @param[in]  idx  USB HID instance index.
 * @param[out] size Size of the report map.
 *
 * @return Pointer to the report map, or NULL if there is none.
 */
const uint8_t *hid_passthrough_report_map(size_t idx, size_t *size);

/** @brief Handle a host GET_REPORT request for a feature report.
 *
 * @param[in]  idx  USB HID instance index.
 * @param[in]  id   Report ID.
 * @param[out] buf  Buffer for the report data (without the report ID).
 * @param[in]  size Size of the buffer.
 *
 * @return Size of the report data, or a negative error code.
 */
int hid_passthrough_get_feature(size_t idx, uint8_t id, uint8_t *buf, size_t size);

/** @brief Handle a host SET_REPORT request for a feature report.
 *
 * @param[in] idx  USB HID instance index.
 * @param[in] id   Report ID.
 * @param[in] buf  Report data (without the report ID).
 * @param[in] size Size of the report data.
 *
 * @return 0 on success, or a negative error code.
 */
int hid_passthrough_set_feature(size_t idx, uint8_t id, const uint8_t *buf, size_t size);

/** @brief Handle a host SET_REPORT request for an output report.
 *
 * @param[in] idx  USB HID instance index.
 * @param[in] id   Report ID.
 * @param[in] buf  Report data (without the report ID).
 * @param[in] size Size of the report data.
 *
 * @return 0 on success, or a negative error code.
 */
int hid_passthrough_set_output(size_t idx, uint8_t id, const uint8_t *buf, size_t size);

/** @brief Get the index of the USB HID instance that is a HID report subscriber.
 *
 * Implemented by the USB state module.
 *
 * @param[in] subscriber HID report subscriber.
 *
 * @return USB HID instance index, or a negative error code if the subscriber
 *         is not a USB HID instance.
 */
int usb_state_hid_subscriber_idx(const void *subscriber);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* _HID_PASSTHROUGH_H_ */
