/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/types.h>
#include <stdio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/gatt_dm.h>

#define MODULE ble_discovery
#include <caf/events/module_state_event.h>

#include <caf/events/ble_common_event.h>
#include "ble_event.h"
#include "ble_discovery_def.h"
#include "dev_descr.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_DESKTOP_BLE_DISCOVERY_LOG_LEVEL);

enum discovery_state {
	DISCOVERY_STATE_START,
	DISCOVERY_STATE_DEV_DESCR_LLPM,
	DISCOVERY_STATE_DEV_DESCR_HWID,
	DISCOVERY_STATE_DIS,
	DISCOVERY_STATE_HIDS,

	DISCOVERY_STATE_COUNT,
};

static enum discovery_state state;

static uint16_t peer_vid;
static uint16_t peer_pid;
static enum peer_type peer_type;
static bool peer_llpm_support;
static uint8_t peer_hwid[HWID_LEN];

/* HID Service instances of the peer: how many there are and which one is being
 * discovered. There is only one unless CONFIG_DESKTOP_BLE_DISCOVERY_HIDS_ALL.
 */
static uint8_t hids_count;
static uint8_t hids_idx;

static struct bt_conn *discovering_peer_conn;
static const struct bt_uuid * const pnp_uuid = BT_UUID_DIS_PNP_ID;

static struct k_work next_discovery_step;

#define VID_POS_IN_PNP_ID	sizeof(uint8_t)
#define PID_POS_IN_PNP_ID	(VID_POS_IN_PNP_ID + sizeof(uint16_t))
#define MIN_LEN_DIS_PNP_ID	(PID_POS_IN_PNP_ID + sizeof(uint16_t))


static void peer_disconnect(struct bt_conn *conn)
{
	__ASSERT_NO_MSG(discovering_peer_conn != NULL);
	__ASSERT_NO_MSG(discovering_peer_conn == conn);
	int err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

	if (err && (err != -ENOTCONN)) {
		LOG_ERR("Cannot disconnect peer (err:%d)", err);
		module_set_state(MODULE_STATE_ERROR);
	}
	bt_conn_unref(discovering_peer_conn);
	discovering_peer_conn = NULL;
	state = DISCOVERY_STATE_START;
}

static void hids_discovery_completed(struct bt_gatt_dm *dm, void *context)
{
	__ASSERT_NO_MSG(dm != NULL);
	__ASSERT_NO_MSG(bt_gatt_dm_conn_get(dm) == discovering_peer_conn);
	BUILD_ASSERT(PEER_TYPE_COUNT <= __CHAR_BIT__, "");
	LOG_INF("HIDS discovery procedure succeeded (%" PRIu8 "/%" PRIu8 ")",
		hids_idx + 1, hids_count);

	bt_gatt_dm_data_print(dm);

	/* This module is the last one to process the events and release the
	 * discovery data.
	 */
	if ((hids_idx + 1) < hids_count) {
		struct ble_discovery_hids_event *event = new_ble_discovery_hids_event();

		event->dm = dm;
		event->hids_idx = hids_idx;
		APP_EVENT_SUBMIT(event);
		return;
	}

	struct ble_discovery_complete_event *event =
		new_ble_discovery_complete_event();

	event->dm = dm;
	event->pid = peer_pid;
	event->peer_llpm_support = peer_llpm_support;
	__ASSERT_NO_MSG(sizeof(peer_hwid) == sizeof(event->hwid));
	memcpy(event->hwid, peer_hwid, sizeof(peer_hwid));
	event->peer_type = peer_type;
	event->hids_idx = hids_idx;

	APP_EVENT_SUBMIT(event);
}

static bool dev_descr_skippable(void)
{
	return IS_ENABLED(CONFIG_DESKTOP_BLE_DISCOVERY_DEV_DESCR_OPTIONAL) &&
	       ((state == DISCOVERY_STATE_DEV_DESCR_LLPM) ||
		(state == DISCOVERY_STATE_DEV_DESCR_HWID));
}

static void skip_dev_descr(void)
{
	LOG_INF("No device description - assuming no LLPM support and zero HW ID");
	peer_llpm_support = false;
	memset(peer_hwid, 0, sizeof(peer_hwid));
	state = DISCOVERY_STATE_DIS - 1;
	k_work_submit(&next_discovery_step);
}

static void service_not_found(struct bt_conn *conn, void *context)
{
	if (dev_descr_skippable()) {
		skip_dev_descr();
		return;
	}

	LOG_ERR("Service not found");
	peer_disconnect(conn);
}

static void discovery_error(struct bt_conn *conn, int err, void *context)
{
	LOG_ERR("Error discovering peer (err:%d)", err);
	peer_disconnect(conn);
}

static void read_dev_descr_llpm(const uint8_t *ptr, uint16_t len)
{
	__ASSERT_NO_MSG(len >= DEV_DESCR_LEN);
	peer_llpm_support = ptr[DEV_DESCR_LLPM_SUPPORT_POS];
	LOG_INF("LLPM %ssupported", peer_llpm_support ? ("") : ("not "));
}

static void read_dev_descr_hwid(const uint8_t *ptr, uint16_t len)
{
	__ASSERT_NO_MSG(len == HWID_LEN);
	memcpy(peer_hwid, ptr, HWID_LEN);

	const size_t log_buf_len = 2 * len + 1;
	char log_buf[log_buf_len];
	int pos = 0;
	int err = snprintf(&log_buf[pos], log_buf_len - pos,
			   "%02x", ptr[0]);

	for (size_t i = 1; (i < len) && (err > 0); i++) {
		pos += err;
		err = snprintf(&log_buf[pos], log_buf_len - pos,
			       "%02x", ptr[i]);
	}

	if (err > 0) {
		LOG_INF("HW ID: %s", log_buf);
	} else {
		LOG_ERR("Failed to log HW ID");
	}
}

static void read_dis(const uint8_t *ptr, uint16_t len)
{
	__ASSERT_NO_MSG(len >= MIN_LEN_DIS_PNP_ID);

	peer_vid = sys_get_le16(&ptr[VID_POS_IN_PNP_ID]);
	peer_pid = sys_get_le16(&ptr[PID_POS_IN_PNP_ID]);

	LOG_INF("VID: %02x PID: %02x", peer_vid, peer_pid);
}

static uint8_t read_attr(struct bt_conn *conn, uint8_t err,
		      struct bt_gatt_read_params *params,
		      const void *data, uint16_t length)
{
	if (err) {
		LOG_ERR("Problem reading GATT (err:%" PRIu8 ")", err);
		peer_disconnect(conn);
		return BT_GATT_ITER_STOP;
	}

	__ASSERT_NO_MSG(data != NULL);
	const uint8_t *data_ptr = data;

	switch (state) {
	case DISCOVERY_STATE_DEV_DESCR_LLPM:
		read_dev_descr_llpm(data_ptr, length);
		break;

	case DISCOVERY_STATE_DEV_DESCR_HWID:
		read_dev_descr_hwid(data_ptr, length);
		break;

	case DISCOVERY_STATE_DIS:
		read_dis(data_ptr, length);
		break;

	default:
		__ASSERT_NO_MSG(false);
	}

	k_work_submit(&next_discovery_step);

	return BT_GATT_ITER_STOP;
}

static void discovery_completed(struct bt_gatt_dm *dm, void *context)
{
	__ASSERT_NO_MSG(dm != NULL);
	__ASSERT_NO_MSG(discovering_peer_conn == bt_gatt_dm_conn_get(dm));
	const struct bt_uuid *uuid = NULL;
	const struct bt_gatt_dm_attr *attr;
	int err;

	switch (state) {
	case DISCOVERY_STATE_DEV_DESCR_LLPM:
		uuid = &custom_desc_chrc_uuid.uuid;
		break;

	case DISCOVERY_STATE_DEV_DESCR_HWID:
		uuid = &hwid_chrc_uuid.uuid;
		break;

	case DISCOVERY_STATE_DIS:
		uuid = pnp_uuid;
		break;

	default:
		/* Should not happen. */
		__ASSERT_NO_MSG(false);
	}

	attr = bt_gatt_dm_char_by_uuid(dm, uuid);

	if (!attr) {
		err = bt_gatt_dm_data_release(dm);
		if (err) {
			goto error;
		}
		if (dev_descr_skippable()) {
			skip_dev_descr();
			return;
		}
		LOG_ERR("Characteristic not found - disconnecting");
		peer_disconnect(bt_gatt_dm_conn_get(dm));
		return;
	}

	static struct bt_gatt_read_params rp =  {
		.func = read_attr,
		.handle_count = 1,
		.single.offset = 0,
	};

	rp.single.handle = attr->handle + 1;

	err = bt_gatt_read(bt_gatt_dm_conn_get(dm), &rp);
	if (err) {
		LOG_ERR("GATT read problem (err:%d)", err);
		peer_disconnect(bt_gatt_dm_conn_get(dm));
	}

	err = bt_gatt_dm_data_release(dm);
	if (err) {
		goto error;
	}

	return;
error:
	LOG_ERR("Discovery data release failed (err:%d)", err);
	module_set_state(MODULE_STATE_ERROR);
}

static void start_discovery(struct bt_conn *conn,
			    const struct bt_uuid *svc_uuid,
			    bool discovery_hids)
{
	static const struct bt_gatt_dm_cb discovery_cb = {
		.completed = discovery_completed,
		.service_not_found = service_not_found,
		.error_found = discovery_error,
	};

	static const struct bt_gatt_dm_cb hids_discovery_cb = {
		.completed = hids_discovery_completed,
		.service_not_found = service_not_found,
		.error_found = discovery_error,
	};

	int err = bt_gatt_dm_start(conn, svc_uuid,
			discovery_hids ? &hids_discovery_cb : &discovery_cb,
			NULL);

	__ASSERT_NO_MSG(err != -EALREADY);
	__ASSERT_NO_MSG(err != -EINVAL);
	if (err) {
		LOG_ERR("Cannot start discovery (err:%d)", err);
		peer_disconnect(conn);
	}
}

static bool verify_peer(void)
{
	if (peer_vid != vendor_vid) {
		LOG_WRN("Invalid peripheral VID");
		return false;
	}

	for (size_t i = 0; i < ARRAY_SIZE(bt_peripherals); i++) {
		if (bt_peripherals[i].pid == peer_pid) {
			peer_type = bt_peripherals[i].peer_type;
			return true;
		}
	}

	LOG_ERR("Unrecognized peer");
	return false;
}

static uint8_t count_hids_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	if (attr) {
		if (hids_count < UINT8_MAX) {
			hids_count++;
		}
		return BT_GATT_ITER_CONTINUE;
	}

	LOG_INF("Peer has %" PRIu8 " HID Service(s)", hids_count);

	if (hids_count == 0) {
		LOG_ERR("Service not found");
		peer_disconnect(conn);
	} else {
		start_discovery(conn, BT_UUID_HIDS, true);
	}

	return BT_GATT_ITER_STOP;
}

static void start_hids_discovery(struct bt_conn *conn)
{
	if (!IS_ENABLED(CONFIG_DESKTOP_BLE_DISCOVERY_HIDS_ALL)) {
		hids_count = 1;
		start_discovery(conn, BT_UUID_HIDS, true);
		return;
	}

	/* Count the HID Services first. The last one is reported with the
	 * discovery complete event, the ones before with HIDS events.
	 */
	static struct bt_gatt_discover_params params = {
		.func = count_hids_cb,
		.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
		.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
		.type = BT_GATT_DISCOVER_PRIMARY,
	};

	params.uuid = BT_UUID_HIDS;
	hids_count = 0;

	int err = bt_gatt_discover(conn, &params);

	if (err) {
		LOG_ERR("Cannot count HID Services (err:%d)", err);
		peer_disconnect(conn);
	}
}

static void next_discovery_step_fn(struct k_work *w)
{
	BUILD_ASSERT((DISCOVERY_STATE_HIDS + 1) == DISCOVERY_STATE_COUNT,
		"HIDs must be discovered last - after device is verified");

	state++;

	switch (state) {
	case DISCOVERY_STATE_DEV_DESCR_LLPM:
	case DISCOVERY_STATE_DEV_DESCR_HWID:
		start_discovery(discovering_peer_conn, &custom_desc_uuid.uuid, false);
		break;

	case DISCOVERY_STATE_DIS:
		start_discovery(discovering_peer_conn, BT_UUID_DIS, false);
		break;

	case DISCOVERY_STATE_HIDS:
		if (!verify_peer()) {
			peer_disconnect(discovering_peer_conn);
		} else {
			start_hids_discovery(discovering_peer_conn);
		}
		break;

	case DISCOVERY_STATE_START:
	case DISCOVERY_STATE_COUNT:
	default:
		__ASSERT_NO_MSG(false);
	}
}

static void init(void)
{
	k_work_init(&next_discovery_step, next_discovery_step_fn);
	state = DISCOVERY_STATE_START;
	module_set_state(MODULE_STATE_READY);
}

static bool app_event_handler(const struct app_event_header *aeh)
{
	if (is_ble_peer_event(aeh)) {
		const struct ble_peer_event *event =
			cast_ble_peer_event(aeh);

		switch (event->state) {
		case PEER_STATE_CONNECTED:
			discovering_peer_conn = event->id;
			bt_conn_ref(discovering_peer_conn);
			hids_count = 0;
			hids_idx = 0;
			k_work_submit(&next_discovery_step);
			break;

		default:
			break;
		}
		return false;
	}

	if (is_ble_discovery_complete_event(aeh)) {
		const struct ble_discovery_complete_event *event =
			cast_ble_discovery_complete_event(aeh);

		int err = bt_gatt_dm_data_release(event->dm);

		if (err) {
			LOG_ERR("Discovery data release failed (err:%d)", err);
			module_set_state(MODULE_STATE_ERROR);
		}
		bt_conn_unref(discovering_peer_conn);
		discovering_peer_conn = NULL;
		state = DISCOVERY_STATE_START;

		return false;
	}

	if (IS_ENABLED(CONFIG_DESKTOP_BLE_DISCOVERY_HIDS_ALL) &&
	    is_ble_discovery_hids_event(aeh)) {
		const struct ble_discovery_hids_event *event =
			cast_ble_discovery_hids_event(aeh);

		int err = bt_gatt_dm_data_release(event->dm);

		if (err) {
			LOG_ERR("Discovery data release failed (err:%d)", err);
			module_set_state(MODULE_STATE_ERROR);
			return false;
		}

		/* Discover the next HID Service instance. */
		hids_idx++;
		err = bt_gatt_dm_continue(event->dm, NULL);
		if (err) {
			LOG_ERR("Cannot continue discovery (err:%d)", err);
			peer_disconnect(discovering_peer_conn);
		}

		return false;
	}

	if (is_module_state_event(aeh)) {
		const struct module_state_event *event =
			cast_module_state_event(aeh);

		if (check_state(event, MODULE_ID(main), MODULE_STATE_READY)) {
			init();
		}

		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);
	return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, ble_peer_event);
APP_EVENT_SUBSCRIBE_FINAL(MODULE, ble_discovery_complete_event);
#if CONFIG_DESKTOP_BLE_DISCOVERY_HIDS_ALL
APP_EVENT_SUBSCRIBE_FINAL(MODULE, ble_discovery_hids_event);
#endif
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
