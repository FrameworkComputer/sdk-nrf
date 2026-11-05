/*
 * Copyright (c) 2026 Framework Computer Inc
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/atomic.h>

#include <bluetooth/gatt_dm.h>
#include <bluetooth/services/hogp.h>

#define MODULE hid_passthrough
#include <caf/events/module_state_event.h>

#include "hid_report_desc.h"
#include "hid_passthrough.h"
#include "hid_passthrough_def.h"

#include "hid_event.h"
#include <caf/events/ble_common_event.h>
#include "ble_event.h"
#include <caf/events/power_event.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(MODULE, CONFIG_DESKTOP_HID_PASSTHROUGH_LOG_LEVEL);

#define IFACE_COUNT		ARRAY_SIZE(hid_passthrough_ifaces)
#define HIDS_PER_PEER		CONFIG_DESKTOP_HID_PASSTHROUGH_HIDS_PER_PEER
#define PEER_COUNT		CONFIG_BT_MAX_CONN
#define SERVICE_COUNT		(PEER_COUNT * HIDS_PER_PEER)
#define REPORT_SIZE_MAX		CONFIG_DESKTOP_HID_PASSTHROUGH_REPORT_SIZE_MAX
#define QUEUE_SIZE		CONFIG_DESKTOP_HID_PASSTHROUGH_QUEUE_SIZE
#define NO_IFACE		UINT8_MAX

/* Reports the module injects ahead of the queue: releases and lifts. */
#define INJECT_MAX		4
/* Input reports of an exclusive interface tracked for a lift. */
#define LIFT_REPORT_MAX		2
/* The owner of an exclusive interface that sent nothing for this long, with a
 * contact or button down, is taken over from (and lifted).
 */
#define OWNER_STALE_MS		500
/* Retry delay of a report map read that the ATT layer refused. */
#define MAP_READ_RETRY_MS	20
/* Feature reports longer than this do not fit a single ATT read. */
#define FEATURE_READ_SIZE_MAX	64

/* Patterns of a top level collection in a report map: Usage Page, Usage,
 * Collection (Application).
 */
#define TLC_PATTERN_LEN		6

BUILD_ASSERT(IFACE_COUNT <= 8, "Interface bitmasks are 8 bit");

struct service {
	struct bt_hogp hogp;
	/* Interface the service is forwarded to, NO_IFACE if none (yet). */
	uint8_t iface;
	/* Incremented when the service is released, so that its reports still
	 * in a queue are dropped.
	 */
	uint8_t gen;
	/* Report map read: the last bytes seen, to find top level collections
	 * across chunks.
	 */
	uint8_t map_win[TLC_PATTERN_LEN];
	uint8_t map_win_len;
	uint8_t map_match_bm;
	/* Offset of the next report map chunk to read. */
	size_t map_offset;
	struct k_work_delayable map_read_retry;
	/* Feature reports still to be written to the peripheral (bit per entry
	 * of the interface's features).
	 */
	atomic_t feat_dirty;
	/* Feature refresh: the report being read. */
	struct bt_hogp_rep_info *refresh_rep;
	bool refreshed;
};

struct queued_report {
	uint32_t timestamp;
	uint8_t svc_idx;
	uint8_t gen;
	uint8_t size;
	uint8_t data[REPORT_SIZE_MAX];
};

struct stored_report {
	uint8_t size;
	uint8_t data[REPORT_SIZE_MAX];
};

struct iface_state {
	const void *subscriber;
	uint8_t pipeline_size;
	uint8_t in_flight;
	bool connected;
	bool boot;

	struct k_msgq queue;
	char __aligned(4) queue_buf[QUEUE_SIZE * sizeof(struct queued_report)];

	struct stored_report inject[INJECT_MAX];
	uint8_t inject_cnt;

	/* Exclusive interface: the service whose input reaches the host. */
	struct service *owner;
	bool owner_idle;
	uint32_t owner_last_ms;
	/* The owner's last reports that hold something down on the host. */
	struct stored_report owner_down[LIFT_REPORT_MAX];

	/* Feature reports the host wrote since the last USB reset. */
	uint32_t host_wrote_bm;

	/* Last output report of each ID the host wrote, replayed to every
	 * peripheral that connects. The data is guarded by the lock.
	 */
	struct stored_report out[2];
	atomic_t out_dirty;
};

static struct service services[SERVICE_COUNT];
static struct iface_state ifaces[IFACE_COUNT];
static struct k_spinlock lock;
static bool suspended;

static struct k_work send_work;
static struct k_work feat_work;
static struct k_work out_work;
static bool feat_write_busy;
static uint8_t feat_write_buf[FEATURE_READ_SIZE_MAX];


const uint8_t *hid_passthrough_report_map(size_t idx, size_t *size)
{
	if (idx >= IFACE_COUNT) {
		return NULL;
	}

	*size = hid_passthrough_ifaces[idx].report_map_size;
	return hid_passthrough_ifaces[idx].report_map;
}

static struct hid_passthrough_feature *find_feature(size_t idx, uint8_t id)
{
	const struct hid_passthrough_iface *def = &hid_passthrough_ifaces[idx];

	for (size_t i = 0; i < def->feature_cnt; i++) {
		if (def->features[i].id == id) {
			return &def->features[i];
		}
	}

	return NULL;
}

static size_t svc_idx(const struct service *svc)
{
	return svc - &services[0];
}

static struct service *svc_get(struct bt_hogp *hogp)
{
	return CONTAINER_OF(hogp, struct service, hogp);
}

static bool svc_is_connected(const struct service *svc)
{
	return bt_hogp_assign_check(&svc->hogp);
}

static struct bt_hogp_rep_info *find_rep(struct bt_hogp *hogp, enum bt_hids_report_type type,
					 uint8_t id)
{
	struct bt_hogp_rep_info *rep = NULL;

	while (NULL != (rep = bt_hogp_rep_next(hogp, rep))) {
		if ((bt_hogp_rep_type(rep) == type) && (bt_hogp_rep_id(rep) == id)) {
			return rep;
		}
	}

	return NULL;
}

/* Feature reports ************************************************************/

int hid_passthrough_get_feature(size_t idx, uint8_t id, uint8_t *buf, size_t size)
{
	if (idx >= IFACE_COUNT) {
		return -ENOENT;
	}

	struct hid_passthrough_feature *f = find_feature(idx, id);

	if (!f) {
		return -ENOTSUP;
	}

	size_t len = MIN(size, f->size);
	k_spinlock_key_t key = k_spin_lock(&lock);

	memcpy(buf, f->data, len);
	k_spin_unlock(&lock, key);

	return len;
}

/* Mark features to be written to every service of the interface. */
static void queue_feature_write(size_t idx, uint32_t feat_bm)
{
	for (size_t i = 0; i < ARRAY_SIZE(services); i++) {
		if (services[i].iface == idx) {
			atomic_or(&services[i].feat_dirty, feat_bm);
		}
	}

	k_work_submit(&feat_work);
}

int hid_passthrough_set_feature(size_t idx, uint8_t id, const uint8_t *buf, size_t size)
{
	if (idx >= IFACE_COUNT) {
		return -ENOENT;
	}

	struct hid_passthrough_feature *f = find_feature(idx, id);

	if (!f) {
		return -ENOTSUP;
	}

	size_t feat_idx = f - hid_passthrough_ifaces[idx].features;
	k_spinlock_key_t key = k_spin_lock(&lock);

	memcpy(f->data, buf, MIN(size, f->size));
	ifaces[idx].host_wrote_bm |= BIT(feat_idx);
	k_spin_unlock(&lock, key);

	LOG_INF("Host set feature %" PRIu8 " on interface %zu", id, idx);

	queue_feature_write(idx, BIT(feat_idx));

	return 0;
}

/* Every feature the host owns or wrote, to be asserted to a peripheral. */
static uint32_t host_feature_bm(size_t idx)
{
	const struct hid_passthrough_iface *def = &hid_passthrough_ifaces[idx];
	uint32_t bm = 0;

	for (size_t i = 0; i < def->feature_cnt; i++) {
		if (def->features[i].host_owned) {
			bm |= BIT(i);
		}
	}

	k_spinlock_key_t key = k_spin_lock(&lock);

	bm |= ifaces[idx].host_wrote_bm;
	k_spin_unlock(&lock, key);

	return bm;
}

/* The USB host went away. A real device returns its mode state to power-on
 * values on a USB reset: a host that never writes it (BIOS, macOS) would
 * otherwise inherit whatever the previous host left.
 */
static void reset_host_features(size_t idx)
{
	const struct hid_passthrough_iface *def = &hid_passthrough_ifaces[idx];
	k_spinlock_key_t key = k_spin_lock(&lock);

	for (size_t i = 0; i < def->feature_cnt; i++) {
		if (def->features[i].host_owned) {
			def->features[i].data[0] = def->features[i].reset_val;
		}
	}
	ifaces[idx].host_wrote_bm = 0;
	k_spin_unlock(&lock, key);

	if (def->feature_cnt > 0) {
		queue_feature_write(idx, host_feature_bm(idx));
	}
}

static void feat_write_done(struct bt_hogp *hogp, struct bt_hogp_rep_info *rep, uint8_t err)
{
	if (err) {
		LOG_ERR("Feature %" PRIu8 " write failed (err:%" PRIu8 ")",
			bt_hogp_rep_id(rep), err);
	}

	feat_write_busy = false;
	k_work_submit(&feat_work);
}

/* Write the dirty features to the peripherals, one at a time. A write sends
 * the newest cached value, so repeated host writes collapse.
 */
static void feat_work_fn(struct k_work *work)
{
	if (feat_write_busy) {
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(services); i++) {
		struct service *svc = &services[i];
		atomic_val_t dirty = atomic_get(&svc->feat_dirty);

		if (!dirty) {
			continue;
		}

		if ((svc->iface == NO_IFACE) || !svc->refreshed ||
		    !bt_hogp_ready_check(&svc->hogp)) {
			/* Written once the refresh is done. */
			continue;
		}

		size_t feat_idx = __builtin_ctz((uint32_t)dirty);

		atomic_clear_bit(&svc->feat_dirty, feat_idx);

		struct hid_passthrough_feature *f =
			&hid_passthrough_ifaces[svc->iface].features[feat_idx];
		struct bt_hogp_rep_info *rep = find_rep(&svc->hogp, BT_HIDS_REPORT_TYPE_FEATURE,
							f->id);

		if (!rep || (f->size > sizeof(feat_write_buf))) {
			/* Long features (the certification blob) are read only. */
			k_work_submit(&feat_work);
			return;
		}

		k_spinlock_key_t key = k_spin_lock(&lock);

		memcpy(feat_write_buf, f->data, f->size);
		k_spin_unlock(&lock, key);

		LOG_INF("Write feature %" PRIu8 " (0x%02" PRIx8 ") to service %zu",
			f->id, feat_write_buf[0], i);

		int err = bt_hogp_rep_write(&svc->hogp, rep, feat_write_done, feat_write_buf,
					    f->size);

		if (err) {
			LOG_ERR("Cannot write feature %" PRIu8 " (err:%d)", f->id, err);
			k_work_submit(&feat_work);
		} else {
			feat_write_busy = true;
		}
		return;
	}
}

static void refresh_next(struct service *svc);

static uint8_t refresh_read_cb(struct bt_hogp *hogp, struct bt_hogp_rep_info *rep, uint8_t err,
			       const uint8_t *data)
{
	struct service *svc = svc_get(hogp);
	struct hid_passthrough_feature *f = find_feature(svc->iface, bt_hogp_rep_id(rep));

	if (!err && data && f) {
		size_t feat_idx = f - hid_passthrough_ifaces[svc->iface].features;
		k_spinlock_key_t key = k_spin_lock(&lock);

		if (!f->host_owned && !(ifaces[svc->iface].host_wrote_bm & BIT(feat_idx))) {
			memcpy(f->data, data, MIN(f->size, bt_hogp_rep_size(rep)));
		}
		k_spin_unlock(&lock, key);
	}

	refresh_next(svc);

	return BT_GATT_ITER_STOP;
}

/* Read the short feature reports of a peripheral into the cache, then assert
 * the features the host owns or wrote. The peripheral retains them across
 * links, so a write from another host or session could otherwise stick.
 */
static void refresh_next(struct service *svc)
{
	while (NULL != (svc->refresh_rep = bt_hogp_rep_next(&svc->hogp, svc->refresh_rep))) {
		struct bt_hogp_rep_info *rep = svc->refresh_rep;

		if (bt_hogp_rep_type(rep) != BT_HIDS_REPORT_TYPE_FEATURE) {
			continue;
		}

		struct hid_passthrough_feature *f = find_feature(svc->iface, bt_hogp_rep_id(rep));

		if (!f || (f->size > FEATURE_READ_SIZE_MAX) || f->host_owned) {
			continue;
		}

		if (!bt_hogp_rep_read(&svc->hogp, rep, refresh_read_cb)) {
			/* Continued from the read callback. */
			return;
		}
	}

	svc->refreshed = true;
	atomic_or(&svc->feat_dirty, host_feature_bm(svc->iface));
	k_work_submit(&feat_work);
}

/* Output reports *************************************************************/

int hid_passthrough_set_output(size_t idx, uint8_t id, const uint8_t *buf, size_t size)
{
	if (idx >= IFACE_COUNT) {
		return -ENOENT;
	}

	struct iface_state *ifs = &ifaces[idx];

	if ((size + 1) > REPORT_SIZE_MAX) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	size_t i;

	for (i = 0; i < ARRAY_SIZE(ifs->out); i++) {
		if ((ifs->out[i].size == 0) || (ifs->out[i].data[0] == id)) {
			ifs->out[i].data[0] = id;
			memcpy(&ifs->out[i].data[1], buf, size);
			ifs->out[i].size = size + 1;
			break;
		}
	}
	k_spin_unlock(&lock, key);

	if (i == ARRAY_SIZE(ifs->out)) {
		LOG_WRN("No room for output report %" PRIu8, id);
		return -ENOMEM;
	}

	atomic_set_bit(&ifs->out_dirty, i);
	k_work_submit(&out_work);

	return 0;
}

static void send_output(struct service *svc, const struct stored_report *out)
{
	struct stored_report r;
	k_spinlock_key_t key = k_spin_lock(&lock);

	r = *out;
	k_spin_unlock(&lock, key);

	if (r.size == 0) {
		return;
	}

	struct bt_hogp_rep_info *rep = find_rep(&svc->hogp, BT_HIDS_REPORT_TYPE_OUTPUT, r.data[0]);

	if (!rep) {
		return;
	}

	int err = bt_hogp_rep_write_wo_rsp(&svc->hogp, rep, &r.data[1], r.size - 1, NULL);

	if (err) {
		LOG_WRN("Cannot forward output report %" PRIu8 " (err:%d)", r.data[0], err);
	}
}

static void out_work_fn(struct k_work *work)
{
	for (size_t idx = 0; idx < IFACE_COUNT; idx++) {
		struct iface_state *ifs = &ifaces[idx];

		for (size_t i = 0; i < ARRAY_SIZE(ifs->out); i++) {
			if (!atomic_test_and_clear_bit(&ifs->out_dirty, i)) {
				continue;
			}

			for (size_t s = 0; s < ARRAY_SIZE(services); s++) {
				if ((services[s].iface == idx) &&
				    bt_hogp_ready_check(&services[s].hogp)) {
					send_output(&services[s], &ifs->out[i]);
				}
			}
		}
	}
}

/* Input reports **************************************************************/

static void inject_report(struct iface_state *ifs, const uint8_t *data, size_t size)
{
	if (ifs->inject_cnt >= ARRAY_SIZE(ifs->inject)) {
		LOG_WRN("Injected report dropped");
		return;
	}

	struct stored_report *r = &ifs->inject[ifs->inject_cnt++];

	r->size = size;
	memcpy(r->data, data, size);
}

/* The owner of an exclusive interface stopped sending with something down: its
 * link dropped, or another peripheral takes over mid-touch. The host holds the
 * last report until the next one, so it gets one that lets go.
 */
static void lift_owner(size_t idx)
{
	struct iface_state *ifs = &ifaces[idx];

	for (size_t i = 0; i < ARRAY_SIZE(ifs->owner_down); i++) {
		struct stored_report *r = &ifs->owner_down[i];

		if (r->size == 0) {
			continue;
		}

		hid_passthrough_report_lift(idx, r->data, r->size);
		inject_report(ifs, r->data, r->size);
		LOG_INF("Interface %zu: report %" PRIu8 " lifted", idx, r->data[0]);
		r->size = 0;
	}

	ifs->owner_idle = true;
}

/* Whether an input report of an exclusive interface goes to the host. Input
 * comes from one peripheral at a time: the owner keeps it while it has a
 * contact or button down, once its last report was idle (or it went quiet
 * mid-touch), the next one to send input takes over.
 */
static bool take_input(size_t idx, struct service *svc, const struct queued_report *qr)
{
	struct iface_state *ifs = &ifaces[idx];

	if (ifs->owner != svc) {
		if (ifs->owner && !ifs->owner_idle &&
		    ((qr->timestamp - ifs->owner_last_ms) < OWNER_STALE_MS)) {
			return false;
		}

		LOG_INF("Interface %zu: owner service %zu", idx, svc_idx(svc));
		lift_owner(idx);
		ifs->owner = svc;
	}

	bool idle = hid_passthrough_report_idle(idx, qr->data, qr->size);

	ifs->owner_idle = idle;
	ifs->owner_last_ms = qr->timestamp;

	/* Remember the report for a lift, or forget it if it is idle. */
	struct stored_report *slot = NULL;

	for (size_t i = 0; i < ARRAY_SIZE(ifs->owner_down); i++) {
		struct stored_report *r = &ifs->owner_down[i];

		if ((r->size > 0) && (r->data[0] == qr->data[0])) {
			slot = r;
			break;
		}
		if (!slot && (r->size == 0)) {
			slot = r;
		}
	}

	if (slot) {
		if (idle) {
			if ((slot->size > 0) && (slot->data[0] == qr->data[0])) {
				slot->size = 0;
			}
		} else {
			slot->size = qr->size;
			memcpy(slot->data, qr->data, qr->size);
		}
	}

	return true;
}

static void submit_report(struct iface_state *ifs, const uint8_t *data, size_t size)
{
	struct hid_report_event *event = new_hid_report_event(size);

	event->source = &ifaces[0];
	event->subscriber = ifs->subscriber;
	memcpy(event->dyndata.data, data, size);
	APP_EVENT_SUBMIT(event);

	ifs->in_flight++;
}

/* Convert a report to the form the host takes. Returns its size, 0 to drop it. */
static size_t to_host_report(size_t idx, uint8_t *data, size_t size)
{
	const struct hid_passthrough_iface *def = &hid_passthrough_ifaces[idx];

	if (!ifaces[idx].boot) {
		return size;
	}

	/* Boot protocol: only the boot keyboard report, sent without its ID by
	 * the USB state module.
	 */
	if ((def->boot_report_id == 0) || (data[0] != def->boot_report_id) ||
	    (size != (1 + REPORT_SIZE_KEYBOARD_BOOT))) {
		return 0;
	}

	data[0] = REPORT_ID_BOOT_KEYBOARD;
	return size;
}

static void send_iface(size_t idx)
{
	struct iface_state *ifs = &ifaces[idx];

	while (ifs->connected && (ifs->in_flight < ifs->pipeline_size)) {
		uint8_t data[REPORT_SIZE_MAX];
		size_t size;

		if (ifs->inject_cnt > 0) {
			size = ifs->inject[0].size;
			memcpy(data, ifs->inject[0].data, size);
			ifs->inject_cnt--;
			memmove(&ifs->inject[0], &ifs->inject[1],
				ifs->inject_cnt * sizeof(ifs->inject[0]));
		} else {
			struct queued_report qr;

			if (k_msgq_get(&ifs->queue, &qr, K_NO_WAIT)) {
				return;
			}

			struct service *svc = &services[qr.svc_idx];

			if ((qr.gen != svc->gen) || (svc->iface != idx)) {
				/* The service was released since. */
				continue;
			}

			if (hid_passthrough_ifaces[idx].exclusive && !take_input(idx, svc, &qr)) {
				continue;
			}

			size = qr.size;
			memcpy(data, qr.data, size);

			/* A take over may have injected a lift, which goes first. */
			if (ifs->inject_cnt > 0) {
				inject_report(ifs, data, size);
				continue;
			}
		}

		size = to_host_report(idx, data, size);
		if (size > 0) {
			submit_report(ifs, data, size);
		}
	}

	if (!ifs->connected) {
		/* Nobody to send to. */
		ifs->inject_cnt = 0;
		k_msgq_purge(&ifs->queue);
	}
}

static void send_work_fn(struct k_work *work)
{
	for (size_t idx = 0; idx < IFACE_COUNT; idx++) {
		send_iface(idx);
	}
}

static uint8_t hogp_notify_cb(struct bt_hogp *hogp, struct bt_hogp_rep_info *rep, uint8_t err,
			      const uint8_t *data)
{
	if (!data) {
		return BT_GATT_ITER_STOP;
	}

	if (err) {
		return BT_GATT_ITER_CONTINUE;
	}

	struct service *svc = svc_get(hogp);
	size_t size = bt_hogp_rep_size(rep);
	struct queued_report qr;

	if ((svc->iface == NO_IFACE) || ((size + 1) > sizeof(qr.data))) {
		return BT_GATT_ITER_CONTINUE;
	}

	qr.timestamp = k_uptime_get_32();
	qr.svc_idx = svc_idx(svc);
	qr.gen = svc->gen;
	qr.size = size + 1;
	qr.data[0] = bt_hogp_rep_id(rep);
	memcpy(&qr.data[1], data, size);

	if (k_msgq_put(&ifaces[svc->iface].queue, &qr, K_NO_WAIT)) {
		LOG_WRN("Interface %" PRIu8 " queue full, report dropped", svc->iface);
	}

	k_work_submit(&send_work);

	if (suspended) {
		/* Input wakes up the host. */
		APP_EVENT_SUBMIT(new_wake_up_event());
	}

	return BT_GATT_ITER_CONTINUE;
}

/* HID Services ***************************************************************/

static void subscribe_inputs(struct service *svc)
{
	struct bt_hogp_rep_info *rep = NULL;

	while (NULL != (rep = bt_hogp_rep_next(&svc->hogp, rep))) {
		if (bt_hogp_rep_type(rep) != BT_HIDS_REPORT_TYPE_INPUT) {
			continue;
		}

		int err = bt_hogp_rep_subscribe(&svc->hogp, rep, hogp_notify_cb);

		if (err) {
			LOG_ERR("Cannot subscribe to report %" PRIu8 " (err:%d)",
				bt_hogp_rep_id(rep), err);
		} else {
			LOG_INF("Service %zu: subscribed to report %" PRIu8,
				svc_idx(svc), bt_hogp_rep_id(rep));
		}
	}
}

static void service_matched(struct service *svc)
{
	if (!svc->map_match_bm) {
		LOG_WRN("Service %zu: no interface for its report map, ignored", svc_idx(svc));
		return;
	}

	uint8_t idx = __builtin_ctz(svc->map_match_bm);

	LOG_INF("Service %zu -> interface %" PRIu8, svc_idx(svc), idx);

	svc->iface = idx;
	subscribe_inputs(svc);

	/* Replay the host's output reports. */
	for (size_t i = 0; i < ARRAY_SIZE(ifaces[idx].out); i++) {
		send_output(svc, &ifaces[idx].out[i]);
	}

	if (hid_passthrough_ifaces[idx].feature_cnt > 0) {
		svc->refresh_rep = NULL;
		refresh_next(svc);
	} else {
		svc->refreshed = true;
	}
}

static void map_match(struct service *svc, uint8_t byte)
{
	if (svc->map_win_len < sizeof(svc->map_win)) {
		svc->map_win[svc->map_win_len++] = byte;
	} else {
		memmove(&svc->map_win[0], &svc->map_win[1], sizeof(svc->map_win) - 1);
		svc->map_win[sizeof(svc->map_win) - 1] = byte;
	}

	if (svc->map_win_len < sizeof(svc->map_win)) {
		return;
	}

	for (size_t idx = 0; idx < IFACE_COUNT; idx++) {
		const struct hid_passthrough_iface *def = &hid_passthrough_ifaces[idx];
		const uint8_t pattern[TLC_PATTERN_LEN] = {
			0x05, def->usage_page, 0x09, def->usage, 0xa1, 0x01,
		};

		if (!memcmp(svc->map_win, pattern, sizeof(pattern))) {
			svc->map_match_bm |= BIT(idx);
		}
	}
}

static void map_read_next(struct service *svc);

/* Called for every chunk of the report map. The next chunk is read from the
 * callback, until a read returns no data.
 */
static void hogp_map_cb(struct bt_hogp *hogp, uint8_t err, const uint8_t *data, size_t size,
			size_t offset)
{
	struct service *svc = svc_get(hogp);

	if (err == BT_ATT_ERR_INVALID_OFFSET) {
		/* Read past the end: the map was a multiple of the chunk size. */
		data = NULL;
	} else if (err) {
		LOG_ERR("Service %zu: report map read failed (err:%" PRIu8 ")",
			svc_idx(svc), err);
		return;
	}

	if (data && (size > 0)) {
		for (size_t i = 0; i < size; i++) {
			map_match(svc, data[i]);
		}
		svc->map_offset = offset + size;
		map_read_next(svc);
		return;
	}

	LOG_INF("Service %zu: report map read (%zu bytes)", svc_idx(svc), offset);
	service_matched(svc);
}

static void map_read_next(struct service *svc)
{
	int err = bt_hogp_map_read(&svc->hogp, hogp_map_cb, svc->map_offset, K_NO_WAIT);

	if ((err == -EBUSY) || (err == -ENOMEM)) {
		/* Another request of the peer is in progress, or the ATT layer is
		 * out of buffers while the peer's other services are set up.
		 */
		k_work_reschedule(&svc->map_read_retry, K_MSEC(MAP_READ_RETRY_MS));
	} else if (err) {
		LOG_ERR("Service %zu: cannot read report map (err:%d)", svc_idx(svc), err);
	}
}

static void map_read_retry_fn(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct service *svc = CONTAINER_OF(dwork, struct service, map_read_retry);

	if (svc_is_connected(svc)) {
		map_read_next(svc);
	}
}

static void hogp_ready(struct bt_hogp *hogp)
{
	struct service *svc = svc_get(hogp);

	LOG_INF("Service %zu ready", svc_idx(svc));

	svc->map_win_len = 0;
	svc->map_match_bm = 0;
	svc->map_offset = 0;
	map_read_next(svc);
}

static void hogp_prep_error(struct bt_hogp *hogp, int err)
{
	LOG_ERR("Service %zu: preparation failed (err:%d)", svc_idx(svc_get(hogp)), err);
}

static void register_service(struct bt_gatt_dm *dm)
{
	struct bt_conn *conn = bt_gatt_dm_conn_get(dm);
	struct service *svc = NULL;
	size_t peer_services = 0;

	/* Handles can be assigned now as we are in cooperative thread and won't
	 * be preempted.
	 */
	__ASSERT_NO_MSG(!k_is_preempt_thread());

	for (size_t i = 0; i < ARRAY_SIZE(services); i++) {
		if (svc_is_connected(&services[i])) {
			if (bt_hogp_conn(&services[i].hogp) == conn) {
				peer_services++;
			}
		} else if (!svc) {
			svc = &services[i];
		}
	}

	if (!svc || (peer_services >= HIDS_PER_PEER)) {
		LOG_ERR("No room for another HID Service");
		return;
	}

	int err = bt_hogp_handles_assign(dm, &svc->hogp);

	if (err) {
		LOG_ERR("Cannot assign handles (err:%d)", err);
		return;
	}

	svc->iface = NO_IFACE;
	svc->refreshed = false;
	atomic_clear(&svc->feat_dirty);

	LOG_INF("Service %zu registered", svc_idx(svc));
}

/* Let go of everything the service holds down on the host. */
static void release_service(struct service *svc)
{
	uint8_t idx = svc->iface;

	if (idx != NO_IFACE) {
		struct iface_state *ifs = &ifaces[idx];

		if (hid_passthrough_ifaces[idx].exclusive) {
			if (ifs->owner == svc) {
				lift_owner(idx);
				ifs->owner = NULL;
			}
		} else {
			/* Send every input report of the service zeroed. */
			struct bt_hogp_rep_info *rep = NULL;

			while (NULL != (rep = bt_hogp_rep_next(&svc->hogp, rep))) {
				size_t size = bt_hogp_rep_size(rep) + 1;
				uint8_t data[REPORT_SIZE_MAX] = {0};

				/* The size of a report is known once it was
				 * received. One never received has nothing down.
				 */
				if ((bt_hogp_rep_type(rep) != BT_HIDS_REPORT_TYPE_INPUT) ||
				    (size == 1) || (size > sizeof(data))) {
					continue;
				}

				data[0] = bt_hogp_rep_id(rep);
				inject_report(ifs, data, size);
			}
		}

		k_work_submit(&send_work);
	}

	(void)k_work_cancel_delayable(&svc->map_read_retry);
	svc->iface = NO_IFACE;
	svc->gen++;
	atomic_clear(&svc->feat_dirty);
	bt_hogp_release(&svc->hogp);

	LOG_INF("Service %zu released", svc_idx(svc));
}

/* Events *********************************************************************/

static bool handle_hid_report_subscriber_event(const struct hid_report_subscriber_event *event)
{
	int idx = usb_state_hid_subscriber_idx(event->subscriber);

	if ((idx < 0) || (idx >= IFACE_COUNT)) {
		return false;
	}

	struct iface_state *ifs = &ifaces[idx];

	ifs->subscriber = event->subscriber;
	ifs->connected = event->connected;
	ifs->in_flight = 0;
	ifs->boot = false;

	if (event->connected) {
		ifs->pipeline_size = MAX(1, MIN(event->params.pipeline_size,
						event->params.report_max));
	} else {
		k_spinlock_key_t key = k_spin_lock(&lock);

		memset(ifs->out, 0, sizeof(ifs->out));
		k_spin_unlock(&lock, key);
		reset_host_features(idx);
	}

	LOG_INF("Interface %d %sconnected", idx, event->connected ? "" : "dis");

	k_work_submit(&send_work);

	return false;
}

static bool handle_hid_report_subscription_event(
	const struct hid_report_subscription_event *event)
{
	int idx = usb_state_hid_subscriber_idx(event->subscriber);

	if ((idx < 0) || (idx >= IFACE_COUNT) ||
	    (event->report_id != REPORT_ID_BOOT_KEYBOARD)) {
		return false;
	}

	ifaces[idx].boot = event->enabled;
	LOG_INF("Interface %d: boot protocol %sabled", idx, event->enabled ? "en" : "dis");

	return false;
}

static bool handle_hid_report_sent_event(const struct hid_report_sent_event *event)
{
	int idx = usb_state_hid_subscriber_idx(event->subscriber);

	if ((idx < 0) || (idx >= IFACE_COUNT)) {
		return false;
	}

	struct iface_state *ifs = &ifaces[idx];

	if (ifs->in_flight > 0) {
		ifs->in_flight--;
	}

	send_iface(idx);

	return false;
}

static void init(void)
{
	static const struct bt_hogp_init_params params = {
		.ready_cb = hogp_ready,
		.prep_error_cb = hogp_prep_error,
	};

	for (size_t i = 0; i < ARRAY_SIZE(services); i++) {
		bt_hogp_init(&services[i].hogp, &params);
		services[i].iface = NO_IFACE;
		k_work_init_delayable(&services[i].map_read_retry, map_read_retry_fn);
	}

	for (size_t i = 0; i < ARRAY_SIZE(ifaces); i++) {
		k_msgq_init(&ifaces[i].queue, ifaces[i].queue_buf, sizeof(struct queued_report),
			    QUEUE_SIZE);
		ifaces[i].owner_idle = true;
	}

	k_work_init(&send_work, send_work_fn);
	k_work_init(&feat_work, feat_work_fn);
	k_work_init(&out_work, out_work_fn);
}

static bool app_event_handler(const struct app_event_header *aeh)
{
	if (is_hid_report_sent_event(aeh)) {
		return handle_hid_report_sent_event(cast_hid_report_sent_event(aeh));
	}

	if (is_module_state_event(aeh)) {
		const struct module_state_event *event = cast_module_state_event(aeh);

		if (check_state(event, MODULE_ID(main), MODULE_STATE_READY)) {
			init();
			module_set_state(MODULE_STATE_READY);
		}

		return false;
	}

	if (is_ble_discovery_hids_event(aeh)) {
		register_service(cast_ble_discovery_hids_event(aeh)->dm);

		return false;
	}

	if (is_ble_discovery_complete_event(aeh)) {
		register_service(cast_ble_discovery_complete_event(aeh)->dm);

		return false;
	}

	if (is_hid_report_subscriber_event(aeh)) {
		return handle_hid_report_subscriber_event(cast_hid_report_subscriber_event(aeh));
	}

	if (is_hid_report_subscription_event(aeh)) {
		return handle_hid_report_subscription_event(
			cast_hid_report_subscription_event(aeh));
	}

	if (is_ble_peer_event(aeh)) {
		const struct ble_peer_event *event = cast_ble_peer_event(aeh);

		if (event->state == PEER_STATE_DISCONNECTED) {
			for (size_t i = 0; i < ARRAY_SIZE(services); i++) {
				struct service *svc = &services[i];

				if (svc_is_connected(svc) &&
				    (bt_hogp_conn(&svc->hogp) == event->id)) {
					release_service(svc);
				}
			}
		}

		return false;
	}

	if (is_wake_up_event(aeh)) {
		suspended = false;

		return false;
	}

	if (is_power_down_event(aeh)) {
		suspended = true;

		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE_EARLY(MODULE, ble_discovery_hids_event);
APP_EVENT_SUBSCRIBE_EARLY(MODULE, ble_discovery_complete_event);
APP_EVENT_SUBSCRIBE(MODULE, ble_peer_event);
APP_EVENT_SUBSCRIBE(MODULE, hid_report_subscriber_event);
APP_EVENT_SUBSCRIBE(MODULE, hid_report_subscription_event);
APP_EVENT_SUBSCRIBE(MODULE, hid_report_sent_event);
APP_EVENT_SUBSCRIBE(MODULE, power_down_event);
APP_EVENT_SUBSCRIBE(MODULE, wake_up_event);
