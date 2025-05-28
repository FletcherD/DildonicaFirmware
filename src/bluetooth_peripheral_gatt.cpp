/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

extern "C" {
    #include <zephyr/bluetooth/bluetooth.h>
    #include <zephyr/bluetooth/conn.h>
    #include <zephyr/bluetooth/gatt.h>
}

#include "bluetooth_cpp_compat.hpp"

#include "bluetooth_dildonica.hpp"

// from gatt_write_common.c /////////////////////////////////

static struct bt_gatt_exchange_params mtu_exchange_params;
struct bt_conn *conn_connected;
uint32_t last_write_rate;
void (*start_scan_func)();

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	printk("%s: MTU exchange %s (%u)\n", __func__,
	       err == 0U ? "successful" : "failed",
	       bt_gatt_get_mtu(conn));
}

static int mtu_exchange(struct bt_conn *conn)
{
	int err;

	printk("%s: Current MTU = %u\n", __func__, bt_gatt_get_mtu(conn));

	mtu_exchange_params.func = mtu_exchange_cb;

	printk("%s: Exchange MTU...\n", __func__);
	err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
	if (err) {
		printk("%s: MTU exchange failed (err %d)", __func__, err);
	}

	return err;
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	struct bt_conn_info conn_info{};
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("%s: Failed to connect to %s (%u)\n", __func__, addr,
		       conn_err);
		return;
	}

	err = bt_conn_get_info(conn, &conn_info);
	if (err) {
		printk("Failed to get connection info (%d).\n", err);
		return;
	}

	printk("%s: %s role %u\n", __func__, addr, conn_info.role);

	conn_connected = bt_conn_ref(conn);

	(void)mtu_exchange(conn);

#if defined(CONFIG_BT_SMP)
	if (conn_info.role == BT_CONN_ROLE_CENTRAL) {
		err = bt_conn_set_security(conn, BT_SECURITY_L2);
		if (err) {
			printk("Failed to set security (%d).\n", err);
		}
	}
#endif
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn_info conn_info{};
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	err = bt_conn_get_info(conn, &conn_info);
	if (err) {
		printk("Failed to get connection info (%d).\n", err);
		return;
	}

	printk("%s: %s role %u (reason %u)\n", __func__, addr, conn_info.role,
	       reason);

	conn_connected = nullptr;

	bt_conn_unref(conn);

	if (conn_info.role == BT_CONN_ROLE_CENTRAL) {
		start_scan_func();
	}
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	printk("%s: int (0x%04x, 0x%04x) lat %u to %u\n", __func__,
	       param->interval_min, param->interval_max, param->latency,
	       param->timeout);

	return true;
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
			     uint16_t latency, uint16_t timeout)
{
	printk("%s: int 0x%04x lat %u to %u\n", __func__, interval,
	       latency, timeout);
}

#if defined(CONFIG_BT_SMP)
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	printk("%s: to level %u (err %u)\n", __func__, level, err);
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated,
#if defined(CONFIG_BT_SMP)
	.security_changed = security_changed,
#endif
};

// end from gatt_write_common.c /////////////////////////////////

bool notif_enabled = false;

DildonicaBluetoothMessage lastDildonicaSample;

extern const struct bt_gatt_service_static midi_svc;

#define BT_UUID_MIDI_SERVICE BT_UUID_128_ENCODE(0x64696C64, 0x0000, 0x1000, 0x8000, 0x0000CAFEBABE)
struct bt_uuid_128 midi_service_uuid = BT_UUID_INIT_128(BT_UUID_MIDI_SERVICE);

#define BT_UUID_MIDI_CHARACTERISTIC BT_UUID_128_ENCODE(0x6F6E6963, 0x0000, 0x1000, 0x8000, 0x0000CAFEBABE)
struct bt_uuid_128 midi_characteristic_uuid = BT_UUID_INIT_128(BT_UUID_MIDI_CHARACTERISTIC);

void midi_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("MIDI notifications %s\n", notif_enabled ? "enabled" : "disabled");
}

ssize_t read_midi(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         void *buf, uint16_t len, uint16_t offset)
{
    printk("MIDI read request\n");
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &lastDildonicaSample, sizeof(lastDildonicaSample));
}

ssize_t write_midi(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    printk("MIDI write request (ignored)\n");
    return 1; // Always return 1 to acknowledge the write
}

BT_GATT_SERVICE_DEFINE(midi_svc,
    BT_GATT_PRIMARY_SERVICE(&midi_service_uuid),
    BT_GATT_CHARACTERISTIC(&midi_characteristic_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           read_midi, write_midi, nullptr),
	BT_GATT_CCC(midi_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_MIDI_SERVICE),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

#if defined(CONFIG_BT_SMP)
static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_callbacks = {
	.cancel = auth_cancel,
};
#endif /* CONFIG_BT_SMP */

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated
};


// TODO: Fix MTU exchange!

void setup_bluetooth_peripheral()
{
	int err;

	err = bt_enable(nullptr);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	bt_gatt_cb_register(&gatt_callbacks);

#if defined(CONFIG_BT_SMP)
	(void)bt_conn_auth_cb_register(&auth_callbacks);
#endif /* CONFIG_BT_SMP */

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");

	conn_connected = nullptr;
}
