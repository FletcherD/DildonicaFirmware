/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

extern int mtu_exchange(struct bt_conn *conn);
extern int write_cmd(struct bt_conn *conn);
extern struct bt_conn *conn_connected;
extern uint32_t last_write_rate;

static uint8_t midi_data[5]; // Buffer for MIDI messages

#define BT_UUID_MIDI_SERVICE BT_UUID_128_ENCODE(0x03B80E5A, 0xEDE8, 0x4B33, 0xA751, 0x6CE34EC4C700)
static struct bt_uuid_128 midi_service_uuid = BT_UUID_INIT_128(BT_UUID_MIDI_SERVICE);

#define BT_UUID_MIDI_CHARACTERISTIC BT_UUID_128_ENCODE(0x7772E5DB, 0x3868, 0x4112, 0xA1A9, 0xF2669D106BF3)
static struct bt_uuid_128 midi_characteristic_uuid = BT_UUID_INIT_128(BT_UUID_MIDI_CHARACTERISTIC);

static void midi_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("MIDI notifications %s\n", notif_enabled ? "enabled" : "disabled");
}

static ssize_t read_midi(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         void *buf, uint16_t len, uint16_t offset)
{
    printk("MIDI read request\n");
    return bt_gatt_attr_read(conn, attr, buf, len, offset, midi_data, sizeof(midi_data));
}

BT_GATT_SERVICE_DEFINE(midi_svc,
    BT_GATT_PRIMARY_SERVICE(&midi_service_uuid),
    BT_GATT_CHARACTERISTIC(&midi_characteristic_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           read_midi, NULL, midi_data),
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

void send_midi_control_change(uint16_t timestamp, uint8_t channel, uint8_t controller, uint8_t value)
{
	struct bt_conn *conn = NULL;

	if (conn_connected) {
		/* Get a connection reference to ensure that a
			* reference is maintained in case disconnected
			* callback is called while we perform GATT Write
			* command.
			*/
		conn = bt_conn_ref(conn_connected);
	} else {
        printk("Not connected\n");
        return;
    }

	if (conn) {
		// Construct MIDI message
		midi_data[0] = 0x80 | ((timestamp >> 7) & 0b0111111);
		midi_data[1] = 0x80 | ((timestamp) 		& 0b1111111);
		midi_data[2] = 0xB0 | (channel & 0x0F); // Control Change status byte
		midi_data[3] = controller & 0x7F; // Controller number
		midi_data[4] = value & 0x7F; // Controller value

		// Send MIDI message as a notification
		// int err = bt_gatt_write_without_response(conn_connected, midi_svc.attrs[1].handle, midi_data, sizeof(midi_data), false);
		int err = bt_gatt_notify(conn_connected, &midi_svc.attrs[1], midi_data, sizeof(midi_data));
		if (err) {
			printk("Failed to send MIDI notification (err %d)\n", err);
		} else {
			printk("Sent MIDI Control Change Notification: Channel %d, Controller %d, Value %d\n", channel, controller, value);
		}

		bt_conn_unref(conn);
	} else {
        printk("Not connected\n");
        return;
    }
}

void setup_bluetooth_peripheral()
{
	int err;

	err = bt_enable(NULL);
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

	conn_connected = NULL;
}