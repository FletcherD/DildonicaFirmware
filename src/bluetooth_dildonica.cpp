#include <zephyr/kernel.h>

#include "circularbuffer.hpp"

#include "bluetooth_dildonica.hpp"

extern "C" {
    #include <zephyr/bluetooth/bluetooth.h>
    #include <zephyr/bluetooth/conn.h>
    #include <zephyr/bluetooth/gatt.h>
}

extern bool notif_enabled;
extern struct bt_conn *conn_connected;

extern const struct bt_gatt_service_static midi_svc;

void send_bluetooth_sample(DildonicaBluetoothMessage btMessage) {

    struct bt_conn *conn = nullptr;

    if (conn_connected && notif_enabled) {
        /* Get a connection reference to ensure that a
            * reference is maintained in case disconnected
            * callback is called while we perform GATT Write
            * command.
            */
        conn = bt_conn_ref(conn_connected);
    }

    if (conn) {

        //Send MIDI message as a notification
        int err = bt_gatt_notify(conn, &midi_svc.attrs[1], &btMessage, sizeof(btMessage));
        if (err) {
            printk("Failed to send MIDI notification (err %d)\n", err);
        }

        bt_conn_unref(conn);			
    }
}
