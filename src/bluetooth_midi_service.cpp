#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>

extern "C" {
    #include <zephyr/bluetooth/uuid.h>
}

#define BT_UUID_MIDI_SERVICE BT_UUID_128_ENCODE(0x03B80E5A, 0xEDE8, 0x4B33, 0xA751, 0x6CE34EC4C700)
static const struct bt_uuid_128 midi_service_uuid = BT_UUID_INIT_128(BT_UUID_MIDI_SERVICE);

#define BT_UUID_MIDI_CHARACTERISTIC BT_UUID_128_ENCODE(0x7772E5DB, 0x3868, 0x4112, 0xA1A9, 0xF2669D106BF3)
static const struct bt_uuid_128 midi_characteristic_uuid = BT_UUID_INIT_128(BT_UUID_MIDI_CHARACTERISTIC);

extern "C" {
    void midi_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);
    ssize_t read_midi(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                      void *buf, uint16_t len, uint16_t offset);
}

BT_GATT_SERVICE_DEFINE(midi_svc,
    BT_GATT_PRIMARY_SERVICE(&midi_service_uuid),
    BT_GATT_CHARACTERISTIC(&midi_characteristic_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           read_midi, nullptr, nullptr),
    BT_GATT_CCC(midi_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);