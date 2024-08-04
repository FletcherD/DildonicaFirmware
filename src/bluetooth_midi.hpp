#ifndef BLUETOOTH_MIDI_HPP
#define BLUETOOTH_MIDI_HPP

#include <zephyr/bluetooth/gatt.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const struct bt_gatt_service_static midi_svc;

#ifdef __cplusplus
}
#endif

#endif // BLUETOOTH_MIDI_HPP