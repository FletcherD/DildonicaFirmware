#ifndef BLUETOOTH_CPP_COMPAT_HPP
#define BLUETOOTH_CPP_COMPAT_HPP

#include <zephyr/bluetooth/bluetooth.h>=

#ifdef __cplusplus
extern "C" {
#endif

// Wrapper function to create bt_le_adv_param
inline const struct bt_le_adv_param* create_bt_le_adv_param(uint32_t options,
                                                            uint32_t interval_min,
                                                            uint32_t interval_max,
                                                            const bt_addr_le_t *peer)
{
    static struct bt_le_adv_param param;
    param = {
        .id = BT_ID_DEFAULT,
        .sid = 0,
        .secondary_max_skip = 0,
        .options = options,
        .interval_min = interval_min,
        .interval_max = interval_max,
        .peer = peer
    };
    return &param;
}

// Redefine the problematic macro
#undef BT_LE_ADV_PARAM
#define BT_LE_ADV_PARAM(_options, _int_min, _int_max, _peer) \
    create_bt_le_adv_param((_options), (_int_min), (_int_max), (_peer))

#ifdef __cplusplus
}
#endif

#endif // BLUETOOTH_CPP_COMPAT_HPP