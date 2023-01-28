#ifndef NRF52_PCA10040_S132_BLE_DEVICE_H
#define NRF52_PCA10040_S132_BLE_DEVICE_H

#include <stdint.h>

#define DEVICE_NAME "sat"
#define MANUFACTURER_COMPANY_ID_TEST 0xFFFF
#define BLE_APPEARANCE BLE_APPEARANCE_GENERIC_THERMOMETER
#define BLE_ADVERTISE_INTERVAL_MS (1000)
#define BLE_TIMEOUT_NEVER 0

void ble_device_init();

void ble_device_set_advertising_data(const uint8_t* data, uint8_t len);

#endif //NRF52_PCA10040_S132_BLE_DEVICE_H