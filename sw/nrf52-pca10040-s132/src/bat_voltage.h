#ifndef NRF52_PCA10040_S132_BAT_VOLTAGE_H
#define NRF52_PCA10040_S132_BAT_VOLTAGE_H

#include "nrf_drv_saadc.h"

typedef struct { } bat_voltage_t;

void bat_voltage_init(bat_voltage_t *bat, const uint32_t adc_pin);

float bat_voltage_value(const bat_voltage_t *bat);

#endif //NRF52_PCA10040_S132_BAT_VOLTAGE_H
