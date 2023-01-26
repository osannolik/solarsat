#ifndef NRF52_PCA10040_S132_SENSOR_H
#define NRF52_PCA10040_S132_SENSOR_H

#include "nrf_drv_twi.h"
#include "bme68x.h"

typedef struct {
  nrf_drv_twi_t const *twi;
  uint8_t              twi_buffer[100];
  volatile bool        twi_xfer_done;
  struct bme68x_dev    bme;
  struct bme68x_conf   conf;
  struct bme68x_data   data;
} sensor_t;

typedef struct {
  /*! Temperature in degree celsius */
  float temperature;
  /*! Pressure in Pascal */
  float pressure;
  /*! Humidity in % relative humidity x1000 */
  float humidity;
} sensor_measurement_t;

void sensor_init(sensor_t *sensor, const uint32_t scl_pin, const uint32_t sda_pin);

void sensor_config(sensor_t *sensor);

uint32_t start_measurements(sensor_t *sensor);

sensor_measurement_t collect_measurements(sensor_t *sensor);

#endif //NRF52_PCA10040_S132_SENSOR_H
