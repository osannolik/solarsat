#include <stdbool.h>

#include "nrf_delay.h"
#include "sensor.h"

#define BME68X_I2C_ADDR          (0x76u)
#define SENSOR_AMBIENT_TEMP_DEGC (23)

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);

void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context) {
  sensor_t *sensor = (sensor_t *) p_context;

  switch (p_event->type) {
    case NRF_DRV_TWI_EVT_DONE:
      sensor->twi_xfer_done = true;
      break;
    default:
      break;
  }
}

static void block_while_twi_busy(nrf_drv_twi_t const *twi) {
  while (nrf_drv_twi_is_busy(twi)) { __WFE(); }
}

static int8_t read_i2c(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
  sensor_t *sensor = (sensor_t *) intf_ptr;

  sensor->twi_xfer_done = false;
  ret_code_t err_code = nrf_drv_twi_tx(sensor->twi, BME68X_I2C_ADDR, &reg_addr, 1, true);
  APP_ERROR_CHECK(err_code);
  while (sensor->twi_xfer_done == false) { __WFE(); }

  err_code = nrf_drv_twi_rx(sensor->twi, BME68X_I2C_ADDR, reg_data, length);
  APP_ERROR_CHECK(err_code);
  while (sensor->twi_xfer_done == false) { __WFE(); }

  block_while_twi_busy(sensor->twi);

  return (int8_t) err_code;
}

static int8_t write_i2c(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
  sensor_t *sensor = (sensor_t *) intf_ptr;

  sensor->twi_buffer[0] = reg_addr;
  for (uint32_t i = 0; i < length; i++) {
    sensor->twi_buffer[1 + i] = reg_data[i];
  }

  sensor->twi_xfer_done = false;
  ret_code_t err_code = nrf_drv_twi_tx(sensor->twi, BME68X_I2C_ADDR, sensor->twi_buffer, length + 1, false);
  APP_ERROR_CHECK(err_code);
  while (sensor->twi_xfer_done == false) { __WFE(); }

  block_while_twi_busy(sensor->twi);

  return (int8_t) err_code;
}

static void delay_us(uint32_t period, void *intf_ptr) {
  (void) intf_ptr;
  nrf_delay_us(period);
}

void sensor_init(sensor_t *sensor, const uint32_t scl_pin, const uint32_t sda_pin) {
  ret_code_t err_code;

  sensor->twi = &m_twi;

  const nrf_drv_twi_config_t twi_bme680_config = {
      .scl                = scl_pin,
      .sda                = sda_pin,
      .frequency          = NRF_DRV_TWI_FREQ_400K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
      .clear_bus_init     = false
  };

  err_code = nrf_drv_twi_init(sensor->twi, &twi_bme680_config, twi_handler, (void*) sensor);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(sensor->twi);

  sensor->bme.read = read_i2c;
  sensor->bme.write = write_i2c;
  sensor->bme.intf = BME68X_I2C_INTF;
  sensor->bme.delay_us = delay_us;
  sensor->bme.intf_ptr = (void*) sensor;
  sensor->bme.amb_temp = SENSOR_AMBIENT_TEMP_DEGC;

  err_code = bme68x_init(&sensor->bme);
  APP_ERROR_CHECK(err_code);
}

void sensor_config(sensor_t *sensor) {
  ret_code_t err_code;

  sensor->conf.filter = BME68X_FILTER_OFF;
  sensor->conf.odr = BME68X_ODR_NONE;
  sensor->conf.os_hum = BME68X_OS_16X;
  sensor->conf.os_pres = BME68X_OS_1X;
  sensor->conf.os_temp = BME68X_OS_2X;
  err_code = bme68x_set_conf(&sensor->conf, &sensor->bme);
  APP_ERROR_CHECK(err_code);

  struct bme68x_heatr_conf heatr_conf;
  heatr_conf.enable = BME68X_DISABLE;
  heatr_conf.heatr_temp = 0;
  heatr_conf.heatr_dur = 0;
  err_code = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &sensor->bme);
  APP_ERROR_CHECK(err_code);
}

uint32_t start_measurements(sensor_t *sensor) {
  int8_t err_code;

  err_code = bme68x_set_op_mode(BME68X_FORCED_MODE, &sensor->bme);
  APP_ERROR_CHECK(err_code);

  uint32_t delay_period_us;
  delay_period_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &sensor->conf, &sensor->bme);
  return delay_period_us;
}

sensor_measurement_t collect_measurements(sensor_t *sensor) {
  uint8_t n_fields;
  int8_t err_code;

  err_code = bme68x_get_data(BME68X_FORCED_MODE, &sensor->data, &n_fields, &sensor->bme);
  APP_ERROR_CHECK(err_code);

  const sensor_measurement_t measurement = {
      .temperature = sensor->data.temperature,
      .pressure = sensor->data.pressure,
      .humidity = sensor->data.humidity};
  return measurement;
}