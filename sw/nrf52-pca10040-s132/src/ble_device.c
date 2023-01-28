#include "ble_device.h"

#include "app_error.h"

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"

#define APP_BLE_CONN_CFG_TAG 1 /**< A tag identifying the SoftDevice BLE configuration. */
#define ADV_MANU_DATA_SIZE_MAX 8

/**< Parameters to be passed to the stack when starting advertising. */
static ble_gap_adv_params_t m_adv_params;

/**< Advertising handle used to identify an advertising set. */
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;

/**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];

static uint8_t m_manu_data[ADV_MANU_DATA_SIZE_MAX] = { 0 };

static ble_gap_adv_data_t m_adv_data = {
    .adv_data = {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data = {
        .p_data = NULL,
        .len    = 0
    }
};

static void advertising_init(const uint8_t *p_manu_data, const uint8_t len) {
  uint32_t      err_code;
  ble_advdata_t advdata;

  ble_advdata_manuf_data_t manuf_data;
  manuf_data.company_identifier = MANUFACTURER_COMPANY_ID_TEST;
  manuf_data.data.p_data        = (uint8_t*) p_manu_data;
  manuf_data.data.size          = len;

  memset(&advdata, 0, sizeof(advdata));

  advdata.name_type             = BLE_ADVDATA_FULL_NAME;
  advdata.include_appearance    = true;
  advdata.flags                 = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
  advdata.p_manuf_specific_data = &manuf_data;

  memset(&m_adv_params, 0, sizeof(m_adv_params));

  m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
  m_adv_params.p_peer_addr     = NULL;
  m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
  m_adv_params.interval        = MSEC_TO_UNITS(BLE_ADVERTISE_INTERVAL_MS, UNIT_0_625_MS);
  m_adv_params.duration        = BLE_TIMEOUT_NEVER;

  err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
  APP_ERROR_CHECK(err_code);

#if 0
  uint8_t tmp[15+5] = {0x07,0x09,0x4E,0x6F,0x72,0x64,0x69,0x63,0x02,0x01,0x04,0x03,0x03,0x0F,0x18,
                       0x04,0xFF,0xFF,0xFF,0x69};
    uint8_t i;
    for (i=0; i<(15+5); i++) {
        m_adv_data.adv_data.p_data[i] = tmp[i];
    }
    m_adv_data.adv_data.len = 15+5;
#endif

  err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
  APP_ERROR_CHECK(err_code);
}

static void advertising_start(void) {
  ret_code_t err_code;

  err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
  APP_ERROR_CHECK(err_code);
}

void ble_device_set_advertising_data(const uint8_t* data, const uint8_t len) {
  const uint8_t used_len = MIN(len, ADV_MANU_DATA_SIZE_MAX);
  memcpy(m_manu_data, data, used_len);

  ret_code_t err_code;

  err_code = sd_ble_gap_adv_stop(m_adv_handle);
  APP_ERROR_CHECK(err_code);

  advertising_init(m_manu_data, used_len);

  advertising_start();
}

static void ble_stack_init(void) {
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);
}

static void gap_params_init(void) {
  ret_code_t              err_code;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
                                        (const uint8_t *) DEVICE_NAME,
                                        strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE);
  APP_ERROR_CHECK(err_code);
}

#if 0
/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}
#endif

void ble_device_init() {
  //power_management_init();
  ble_stack_init();
  gap_params_init();

  advertising_init(m_manu_data, 4);

  advertising_start();
}