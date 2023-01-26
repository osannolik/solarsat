#include "ble_device.h"

#include "app_timer.h"

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"

#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(1000, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

APP_TIMER_DEF(m_timer);

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
    {
        .adv_data =
            {
                .p_data = m_enc_advdata,
                .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
            },
        .scan_rsp_data =
            {
                .p_data = NULL,
                .len    = 0

            }
    };

#define MDATA_LEN 2
static uint8_t mdata[MDATA_LEN] = {0x01, 0x02};

#define MANUFACTURER_COMPANY_ID_TEST 0xFFFF

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(uint8_t *p_manu_data, const uint8_t len)
{
  uint32_t      err_code;
  ble_advdata_t advdata;

  ble_advdata_manuf_data_t manuf_data;
  manuf_data.company_identifier = MANUFACTURER_COMPANY_ID_TEST;
  manuf_data.data.p_data        = p_manu_data;
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
  m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
  m_adv_params.duration        = 0; // Never time out.

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

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
  ret_code_t err_code;

  err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
  APP_ERROR_CHECK(err_code);

//  err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);
}

#if 0
/**@brief Function for initializing logging. */
static void log_init(void)
{
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}
#endif

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

static void notification_timeout_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);

  mdata[0]++;

  ret_code_t err_code;

  err_code = sd_ble_gap_adv_stop(m_adv_handle);
  APP_ERROR_CHECK(err_code);

  advertising_init(mdata, MDATA_LEN);

  err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
#define D_NAME "sat"
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

static void gap_params_init(void)
{

  ret_code_t              err_code;
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
                                        (const uint8_t *)D_NAME,
                                        strlen(D_NAME));
  APP_ERROR_CHECK(err_code);

  err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_THERMOMETER);
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

#define NOTIFICATION_INTERVAL           APP_TIMER_TICKS(2000)

void ble_device_init() {

  //log_init();
  //leds_init();
  //power_management_init();
  ble_stack_init();
  gap_params_init();
  advertising_init(mdata, MDATA_LEN);

  advertising_start();

  ret_code_t err_code;

  err_code = app_timer_create(&m_timer, APP_TIMER_MODE_REPEATED, notification_timeout_handler);
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_start(m_timer, NOTIFICATION_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);

}