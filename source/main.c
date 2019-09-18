/**
 * This software is subject to the ANT+ Shared Source License
 * www.thisisant.com/swlicenses
 * Copyright (c) Dynastream Innovations, Inc. 2012
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * 1) Redistributions of source code must retain the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2) Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 * 
 * 3) Neither the name of Dynastream nor the names of its
 *    contributors may be used to endorse or promote products
 *    derived from this software without specific prior
 *    written permission.
 * 
 * The following actions are prohibited:
 * 1) Redistribution of source code containing the ANT+ Network
 *    Key. The ANT+ Network Key is available to ANT+ Adopters.
 *    Please refer to http://thisisant.com to become an ANT+
 *    Adopter and access the key.
 * 
 * 2) Reverse engineering, decompilation, and/or disassembly of
 *    software provided in binary form under this license.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE HEREBY
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; DAMAGE TO ANY DEVICE, LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE. SOME STATES DO NOT ALLOW
 * THE EXCLUSION OF INCIDENTAL OR CONSEQUENTIAL DAMAGES, SO THE
 * ABOVE LIMITATIONS MAY NOT APPLY TO YOU.
 * 
 */

/**@file
 * @defgroup ant_bpwr_sensor_main ANT Bicycle Power sensor example
 * @{
 * @ingroup nrf_ant_bicycle_power
 *
 * @brief Example of ANT Bicycle Power profile display.
 *
 * Before compiling this example for NRF52, complete the following steps:
 * - Download the S212 SoftDevice from <a href="https://www.thisisant.com/developer/components/nrf52832" target="_blank">thisisant.com</a>.
 * - Extract the downloaded zip file and copy the S212 SoftDevice headers to <tt>\<InstallFolder\>/components/softdevice/s212/headers</tt>.
 * If you are using Keil packs, copy the files into a @c headers folder in your example folder.
 * - Make sure that @ref ANT_LICENSE_KEY in @c nrf_sdm.h is uncommented.
 */

//#define ANT_LICENSE_KEY "3831-521d-7df9-24d8-eff3-467b-225f-a00e" // This is an EVALUATION license key - DO NOT USE FOR COMMERCIAL PURPOSES


#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_soc.h"
#include "bsp.h"
#include "hardfault.h"
#include "app_error.h"
#include "nordic_common.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "app_timer.h"
#include "math.h"
#include "nrf_section.h"
#include "nrf_fstorage_sd.h"
#include "fds.h"
#include "nrf_drv_gpiote.h"
#include "app_gpiote.h"
#include "nrf_pwr_mgmt.h"
#include "ant_boot_settings_api.h"

#include "ad7124_cmd.h"
#include "bmi160.h"
#include "bmi160_cmd.h"
#include "peripheral_if.h"



#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"


#define PARM_FILE_ID    0x5329
#define PARM_REC_KEY    0x9235




//#define NRF_LOG_MODULE_NAME APP17003
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_sdh.h"
   
#include "nrf_ble_gatt.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "app_adc.h"   
#include "sysparams.h"

//#include "app_ble_uart.h"
#include "app_ble_nus_peripheral.h"
#include "app_ant_bpwr.h"
#include "nrf52_ble_dfu.h"
   
#include "sensorData.h"
#include "ble_sensor_node.h"

#include "packetvalid.h"


#define MODIFICATION_TYPE_BUTTON 0 /* predefined value, MUST REMAIN UNCHANGED */
#define MODIFICATION_TYPE_AUTO   1 /* predefined value, MUST REMAIN UNCHANGED */

#if (MODIFICATION_TYPE != MODIFICATION_TYPE_BUTTON) \
    && (MODIFICATION_TYPE != MODIFICATION_TYPE_AUTO)
    #error Unsupported value of MODIFICATION_TYPE.
#endif

#ifndef SENSOR_TYPE // can be provided as preprocesor global symbol
    #define SENSOR_TYPE (TORQUE_NONE)
#endif

#define APP_TIMER_PRESCALER         0 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE     0x04 /**< Size of timer operation queues. */
//#define CALIBRATION_DATA            0x55AAu /**< General calibration data value. */

#define POLL_INTERVAL   1000      // ms
#define POLL_TICKS      (POLL_INTERVAL*2048)/1000
// ms to tick, measure interval of 100 ms
#define MEASURE_INTERVAL		APP_TIMER_TICKS(POLL_INTERVAL)  /**< measurement interval (ticks). */
#define REPORT_INTERVAL APP_TIMER_TICKS(100)

sysParam_t sysParam;
      

static uint8_t testRPM = 0;
static uint8_t testTorque = 0;


static uint16_t timeout_secs = 0;


app_gpiote_user_id_t m_app_gpiote_my_id;
app_gpiote_user_id_t  id_ad7124;
app_gpiote_user_id_t  id_adxl335;
uint32_t adc_buffer[2];
static uint8_t bmi_buffer[12];
void adc_drdy(void);
void bmi_drdy(void);

//BLE_NUS_DEF(m_nus);                                                                 /**< BLE NUS service instance. */
//BLE_SENSORNODE_DEF(m_snode);
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);
static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static ble_uuid_t m_adv_uuids[]= /**< Universally unique service identifier. */
{
  {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE},
//  {BLE_UUID_SENSORNODE_SERVICE, BLE_SENSORNODE_SERVICE_UUID_TYPE},
//  {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
};



/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

//ant_bpwr_profile_t m_ant_bpwr;
/** @snippet [ANT BPWR TX Instance] */

//ant_bpwr_simulator_t m_ant_bpwr_simulator;    /**< Simulator used to simulate profile data. */

#define REPORT_INTERVAL APP_TIMER_TICKS(100)

APP_TIMER_DEF(m_periodic_timer_id);
APP_TIMER_DEF(m_inertial_timer_id);
APP_TIMER_DEF(m_report_timer_id);
APP_TIMER_DEF(m_flash_timer_id);

static void advertising_start(bool erase_bonds); 
static void advertising_reconfig();
bool m_is_ready = false;
bool m_sysoff_started;
bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
  uint32_t err_code;

  NRF_LOG_INFO("APP_SHUTDOWN Handler\n");

  switch (event)
  {
    case NRF_PWR_MGMT_EVT_PREPARE_SYSOFF:
      NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_SYSOFF\n");
      sd_ble_gap_adv_stop();
      //ad7124cmd_goStop();
      ad7124cmd_goSleep();
      bmi160_cmd_stop();      
      bmi160_cmd_start_anymotion();
      //app_adc_deinit(); // not help
      break;

    case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
      NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_WAKEUP\n");
      UNUSED_VARIABLE(err_code);
      sd_ble_gap_adv_stop();
      //ad7124cmd_goStop();
      ad7124cmd_goSleep();
      bmi160_cmd_stop();      
      bmi160_cmd_start_anymotion();
      //app_adc_deinit(); // not help
      break;

    case NRF_PWR_MGMT_EVT_PREPARE_DFU:
      NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_WAKEUP\n");
      NRF_LOG_ERROR("Entering DFU is not supported by this example.\r\n");
      APP_ERROR_HANDLER(NRF_ERROR_API_NOT_IMPLEMENTED);
      break;
  }
  err_code = app_timer_stop_all();
  APP_ERROR_CHECK(err_code);

  return true;
}
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler,0);

/** @snippet [ANT BPWR simulator call] */

static void flash_timeout_handler(void *p_context)
{
  bsp_board_led_off(0);
}  
static void periodic_timeout_handler(void *p_context)
{
  appParam.ranSeconds++;
  appParam.rpm_idle_seconds++;
  appParam.connection_idle_seconds++;

  float r = appParam.imu.radSum*60/(appParam.imu.time)/6.28;
  //m_rpm.f(&m_rpm.filter,r);
  appParam.rpmHis.his[appParam.rpmHis.index++] = r;
  if(appParam.rpmHis.index == appParam.rpmHis.nofElement)
    appParam.rpmHis.index = 0;
  
  r = 0;
  for(uint8_t i=0;i<appParam.rpmHis.nofElement;i++)
    r += appParam.rpmHis.his[i];
  r /= appParam.rpmHis.nofElement;
  //appParam.rpm = m_rpm.filter.y[0]>10?m_rpm.filter.y[0]:0;
  if(r > 10){
    appParam.rpm = r;
    appParam.period_ms = 60./appParam.rpm *1000;
    appParam.period = lround(appParam.period_ms*2.048);
    appParam.rpm_idle_seconds = 0;
    
    float f = 0.;
    for(uint8_t i=0;i<appParam.tHis.nofElement;i++)
      f += appParam.tHis.his[i];
    f /= appParam.tHis.nofElement;
    appParam.power = (uint16_t)lround(f*appParam.rpm*1000/9549.*2.);
    appParam.torque[0] = (uint16_t)(lround(f*32*2));
    
  }else{
    appParam.rpm = 0;
    appParam.period = 0;
    if(appParam.rpm_idle_seconds > moduleParam.idleTimeout){
      // enter sleep mode
      if(appParam.state == ST_IDLE || appParam.state == ST_ADVING){
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_CONTINUE);
//        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
      }
    }
    appParam.power = 0;
    appParam.torque[0] = 0;
    for(uint8_t i=0;i<appParam.tHis.nofElement;i++)
      appParam.tHis.his[i] = 0;
  }
  appParam.imu.radSum = 0;
  appParam.imu.time = 0;
  
  // battery check
  if(appParam.bat_volt < moduleParam.lbtThreshold){
    appParam.lbtCount++;
    if(appParam.lbtCount > moduleParam.lbtKeepSecs){
      appParam.lbtCount--;
      bsp_board_led_on(0);
      app_timer_start(m_flash_timer_id,APP_TIMER_TICKS(50),NULL);
    }
  }
  else{
    appParam.lbtCount = 0;
  }
  
}

uint32_t		m_flag_measure = 0;
static void inertial_timeout_handler(void * p_context)
{
  
}

void handler_report_evt(bool act,uint16_t interval)
{
  NRF_LOG_INFO("Report interval:%d",interval);
  uint32_t err_code;
  if(act){
    ad7124_cmd_config();  
    ad7124cmd_startConversion();    
    bmi160_cms_startConv();
#ifdef APP_ANT
      err_code = app_timer_start(m_report_timer_id,APP_TIMER_TICKS(250),NULL);
#else
    if(interval >= 5){
      err_code = app_timer_start(m_report_timer_id,APP_TIMER_TICKS(interval),NULL);
    }
    else
      err_code = app_timer_start(m_report_timer_id,APP_TIMER_TICKS(50),NULL);
#endif
    APP_ERROR_CHECK(err_code);
  }
  else{
    ad7124cmd_goStop();
    bmi160_cmd_stop();
    err_code = app_timer_stop(m_report_timer_id);
    APP_ERROR_CHECK(err_code);
  }
}

void report_timeout_handler(void *p_context)
{
  //NRF_LOG_INFO(__func__);
  uint8_t buf[20];
  memcpy(&buf[0],adc_buffer,8);
  if(appParam.opmode == CMD_PID_CONTROL_DEV_START){
    memcpy(&buf[8],bmi_buffer,12);
    app_ble_nus_feed_data(buf,20);
  }else if(appParam.opmode == CMD_PID_CONTROL_DEV_START_ADC){
    app_ble_nus_feed_data(buf,8);
  }
}
//void report_timeout_handler(void *p_context)
//{
//  //NRF_LOG_INFO(__func__);
//  uint8_t buf[20];
//  memcpy(&buf[0],&appParam.pitch,4);
//  memcpy(&buf[4],&appParam.pitch_prev,4);
//  memcpy(&buf[8],&appParam.torque_nm[0],4);
//  app_ble_nus_feed_data(buf,12);
//}
//static void measure_timeout_handler(void * p_context)
//{
//  UNUSED_PARAMETER(p_context);
//  uint8_t u8;
//  m_flag_measure = 1;
//  sysstate_t state = sysGetState();
//  switch(state){
//  case SYS_IDLE:
//  case SYS_NORMAL:
//    break;
//  case SYS_GO_SLEEP:
//    goSleep();
//    NRF_LOG_INFO("Go Sleep\r\n");
//    break;
//  case SYS_SLEEP:
//    // shut down system, wakeup by gpiote
//    
//    if(!appTimerStopped){
//      app_timer_stop_all();
//      appTimerStopped = true;
//    }
//    break;
//  case SYS_GO_WKUP:
//    NRF_LOG_INFO("Go Wakeup\r\n");
//    peripheral_start();
//
//    break;
//    
//  }
//  NRF_LOG_FLUSH();
//
//}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
  NRF_LOG_INFO("FUNC:%s, code:%d",__func__,ble_adv_evt);
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
          
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
          if(m_conn_handle == BLE_CONN_HANDLE_INVALID){
            appParam.state = ST_IDLE;
            NRF_LOG_INFO("BLE ADV IDLE");
          }
          //advertising_reconfig();
            //advertising_start(false);
            //sleep_mode_enter();
            break;
        default:
            break;
    }
}

/*********  main code section ********************/
static void advertising_reconfig()
{
  for(uint8_t i=0;i<m_advertising.advdata.p_manuf_specific_data->data.size;i++){
    m_advertising.advdata.p_manuf_specific_data->data.p_data[i]++;
  }
  //sd_ble_gap_adv_data_set
  advertising_start(false);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));
    
    ble_advdata_manuf_data_t    m_data;
    uint8_t data[] = "Hello, Grididea";
    m_data.company_identifier = 0x5329;
    m_data.data.p_data = data;
    m_data.data.size = sizeof(data);
    
    //int8_t tx_power = 4;
    //init.advdata.p_manuf_specific_data = &m_data;

    //init.advdata.p_tx_power_level = &tx_power;
    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
//    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    init.evt_handler = on_adv_evt;
    
    //init.config.ble_adv_on_disconnect_disabled = true;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{

    // Initialize timer module.
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_inertial_timer_id,
                            APP_TIMER_MODE_REPEATED,
                            inertial_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_report_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                report_timeout_handler);
    APP_ERROR_CHECK(err_code);    
    
    err_code = app_timer_create(&m_periodic_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                periodic_timeout_handler);
    APP_ERROR_CHECK(err_code);    

    err_code = app_timer_create(&m_flash_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                flash_timeout_handler);
    APP_ERROR_CHECK(err_code);    
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
//                                          (const uint8_t *) moduleParam.appName,
//                                          strlen(moduleParam.appName));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = sd_ble_gap_tx_power_set(4);    
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;

    // dfu service
    ble_dfu_buttonless_init_t dfus_init =
    {
        .evt_handler = ble_dfu_evt_handler
    };

    // Initialize the async SVCI interface to bootloader.
    if(NRF_UICR->NRFFW[0] != 0xffffffff){
      err_code = ble_dfu_buttonless_async_svci_init();
      APP_ERROR_CHECK(err_code);
    }    
    
//    err_code = ble_dfu_buttonless_init(&dfus_init);
//    APP_ERROR_CHECK(err_code);

    // NUS service
    app_ble_nus_init(handler_report_evt);
    
    // sensornode service
    //app_sensornode_init();
    
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
  uint32_t err_code;
  err_code = app_timer_start(m_periodic_timer_id, MEASURE_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
    ble_gap_phys_t phy;
    phy.rx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS;
    phy.tx_phys = BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            appParam.state = ST_CONNECTED;
            sd_ble_gap_adv_stop();
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            set_conn_handle(m_conn_handle);
            appParam.state = ST_IDLE;
            advertising_start(false);
            break;

#if defined(S132)
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_INFO("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
        case BLE_GAP_EVT_PHY_UPDATE:
        {
          ble_gap_evt_phy_update_t phy_update = p_ble_evt->evt.gap_evt.params.phy_update;
          if(phy_update.status == BLE_HCI_STATUS_CODE_SUCCESS)
          {
          NRF_LOG_INFO("PHY updated: %i, %i", phy_update.tx_phy, phy_update.rx_phy);
          //app_aggregator_phy_update(p_ble_evt->evt.gap_evt.conn_handle, phy_update.tx_phy, phy_update.rx_phy);
          }
        } break;
#endif

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_INFO("gap evt sec params request.");
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

         case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            ble_gap_data_length_params_t dl_params;

            // Clearing the struct will effectivly set members to @ref BLE_GAP_DATA_LENGTH_AUTO
            memset(&dl_params, 0, sizeof(ble_gap_data_length_params_t));
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dl_params, NULL);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    if(!nrf_sdh_is_enabled()){
      err_code = nrf_sdh_enable_request();
      APP_ERROR_CHECK(err_code);
    }
    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

//    ble_cfg_t ble_cfg;
//    memset(&ble_cfg,0,sizeof(ble_cfg_t));
//    ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
//    ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = 5;
//    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS,&ble_cfg,ram_start);
//    APP_ERROR_CHECK(err_code);
    
    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init()
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, BIN_PKT_SZ+3);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}

/**
 * @brief Function for setup all thinks not directly associated with ANT stack/protocol.
 * @desc Initialization of: @n
 *         - app_tarce for debug.
 *         - app_timer, pre-setup for bsp.
 *         - bsp for signaling LEDs and user buttons.
 */
static void utils_setup(void)
{
  ret_code_t err_code;
  
  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);
    // Initialize and start a single continuous mode timer, which is used to update the event time
    // on the main data page.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);


    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    
    ble_stack_init();
    gap_params_init();
    gatt_init();

    
    
}

static void hal_init(void)
{
  ret_code_t err_code;
  
  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);
  
  bsp_board_leds_init();
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
//  fs_sys_event_handler(sys_evt);
  ant_boot_settings_event(sys_evt);
}

static uint32_t sd_evt_handler(void)
{
  uint32_t ulEvent;
  while(sd_evt_get(&ulEvent) != NRF_ERROR_NOT_FOUND){
    ant_boot_settings_event(ulEvent);
  }
  
  return 0;
}

static volatile uint8_t writeflag = 0;
volatile uint8_t fdReady = 0;

static void fds_evt_handler(fds_evt_t const * const p_fds_evt)
{
  switch(p_fds_evt->id){
  case FDS_EVT_INIT:
    if(p_fds_evt->result == FDS_SUCCESS)
      fdReady = 1;
    else
      fdReady = 0;
    break;
  case FDS_EVT_WRITE:
    writeflag = 1;
    break;
  case FDS_EVT_UPDATE:
    if(p_fds_evt->result == FDS_SUCCESS)
    {
      fdReady = 1;
    }
    break;
  default:
    break;
  }
}
/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
//static void buttons_leds_init(bool * p_erase_bonds)
//{
//    uint32_t err_code;
//    bsp_event_t startup_event;
//
//    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);
//
//    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
//}

/**@brief Function to execute while waiting for the wait burst busy flag
*/
static void event_burst_wait_handle(void)
{
    // No implementation needed
}

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Power manager.
 */
static void log_init(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
static void power_management_init(void)
{
    uint32_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}
/** @brief Clear bonding information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
  NRF_LOG_INFO(__func__);
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEBUG("advertising is started");
    }
}
uint16_t adc_cntr = 0;
void adc_drdy(void)
{
  adc_cntr++;
  if(appParam.opmode == 0 && appParam.sampleToIgnore == 0)
    sensorDataFeedADC(0,adc_buffer[0]>>8);
}

void bmi_drdy(void)
{
  //NRF_LOG_INFO(__func__);
  //todo: feed IMU data to filter and fusion
  if(appParam.sampleToIgnore)
    appParam.sampleToIgnore--;
  if(appParam.opmode == 0 && appParam.sampleToIgnore == 0)
    sensorDataFeedIMU(bmi_buffer);
}

#define USE_SD   0
uint32_t loopCount = 0;
uint32_t hclk_run;
//static EventQueue eventQueue(16*32);
#define FPU_EXCEPTION_MASK 0x0000009F 
/**@brief Function for application main entry, does not return.
 */
int main(void)
{
  log_init();
  hal_init();
  timers_init();
  power_management_init();
  ble_stack_init();
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();
  
  sys_param_init();

  appParam.rpmHis.nofElement = 8;
  appParam.rpmHis.index = 0;
  appParam.tHis.nofElement = 4;
  appParam.tHis.index = 0;
  
  app_adc_init();
  //peer_manage_init();
  uint16_t rate = 25*(1 << (moduleParam.imu_rate_code-0x6));
  sensorDataInit(rate);
  appParam.adc_ptr = adc_buffer;
  appParam.bmi_ptr = bmi_buffer;

  appParam.sampleToIgnore = 50;
  bmi160_cmd_init(bmi_drdy,bmi_buffer);
  ad7124cmd_init(adc_drdy, adc_buffer);
#ifdef APP_ANT
  ant_bpwr_init();
#endif
  NRF_LOG_INFO("Application started\n");
  ad7124cmd_startConversion();
  bmi160_cms_startConv();

  //bmi160_cmd_start_anymotion();
  application_timers_start();
  advertising_start(false);
  appParam.state = ST_ADVING;
  app_adc_start();
#ifdef APP_ANT
  ant_bpwr_start();
#endif
    for (;;)
    {
      if(NRF_LOG_PROCESS() == false){
        nrf_pwr_mgmt_run();
      }
    } 
  
}


/**
 *@}
 **/
