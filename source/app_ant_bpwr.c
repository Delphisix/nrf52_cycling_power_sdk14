/**

file: app_ant_bpwr.c
autohr: Jason
description: ant plus bicycle power application



************************************************************/
#include <string.h>
#include "nrf_sdh.h"
#include "ant_ota.h"
#include "ant_stack_config.h"
#include "ant_bpwr.h"
#include "ant_state_indicator.h"
#include "ant_key_manager.h"
#include "ant_bpwr_simulator.h"

#include "app_ant_bpwr.h"
#include "sysParams.h"
#include "nrf_log.h"

#define BPWR_CHANNEL_NUMBER         0x00 /**< Channel number assigned to Bicycle Power profile. */
#define ANTPLUS_NETWORK_NUMBER      0       /**< Network number. */

void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event);
void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1);


BPWR_SENS_CHANNEL_CONFIG_DEF(m_ant_bpwr,
                             BPWR_CHANNEL_NUMBER,
                             CHAN_ID_TRANS_TYPE,
                             CHAN_ID_DEV_NUM,
                             ANTPLUS_NETWORK_NUMBER);

BPWR_SENS_PROFILE_CONFIG_DEF(m_ant_bpwr,
                            (ant_bpwr_torque_t)(SENSOR_TYPE),
                            ant_bpwr_calib_handler,
                            ant_bpwr_evt_handler);


static ant_bpwr_profile_t m_ant_bpwr;
NRF_SDH_ANT_OBSERVER(m_ant_observer, ANT_BPWR_ANT_OBSERVER_PRIO,
                     ant_bpwr_sens_evt_handler, &m_ant_bpwr);


/** @snippet [ANT BPWR TX Profile handling] */
static ant_bpwr_simulator_t  m_ant_bpwr_simulator;


/**@brief Function for dispatching an ANT stack event to all modules with an ANT stack event handler.
 *
 * @details This function is called from the ANT Stack event interrupt handler after an ANT stack
 *          event has been received.
 *
 * @param[in] p_ant_evt  ANT stack event.
 *
 * @snippet [ANT BPWR TX Profile handling] */
void ant_evt_dispatch(ant_evt_t * p_ant_evt)
{
  //ant_bpwr_sens_evt_handler(&m_ant_bpwr, p_ant_evt);
  //ant_state_indicator_evt_handler(p_ant_evt);
  //ota_channel_event_handle(p_ant_evt);
}


/**@brief Function for handling ANT BPWR events.
 */
/** @snippet [ANT BPWR simulator call] */
void ant_bpwr_evt_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_evt_t event)
{
  uint32_t test = 0;
  int16_t retv;
  int16_t polls;
 
  switch (event)
  {
    case ANT_BPWR_PAGE_1_UPDATED:
      m_ant_bpwr.page_1.data.general_calib = ANT_BPWR_CALIB_ID_AUTO_SUPPORT;
      m_ant_bpwr.page_1.auto_zero_status = ANT_BPWR_AUTO_ZERO_OFF;
      break;
            /* fall through */
    case ANT_BPWR_PAGE_16_UPDATED:
      m_ant_bpwr.page_16.update_event_count++;
      //m_ant_bpwr.page_16. = (uint16_t)appParam.rpm;
      m_ant_bpwr.page_16.instantaneous_power = appParam.power;
      m_ant_bpwr.common.instantaneous_cadence = appParam.rpm;
      break;
            /* fall through */
    case ANT_BPWR_PAGE_17_UPDATED:
      m_ant_bpwr.page_17.update_event_count++;
      m_ant_bpwr.page_17.accumulated_torque += appParam.torque[0];
      m_ant_bpwr.page_17.period = 0;
      m_ant_bpwr.page_17.tick = 0;
      
      break;
            /* fall through */
    case ANT_BPWR_PAGE_18_UPDATED:
      m_ant_bpwr.page_18.update_event_count++;
      m_ant_bpwr.page_18.accumulated_torque += appParam.torque[0];
      m_ant_bpwr.page_18.tick++;
      m_ant_bpwr.page_18.period += appParam.period;
      
      break;
            /* fall through */
    case ANT_BPWR_PAGE_80_UPDATED:
      
      break;
            /* fall through */
    case ANT_BPWR_PAGE_81_UPDATED:
     
      break;
  case ANT_BPWR_PAGE_82_UPDATED:
    m_ant_bpwr.page_82.battery_indicator++;
    break;
//    default:
//      ant_bpwr_simulator_one_iteration(&m_ant_bpwr_simulator, event);
  }
   
}

/**@brief Function for handling ANT BPWR events.
 */
/** @snippet [ANT BPWR calibration] */
void ant_bpwr_calib_handler(ant_bpwr_profile_t * p_profile, ant_bpwr_page1_data_t * p_page1)
{
  int32_t tmp;
    switch (p_page1->calibration_id)
    {
        case ANT_BPWR_CALIB_ID_MANUAL:
            m_ant_bpwr.BPWR_PROFILE_calibration_id     = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
            moduleParam.adcOffset[0] = m_ant_bpwr.page_1.data.custom_calib[4] << 8;
            moduleParam.adcOffset[0] |= m_ant_bpwr.page_1.data.custom_calib[5];
            NRF_LOG_INFO("Save parameters manual");
            sysparam_update();
            break;

        case ANT_BPWR_CALIB_ID_AUTO:
            m_ant_bpwr.BPWR_PROFILE_calibration_id     = ANT_BPWR_CALIB_ID_MANUAL_SUCCESS;
            m_ant_bpwr.BPWR_PROFILE_auto_zero_status   = p_page1->auto_zero_status;
            m_ant_bpwr.BPWR_PROFILE_general_calib_data = CALIBRATION_DATA;
            tmp = (appParam.adc_ptr[0] - 0x80000000) >> 16;
            moduleParam.adcOffset[0] = (int16_t)tmp;
            NRF_LOG_INFO("Save parameters auto");
            sysparam_update();
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_REQ:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_REQ_SUCCESS;
            memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data, p_page1->data.custom_calib,
                   sizeof (m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
            break;

        case ANT_BPWR_CALIB_ID_CUSTOM_UPDATE:
            m_ant_bpwr.BPWR_PROFILE_calibration_id = ANT_BPWR_CALIB_ID_CUSTOM_UPDATE_SUCCESS;
            memcpy(m_ant_bpwr.BPWR_PROFILE_custom_calib_data, p_page1->data.custom_calib,
                   sizeof (m_ant_bpwr.BPWR_PROFILE_custom_calib_data));
            break;

        default:
            break;
    }
}
/** @snippet [ANT BPWR calibration] */



///**@brief Function for the BPWR simulator initialization.
// */
//void simulator_setup(void)
//{
//    /** @snippet [ANT BPWR simulator init] */
//    const ant_bpwr_simulator_cfg_t simulator_cfg =
//    {
//        .p_profile   = &m_ant_bpwr,
//        .sensor_type = (ant_bpwr_torque_t)(SENSOR_TYPE),
//    };
//    /** @snippet [ANT BPWR simulator init] */
//
//#if MODIFICATION_TYPE == MODIFICATION_TYPE_AUTO
//    /** @snippet [ANT BPWR simulator auto init] */
//    ant_bpwr_simulator_init(&m_ant_bpwr_simulator, &simulator_cfg, true);
//    /** @snippet [ANT BPWR simulator auto init] */
//#else
//    /** @snippet [ANT BPWR simulator button init] */
//    ant_bpwr_simulator_init(&m_ant_bpwr_simulator, &simulator_cfg, false);
//    /** @snippet [ANT BPWR simulator button init] */
//#endif
//}


/**
 * @brief Function for ANT stack initialization.
 *
 * @details Initializes the SoftDevice and the ANT event interrupt.
 */
static void softdevice_setup(void)
{
  ret_code_t err_code;
  if(!nrf_sdh_is_enabled()){
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
  }

    ASSERT(nrf_sdh_is_enabled());

    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);

    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUM);
    APP_ERROR_CHECK(err_code);  
}

/**
 * @brief Function for Bicycle Power profile initialization.
 *
 * @details Initializes the Bicycle Power profile and open ANT channel.
 */
static void profile_setup(void)
{
/** @snippet [ANT BPWR TX Profile Setup] */
    ret_code_t err_code;

    err_code = ant_bpwr_sens_init(&m_ant_bpwr,
                                  BPWR_SENS_CHANNEL_CONFIG(m_ant_bpwr),
                                  BPWR_SENS_PROFILE_CONFIG(m_ant_bpwr));
    APP_ERROR_CHECK(err_code);

    // fill manufacturer's common data page.
    m_ant_bpwr.page_80 = ANT_COMMON_page80(moduleParam.hw_revision,
                                           moduleParam.manufacturer_id,
                                           moduleParam.model_number);
//    m_ant_bpwr.page_80 = ANT_COMMON_page80(BPWR_HW_REVISION,
//                                           BPWR_MANUFACTURER_ID,
//                                           BPWR_MODEL_NUMBER);
    // fill product's common data page.
    m_ant_bpwr.page_81 = ANT_COMMON_page81(moduleParam.sw_revision_minor,
                                           moduleParam.sw_revision_major,
                                           moduleParam.serial_number);
//    m_ant_bpwr.page_81 = ANT_COMMON_page81(BPWR_SW_REVISION_MAJOR,
//                                           BPWR_SW_REVISION_MINOR,
//                                           BPWR_SERIAL_NUMBER);

    m_ant_bpwr.BPWR_PROFILE_auto_zero_status = ANT_BPWR_AUTO_ZERO_OFF;

    m_ant_bpwr.page_82 = ANT_COMMON_page82(0x11,0x00,0x00,0x00,0x00,0x13);

/** @snippet [ANT BPWR TX Profile Setup] */
}

int8_t ant_bpwr_init()
{
  ret_code_t err_code;
  err_code = ant_state_indicator_init(m_ant_bpwr.channel_number, BPWR_SENS_CHANNEL_TYPE);
  APP_ERROR_CHECK(err_code);

  softdevice_setup();
  profile_setup();
  
  return 0;
}

int8_t ant_bpwr_start()
{
  ret_code_t err_code;
    err_code = ant_bpwr_sens_open(&m_ant_bpwr);
    APP_ERROR_CHECK(err_code);

    err_code = ant_state_indicator_channel_opened();
    APP_ERROR_CHECK(err_code);
  
}