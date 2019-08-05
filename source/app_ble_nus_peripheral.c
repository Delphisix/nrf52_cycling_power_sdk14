/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "packetValid.h"
#include "bmi160_cmd.h"
#include "app_ble_nus_peripheral.h"
#include "sysparams.h"

//extern ble_advertising_t m_advertising;

actionControl actCtrl = NULL;
BLE_NUS_DEF(m_nus);                                                                 /**< BLE NUS service instance. */

//static uint16_t scan_interval = 1000; // in unit of 100us

//static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
//static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
//static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
//{
//    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
//};

#define PACKET_BUFFER_SIZE      BIN_PKT_SZ
static uint8_t packet[BIN_PKT_SZ*2];
static uint8_t packet2[BIN_PKT_SZ*2];
buffer_t pktBuf={packet,packet,packet,packet2,0,0,0};
void sendData();
void app_buf_write_cb(uint8_t *b, uint16_t len)
{
  
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static volatile bool txReady = true;
void nus_data_handler(ble_nus_evt_t * p_evt)
{
  //NRF_LOG_INFO(__func__);

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        //NRF_LOG_INFO("Received data from BLE NUS.");
        //NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        //actCtrl(true,scan_interval);
        //uint8_t str[]="12345";
        //uint16_t sz=5;
        //err_code = ble_nus_string_send(&m_nus,str,&sz);
        writePacketBuffer(p_evt->params.rx_data.p_data,p_evt->params.rx_data.length);
    }
    else if(p_evt->type == BLE_NUS_EVT_TX_RDY){
      //NRF_LOG_INFO("TX completed");
      bsp_board_led_off(0);
      txReady = true;
      //if(pktBuf.pendBuffer)
       // sendData();
    }

}
/**@snippet [Handling the data received over BLE] */



// PACKET HANDLE CODE START
//#define REPORT_INTERVAL APP_TIMER_TICKS(100)
//APP_TIMER_DEF(m_report_timer_id);

//uint8_t vv;
//uint8_t buff[208];
//uint16_t szz = 208;
void sendData()
{
  uint32_t err;
  if(txReady && pktBuf.pendBuffer){
    //NRF_LOG_INFO("Send size:%d",pktBuf.szToRead);
    txReady = false;
    pktBuf.pendBuffer--;
    err = ble_nus_string_send(&m_nus,pktBuf.r,&pktBuf.szToRead);
    bsp_board_led_on(0);
    APP_ERROR_CHECK(err);
  }else{
    //NRF_LOG_INFO("Current pend buffer %d",pktBuf.pendBuffer);
  }
}

void app_ble_nus_feed_data(uint8_t *p, uint8_t sz)
{
  uint32_t err;
  memcpy(pktBuf.w, p, sz);
  pktBuf.w += sz;
  pktBuf.szWritten += sz;
  if((pktBuf.szWritten+CMD_STRUCT_SZ) >= BIN_PKT_SZ){
    buildPacketHeader(MASK_TYPE_DATA|0x7,appParam.pid++,(pktBuf.bufId==0?pktBuf.b1:pktBuf.b2),pktBuf.szWritten+CMD_STRUCT_SZ);
    // put into nus buffer
    pktBuf.r = pktBuf.bufId==0?pktBuf.b1:pktBuf.b2;
    pktBuf.w = pktBuf.bufId==0?pktBuf.b2:pktBuf.b1;
    pktBuf.w += CMD_STRUCT_SZ;
    pktBuf.bufId = pktBuf.bufId == 0? 1:0;
    pktBuf.szToRead = pktBuf.szWritten + CMD_STRUCT_SZ;
    //pktBuf.szToRead = BIN_PKT_SZ;
    pktBuf.szWritten = 0;
    pktBuf.pendBuffer++;
    //memset(pktBuf.r,vv++,208);
  }
  sendData();
  
}

void set_conn_handle(uint16_t h)
{
  m_nus.conn_handle = h;
}

void report_cmd_ok(cmd_header_t *h)
{
  //return ;
  uint8_t b[8];
  uint8_t pid = (h->type & 0xf) | MASK_TYPE_RET_OK;
  uint16_t len = buildPacket(h->type, pid,NULL,0,b);
  uint32_t err = ble_nus_string_send(&m_nus,b,&len);
  APP_ERROR_CHECK(err);
  NRF_LOG_INFO(__func__,err);
}

void report_cmd_ng(cmd_header_t *h)
{
  //return ;
  uint32_t err;
  uint8_t b[8];
  uint8_t pid = (h->type & 0xf) | MASK_TYPE_RET_ERR;
  uint16_t len = buildPacket(h->type, pid,NULL,0,b);
  err = ble_nus_string_send(&m_nus,b,&len);
  APP_ERROR_CHECK(err);
}

void handlePacket(uint8_t *b, uint8_t target)
{
  NRF_LOG_INFO(__func__);
  uint32_t err_code;
  cmd_header_t *header = (cmd_header_t*)b;
  if(target == CMD_DEVICE){
      NRF_LOG_INFO("Device Command");
      uint8_t cmd = header->pid & (~NOCRC_MASK);
      uint16_t arg;
      appParam.opmode = cmd;
      switch(header->type){
        case MASK_TYPE | CMD_TYPE_CONTROL:{
          switch(cmd){
          case CMD_PID_CONTROL_DEV_STOP:
            NRF_LOG_INFO("Stop transfer");
            // todo : issue stop transfer
            if(actCtrl){
              actCtrl(false,0);
              moduleParam.state = 0;
            }
            report_cmd_ok(header);
            break;
          case CMD_PID_CONTROL_DEV_START:
          case CMD_PID_CONTROL_DEV_START_ADC:
            NRF_LOG_INFO("Start transfer");
            // todo : issue start transfer
            pktBuf.r = packet;
            pktBuf.w = packet + CMD_STRUCT_SZ;
            pktBuf.pendBuffer = 0;
            if(actCtrl && (moduleParam.state == 0)){
              actCtrl(true,moduleParam.scan_interval/10);
              moduleParam.state = 1;
              appParam.pid = 0;
            }
            report_cmd_ok(header);
            break;
          default:
            report_cmd_ng(header);
          }
        }break;
        case MASK_TYPE | CMD_TYPE_SETUP:{
          switch(cmd){
          case CMD_PID_SETUP_DEV_SCAN_INTERVAL:
            if(header->len == 10){
              moduleParam.scan_interval = (b[8]) | (b[9]<<8);
              report_cmd_ok(header);
            }
            else if(header->len == 8){
              uint8_t b[10];
              uint8_t type = (header->type & 0xf) | MASK_TYPE_RET_CFG;
              b[8] = moduleParam.scan_interval & 0xff;
              b[9] = (moduleParam.scan_interval >> 8) & 0xff;
              uint16_t len = buildPacket(type,header->pid,NULL,2,b);
              ble_nus_string_send(&m_nus,b,&len);
            }
            else{
              report_cmd_ng(header);
            }
            break;
          case CMD_PID_SETUP_DEV_ADC:
            if(header->len == 8){
              uint8_t b[11];
              uint8_t type = (header->type & 0xf) | MASK_TYPE_RET_CFG;
              b[8] = moduleParam.adc_gain_code;
              b[9] = moduleParam.adc_filter_code & 0xff;
              b[10] = (moduleParam.adc_filter_code >> 8) & 0xff;
              uint16_t len = buildPacket(type,header->pid,NULL,3,b);
              ble_nus_string_send(&m_nus,b,&len);
            }else{
              if((b[8] < 8)){
                moduleParam.adc_gain_code = b[8];
                uint16_t fs = b[9] | (b[10]<<8);
                moduleParam.adc_filter_code = fs;
                report_cmd_ok(header);
              }else{
                report_cmd_ng(header);
              }              
            }
            break;
          case CMD_PID_SETUP_DEV_IMU:
            if(header->len == 8){
              uint8_t b[11];
              uint8_t type = (header->type & 0xf) | MASK_TYPE_RET_CFG;
              b[8] = moduleParam.imu_rate_code;
              b[9] = moduleParam.imu_acc_range;
              b[10] = moduleParam.imu_gyro_range;
              uint16_t len = buildPacket(type,header->pid,NULL,3,b);
              ble_nus_string_send(&m_nus,b,&len);
            }else{
              moduleParam.imu_acc_range = b[8];
              moduleParam.imu_gyro_range = b[9];
              moduleParam.imu_rate_code = b[10];
              report_cmd_ok(header);
            }
            break;
          case CMD_PID_SETUP_SYSTEM:
            if(header->len == 9){
              if(b[8] == 0x02){
                memcpy(&b[9],moduleParam.appName,16);
                uint16_t len = buildPacket(header->type & 0xf,header->pid,NULL,16,b);
                ble_nus_string_send(&m_nus,b,&len);
              }
              else{
                report_cmd_ng(header);
              }
            }
            else{
              if(b[8] == 0x02){
                memcpy(moduleParam.appName,&b[9],header->len-9);
                report_cmd_ok(header);
              }
              else{
                report_cmd_ng(header);
              }
            }
            break;
          case CMD_PID_SETUP_CFG_SAVE:
            NRF_LOG_INFO("Save parameters");
            report_cmd_ok(header);
            sysparam_update();
            break;
          }
        }break;
      }
  }else{
    report_cmd_ng(header);
  }
  
}

// PACKET HANDLE CODE END


/**@brief Application main function.
 */
int app_ble_nus_init(actionControl f)
{
    uint32_t err_code;
    bool     erase_bonds;
    
    if(f) actCtrl = f;
    
    initPacket(handlePacket);

//    NRF_LOG_INFO("UART Start!");
    
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

  return 0;
}

//void app_ble_nus_start()
//{
//    uint32_t err_code;
//    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
//    APP_ERROR_CHECK(err_code);
//}


/**
 * @}
 */
