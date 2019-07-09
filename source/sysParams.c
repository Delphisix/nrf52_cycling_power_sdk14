/**
filename: sysparams.h
autohr: Jason
description: system parameters storage using FDS 
for NRF5


*/

#include "sysparams.h"
#include "fds.h"

#define NRF_LOG_MODULE_NAME     sysparam_m
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define FILE_ID 0x5329
#define REC_KEY 0x9210

module_param_t moduleParam;
app_param_t appParam;

static volatile bool m_fds_ready = false;
static volatile bool m_pending_write = false;
static volatile bool m_pending_update = false;

static uint32_t m_pending_msg_size = 0;
static uint8_t const * m_pending_msg_buff = NULL;

static fds_record_desc_t m_record_desc;
static fds_record_t m_record;


static void file_prepare_record(uint8_t const *p_buff, uint32_t size)
{
  m_record.file_id = FILE_ID;
  m_record.key = REC_KEY;
  m_record.data.p_data = p_buff;
  m_record.data.length_words = BYTES_TO_WORDS(size);
  
}

static ret_code_t file_create(uint8_t const *p_buff,uint32_t size)
{
  ret_code_t err_code;
  
  file_prepare_record(p_buff,size);
  
  err_code = fds_record_write(&m_record_desc,&m_record);
  if(err_code == FDS_ERR_NO_SPACE_IN_FLASH){
    m_pending_write = true;
    m_pending_msg_size = size;
    m_pending_msg_buff = p_buff;
    NRF_LOG_INFO("FDS has no space left, garbage collector triggered!");
    err_code = fds_gc();
  }
  return err_code;
}

static void fds_evt_handler(fds_evt_t const * const p_fds_evt)
{
  ret_code_t err_code;
  
  NRF_LOG_INFO(__func__);
  switch(p_fds_evt->id){
  case FDS_EVT_INIT:
    APP_ERROR_CHECK(p_fds_evt->result);
    m_fds_ready = true;
    break;
  case FDS_EVT_UPDATE:
    APP_ERROR_CHECK(p_fds_evt->result);
    NRF_LOG_INFO("FDS update success");
    break;
  case FDS_EVT_WRITE:
    APP_ERROR_CHECK(p_fds_evt->result);
    NRF_LOG_INFO("FDS write success");
    break;
  case FDS_EVT_GC:
    APP_ERROR_CHECK(p_fds_evt->result);
    NRF_LOG_INFO("FDS finish garbage collect");
    
    if(m_pending_write){
      m_pending_write = false;
      err_code = file_create(m_pending_msg_buff, m_pending_msg_size);
      APP_ERROR_CHECK(err_code);
    }else if(m_pending_update){
      NRF_LOG_DEBUG("Update pending msg.", p_fds_evt->id, p_fds_evt->result);
      m_pending_update = false;
      err_code = sysparam_update();
      APP_ERROR_CHECK(err_code);
    }
    break;
  default:
    break;
  }
  
}


void sys_param_init(void)
{
  ret_code_t err_code;
  
  err_code = fds_register(fds_evt_handler);
  APP_ERROR_CHECK(err_code);
  
  err_code = fds_init();
  APP_ERROR_CHECK(err_code);
  
  while(!m_fds_ready);
 
  sysparam_load();
  
  moduleParam.state = 0;
}

ret_code_t sysparam_update(void)
{
  ret_code_t err_code;
  file_prepare_record((uint8_t*)&moduleParam,sizeof(moduleParam));
  
  err_code = fds_record_update(&m_record_desc,&m_record);
  if(err_code == FDS_ERR_NO_SPACE_IN_FLASH){
    m_pending_update = true;
    m_pending_msg_size = sizeof(moduleParam);
    m_pending_msg_buff = (uint8_t*)&moduleParam;
    err_code = fds_gc();
  }
  return err_code;
}

ret_code_t sysparam_load(void)
{
  ret_code_t err_code;
  fds_find_token_t ftok;
  fds_flash_record_t flash_record;
  
  memset(&ftok,0x0,sizeof(fds_find_token_t));
  
  err_code = fds_record_find(FILE_ID,REC_KEY,&m_record_desc,&ftok);
  
  if(err_code == FDS_SUCCESS){
    NRF_LOG_INFO("sysparam found");
    
    err_code = fds_record_open(&m_record_desc,&flash_record);
    APP_ERROR_CHECK(err_code);
    
    memcpy((uint8_t*)&moduleParam,flash_record.p_data,flash_record.p_header->length_words * sizeof(uint32_t));
    
    err_code = fds_record_close(&m_record_desc);
  }
  else if(err_code == FDS_ERR_NOT_FOUND){
    NRF_LOG_INFO("Create sysparam record");
    moduleParam.flag = 0xAA;
    moduleParam.adc_filter_code = 59;
    moduleParam.adc_gain_code = 0x7;
    moduleParam.imu_acc_range = 0x0;
    moduleParam.imu_gyro_range = 0x0;
    moduleParam.imu_rate_code = 0x8;
    moduleParam.senseoMask = 0xffffffff;
    moduleParam.scan_interval = 50;
    memcpy(moduleParam.appName,"Grididea BP01\0",14);
    err_code = file_create((uint8_t*)&moduleParam,sizeof(moduleParam));
  }
  return err_code;
  
}
