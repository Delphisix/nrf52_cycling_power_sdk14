#ifndef _SYSPARAMS_
#define _SYSPARAMS_
#include "nrf.h"
#include "app_error.h"

typedef struct{
  uint8_t flag;
  uint8_t state;
  uint32_t senseoMask;
  uint32_t scan_interval;
  uint8_t adc_gain_code;
  uint16_t adc_filter_code;
  uint8_t imu_rate_code;
  uint8_t imu_acc_range;
  uint8_t imu_gyro_range;
  uint8_t dummy[3];
}module_param_t;
  
extern module_param_t moduleParam;

void sys_param_init(void);
ret_code_t sysparam_update(void);
ret_code_t sysparam_load(void);
#endif