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
  float vbatRatio;
  float torqueRatio;
  int16_t adcOffset[2];
  uint8_t dummy[3];
  uint8_t appName[16];
}module_param_t;

typedef enum{
  ST_IDLE,
  ST_ADVING,
  ST_CONNECTED,
  ST_DISCONNECTED
}sysState_t;



typedef struct{
  uint8_t n;
  float sum;
}_n_avg_t;

typedef struct{
  float t_pos;
  float t_neg;
  float t_total;
  uint8_t valid;
  float smooth,effeciency;
  bool isPositive,isNegative;
  uint16_t n_pos,n_neg; // sample number
  _n_avg_t mappedTorque[12]; // map to 12 slot/cycle
  uint8_t lastIndex;
  float t_his[256];
  uint8_t hisIndex;
}_accum_torque_t;

typedef struct{
  bool isPos,isNeg;
  float val;
  float time;
  float history[8];
  uint8_t count;
  float sampleIntervalMs;
}_accum_imu_t;



typedef struct{
  sysState_t state;
  //torque_data_t torque;
  uint16_t power;
 // int32_t rpm;
  double dTorque;
  double staticTorque;
  uint8_t actChannel;
  bool goStop;  
  int16_t torque[2]; // in unit of 1/32 N-m
  float torque_nm[2];
  float bat_volt;
  float rpm;
  float pitch;
  float pitch_prev;
  float imt_sample_interval;
  float sampleTime;
  float accum_torque;
  uint16_t nofADCSamples;
  _accum_torque_t t;
  uint32_t *adc_ptr;
  int8_t *bmi_ptr;
  uint16_t period;
  uint16_t period_cnt;
  float period_ms;
  uint8_t validPeriod;
  uint8_t idleCount;
  uint8_t pid;
  _accum_imu_t imu;
  uint8_t opmode;
}app_param_t;
  
extern module_param_t moduleParam;
extern app_param_t appParam;
void sys_param_init(void);
ret_code_t sysparam_update(void);
ret_code_t sysparam_load(void);
#endif