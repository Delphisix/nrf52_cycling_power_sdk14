#ifndef _SENSORDATA_
#define _SENSORDATA_
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "recursive.h"
#include "time_domain.h"
#include "mahonyAHRS.h"

typedef struct{
  float scale;
  int32_t offset;
  filter_object_t filter;
  filter_func f;
}analogData_t;

typedef struct{
  uint32_t calcSample;
  _vibration_object_t x,y,z;
}vibrationData_t;

typedef void (*fusion_func)(float,float,float,float,float,float,float);


#define VIBRATION_DEF(_name,samples) \
static vibrationData_t _name = { \
  samples, \
  {0,0,0,0,0,0},\
  {0,0,0,0,0,0},\
  {0,0,0,0,0,0},\
};

#define ANALOG_LPF_DEF(_name) \
static float _name##_a[1]; \
static float _name##_b[1]; \
static float _name##_x[1]; \
static float _name##_y[1]; \
static analogData_t _name={ \
  1.0,0, \
  { \
    LPF,_name##_a,_name##_b,1,_name##_x,_name##_y \
  }, \
  recursive_lpf, \
}; \

#define ANALOG_HPF_DEF(_name) \
static float _name##_a[2]; \
static float _name##_b[1]; \
static float _name##_x[1]; \
static float _name##_y[1]; \
static analogData_t _name={ \
  1.0,0, \
  { \
    HPF,_name##_a,_name##_b,1,_name##_x,_name##_y \
  }, \
  recursive_hpf, \
}; \
  
#define IMU_DATA_DEF(_name) \
  ANALOG_LPF_DEF(_name##_ax) \
  ANALOG_LPF_DEF(_name##_ay) \
  ANALOG_LPF_DEF(_name##_az) \
  ANALOG_HPF_DEF(_name##_gx) \
  ANALOG_HPF_DEF(_name##_gy) \
  ANALOG_HPF_DEF(_name##_gz) \
  fusion_func _name##_fusion \
    


void sensorDataFeedADC(uint8_t ch, uint32_t v);
void sensorDataFeedIMU(uint8_t *b);
void sensorDataInit(uint16_t imuRate);
#endif