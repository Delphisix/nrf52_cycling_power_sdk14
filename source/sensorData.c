#include <stdio.h>
#include <string.h>
#include <math.h>
#include "nrf.h"
#include "mahonyAHRS.h"
#include "sensorData.h"
#include "sysParams.h"

#define RAD2DEG 180./3.141593

typedef struct{
  int16_t gx,gy,gz,ax,ay,az;
}_imu_data_t;

typedef struct{
  float gx,gy,gz,ax,ay,az;
}_imu_data_float_t;

ANALOG_LPF_DEF(ana_ch0);
//ANALOG_LPF_DEF(ana_ch1);

ANALOG_LPF_DEF(m_rpm);

IMU_DATA_DEF(imu);
VIBRATION_DEF(accel,1000);
ANALOG_LPF_DEF(m_pitch);
static float m_pitchDeg,m_pitchDegPrev;;
static float m_imuSampleInterval;       // in second
static float accumulated_pitch_angle;
static float accumulated_time;

#define PITCH_HISTORY_SZ        8
static float pitchHistory[PITCH_HISTORY_SZ]; 
static uint8_t pitchHisIndex = 0;

void sensorDataFeedADC(uint8_t ch, uint32_t v)
{
  ana_ch0.filter.x[0] = ((int32_t)v - ana_ch0.offset) * ana_ch0.scale;
  if(ana_ch0.f){
    ana_ch0.f(&ana_ch0.filter,ana_ch0.filter.x[0]);
    if(ana_ch0.filter.y[0] > 1){
      if(appParam.t.isPositive){
        appParam.t.t_pos += ana_ch0.filter.y[0];
        appParam.t.n_pos++;
      }else{
        if(appParam.t.n_pos >0)
          appParam.t.t_total = appParam.t.t_pos/appParam.t.n_pos;
        if(appParam.t.n_neg >0)
          appParam.t.t_total += appParam.t.t_neg/appParam.t.n_neg;
        appParam.t.t_pos = appParam.t.t_neg = 0;
        appParam.t.n_pos = appParam.t.n_neg = 0;
        appParam.t.t_pos = ana_ch0.filter.y[0];
        appParam.t.n_pos = 1;
        appParam.t.isPositive = true;
        appParam.t.isNegative = false;
      }
    }
    else if(ana_ch0.filter.y[0] < 1){
      if(appParam.t.isNegative){
        appParam.t.t_neg += ana_ch0.filter.y[0];
        appParam.t.n_neg++;
      }else{
        appParam.t.isPositive = false;
        appParam.t.isNegative = true;
        appParam.t.t_neg = ana_ch0.filter.y[0];
        appParam.t.n_neg =1;
      }
    }
    if(appParam.rpm > 10){
      appParam.power = (uint16_t)lround(appParam.t.t_total*appParam.rpm*1000/9549.*2.);
      appParam.torque[0] = (uint16_t)(lround(appParam.t.t_total*32*2));
    }else{
      appParam.power = 0;
      appParam.torque[0] = 0;
    }
    
  }
}

float sensorDataGetADC(uint8_t ch)
{
  return 0;
}

void validTorque()
{
  if(appParam.t.n_pos >0)
    appParam.t.t_total = appParam.t.t_pos/appParam.t.n_pos;
  if(appParam.t.n_neg >0)
    appParam.t.t_total += appParam.t.t_neg/appParam.t.n_neg;
  
  appParam.torque[0] = appParam.t.t_total*32*2;
  appParam.torque[1] = appParam.torque[0];
  appParam.power = appParam.t.t_total*appParam.rpm*1000/9549.*2.;
  appParam.t.t_pos = appParam.t.t_neg = 0;
  appParam.t.n_pos = appParam.t.n_neg = 0;
}

void sensorDataFeedIMU(uint8_t *b)
{
  _imu_data_t *d = (_imu_data_t*)b;
  _imu_data_float_t newVal;
  
  newVal.gx = (float)(d->gx - imu_gx.offset)*imu_gx.scale;
  newVal.gy = (float)(d->gy - imu_gy.offset)*imu_gy.scale;
  newVal.gz = (float)(d->gz - imu_gz.offset)*imu_gz.scale;
  newVal.ax = (float)(d->ax - imu_ax.offset)*imu_ax.scale;
  newVal.ay = (float)(d->ay - imu_ay.offset)*imu_ay.scale;
  newVal.az = (float)(d->az - imu_az.offset)*imu_az.scale;
  
  if(imu_gx.f)
    imu_gx.f(&imu_gx.filter,newVal.gx);
  if(imu_gy.f)
    imu_gy.f(&imu_gy.filter,newVal.gy);
  if(imu_gz.f)
    imu_gz.f(&imu_gz.filter,newVal.gz);
  if(imu_ax.f)
    imu_ax.f(&imu_ax.filter,newVal.ax);
  if(imu_ay.f)
    imu_ay.f(&imu_ay.filter,newVal.ay);
  if(imu_az.f)
    imu_az.f(&imu_az.filter,newVal.az);
  
  if(imu_fusion){
    appParam.sampleTime += appParam.imt_sample_interval;
    //appParam.period_cnt++;

    imu_fusion(imu_gx.filter.y[0],imu_gy.filter.y[0],imu_gz.filter.y[0],imu_ax.filter.y[0],imu_ay.filter.y[0],imu_az.filter.y[0]);
    
    appParam.imu.time += appParam.imt_sample_interval;
    if(imu_ax.filter.y[0] > 0.1){
      if(appParam.imu.isMax){
        if(imu_ax.filter.y[0] > appParam.imu.val){
          appParam.imu.val = imu_ax.filter.y[0];
        }else{ // max found
          appParam.imu.isMax = false;
          appParam.imu.isMin = true;
          if(appParam.imu.time > 240){
            appParam.period = lround(appParam.imu.time * 2.048);
            appParam.rpm = 60000/appParam.imu.time;
          }
          appParam.imu.time = 0;
        }
      }else{
        appParam.imu.isMax = true; // set to find max
        appParam.imu.val = imu_ax.filter.y[0];
      }
    }
    else if(imu_ax.filter.y[0] < 0.1){
      if(appParam.imu.isMin){
        if(imu_ax.filter.y[0] < appParam.imu.val){
          appParam.imu.val = imu_ax.filter.y[0];          
        }else{
          appParam.imu.isMax = true;
          appParam.imu.isMin = false;
        }
      }else{
        appParam.imu.isMin = true;
        appParam.imu.val = imu_ax.filter.y[0];          
      }
    }
//    if(appParam.sampleTime == 100){
//      // update value to normalized one
//      float ax = 2*(q1*q3 - q0*q2);
//      //float ax = 2*(q1*q3-q0*q2)(1-2*(q1*q1+q2*q2));
//      if(ax > 1) ax = 1;
//      if(ax < -1) ax = -1;
//      appParam.pitch = -asinf(ax);
//      appParam.dTorque = appParam.pitch_prev - appParam.pitch;
//      appParam.pitch_prev = appParam.pitch;
//      ax = 60.*appParam.dTorque*10./6.28;
//      if(ax < 0) ax = 0;
//      if(ax < 300){
//        m_rpm.f(&m_rpm.filter,ax<5?0:ax);
//        appParam.rpm = m_rpm.filter.y[0]>10?m_rpm.filter.y[0]:0;
//        appParam.period = lround(60./appParam.rpm)<<11;
//      }
//      appParam.sampleTime = 0;
//    }
  }
     
}

void sensorDataInit(uint16_t imuRate)
{
  float x = 0.5,y=0.8;

  m_imuSampleInterval = 1000./(float)imuRate;
  appParam.imt_sample_interval = 1000./(float)imuRate;
  
    if(moduleParam.torqueRatio == 0.)
    moduleParam.torqueRatio = 58;

  appParam.t.t_pos = 0.;
  appParam.t.t_neg = 0.;
  appParam.t.t_total = 0.;
  appParam.t.valid = 0;
  appParam.t.isPositive = false;
  appParam.t.isNegative = false;
  
  m_rpm.offset = 0.;
  m_rpm.scale = 1.0;
  m_rpm.filter.a[0] = 0.01;
  m_rpm.filter.b[0] = 1 - m_rpm.filter.a[0];
  m_rpm.filter.y[0] = 0;
  
  ana_ch0.offset = moduleParam.adcOffset[0];
  ana_ch0.offset <<=8;
  ana_ch0.offset +=0x800000;
  ana_ch0.scale = 1./8388608./128.*1000;
  ana_ch0.scale *= -moduleParam.torqueRatio; // convert to N-m
  ana_ch0.filter.a[0] = x;
  ana_ch0.filter.b[0] = 1-x;
  
  m_pitch.offset = 0;
  m_pitch.scale = 180./3.14159; // convert to degree
  m_pitch.filter.a[0] = x;
  m_pitch.filter.b[0] = 1-x;
  
  
  
  
  imu_ax.offset = 0;
  imu_ay.offset = 0;
  imu_az.offset = 0;
  imu_gx.offset = 0;
  imu_gy.offset = 0;
  imu_gz.offset = 0;

  imu_ax.scale = 4.0/65536.0;
  imu_ay.scale = 4.0/65536.0;
  imu_az.scale = 4.0/65536.0;
  
  imu_gx.scale = 4000.0/65536.0;
  imu_gy.scale = 4000.0/65536.0;
  imu_gz.scale = 4000.0/65536.0;
  
  imu_ax.filter.a[0] = x;
  imu_ax.filter.b[0] = 1-x;
  imu_ay.filter.a[0] = x;
  imu_ay.filter.b[0] = 1-x;
  imu_az.filter.a[0] = x;
  imu_az.filter.b[0] = 1-x;
  
  imu_gx.filter.a[0] = (1+y)/2.;
  imu_gx.filter.a[1] = -(1+y)/2.;
  imu_gx.filter.b[0] = 1-y;

  imu_gy.filter.a[0] = (1+y)/2.;
  imu_gy.filter.a[1] = -(1+y)/2.;
  imu_gy.filter.b[0] = 1-y;

  imu_gz.filter.a[0] = (1+y)/2.;
  imu_gz.filter.a[1] = -(1+y)/2.;
  imu_gz.filter.b[0] = 1-y;
  
  imu_fusion = MahonyAHRSupdateIMU;
  
  appParam.imu.isMax = false;
  appParam.imu.isMin = false;
}