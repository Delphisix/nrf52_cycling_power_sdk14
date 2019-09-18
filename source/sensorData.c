#include <stdio.h>
#include <string.h>
#include <math.h>
#include "nrf.h"
#include "mahonyAHRS.h"
#include "sensorData.h"
#include "sysParams.h"
#include "nrf_pwr_mgmt.h"

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
//ANALOG_LPF_DEF(m_pitch);
static float m_pitchDeg,m_pitchDegPrev;;
static float m_imuSampleInterval;       // in second
static float accumulated_pitch_angle;
static float accumulated_time;

#define PITCH_HISTORY_SZ        8
static float pitchHistory[PITCH_HISTORY_SZ]; 
static uint8_t pitchHisIndex = 0;

void sensorDataFeedADC(uint8_t ch, uint32_t v)
{
  //ana_ch0.filter.x[0] = ((int32_t)v - ana_ch0.offset) * ana_ch0.scale;
  if(ana_ch0.f){
    ana_ch0.f(&ana_ch0.filter,((int32_t)v - ana_ch0.offset) * ana_ch0.scale);
    if(ana_ch0.filter.y[0] > 1){
      if(appParam.t.isPositive){
        appParam.t.t_pos += ana_ch0.filter.y[0];
        appParam.t.n_pos++;
      }else{
        float t = 0;
        uint16_t n = appParam.t.n_pos + appParam.t.n_neg;
        if(n){
          t = (appParam.t.t_pos + appParam.t.t_neg)/n;
        }
//        if(appParam.t.n_pos >0)
//          t = appParam.t.t_pos/appParam.t.n_pos;
//        if(appParam.t.n_neg >0)
//          t += appParam.t.t_neg/appParam.t.n_neg;
        appParam.tHis.his[appParam.tHis.index++] = t;
        if(appParam.tHis.index == appParam.tHis.nofElement)
          appParam.tHis.index = 0;
        
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
    
//    if((appParam.t.n_pos > 60) || ())
    
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
  
//  if(imu_gx.f)
//    imu_gx.f(&imu_gx.filter,newVal.gx);
//  if(imu_gy.f)
//    imu_gy.f(&imu_gy.filter,newVal.gy);
//  if(imu_gz.f)
//    imu_gz.f(&imu_gz.filter,newVal.gz);
//  if(imu_ax.f)
//    imu_ax.f(&imu_ax.filter,newVal.ax);
//  if(imu_ay.f)
//    imu_ay.f(&imu_ay.filter,newVal.ay);
//  if(imu_az.f)
//    imu_az.f(&imu_az.filter,newVal.az);
  
  if(imu_fusion){
    appParam.sampleTime += appParam.imt_sample_interval;
    //appParam.period_cnt++;

//    imu_fusion(imu_gx.filter.y[0],imu_gy.filter.y[0],imu_gz.filter.y[0],imu_ax.filter.y[0],imu_ay.filter.y[0],imu_az.filter.y[0],appParam.imu.sampleIntervalMs);
//    imu_fusion(0,0,0,imu_ax.filter.y[0],imu_ay.filter.y[0],imu_az.filter.y[0],appParam.imu.sampleIntervalMs);
    //imu_fusion(0,0,0,newVal.ax,newVal.ay,newVal.az,appParam.imu.sampleIntervalMs);
    
    appParam.imu.time += appParam.imu.sampleIntervalMs;
    
    //if(appParam.imu.time >= 0.1){
      float gx = 2*(q1*q3 - q0*q2);
      //float gy = 2*(q0*q1 + q2*q3);
      //float gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
      
      if(gx > 1) gx = 1;
      if(gx < -1) gx = -1;
      if(newVal.ax > 1) newVal.ax = 1;
      if(newVal.ax < -1) newVal.ax = -1;
      float rad = -asinf(newVal.ax);
      float dr = rad - appParam.imu.prevRad;
      int32_t *dp = (int32_t*)&dr;
      (*dp) &= 0x7fffffff;
      appParam.imu.prevRad = rad;
      appParam.imu.radSum += dr;
      
//      if(appParam.imu.time >= 0.5){
//        float r = appParam.imu.radSum*60/(appParam.imu.time)/6.28;
//        m_rpm.f(&m_rpm.filter,r);
//        appParam.rpm = m_rpm.filter.y[0]>10?m_rpm.filter.y[0]:0;
//        if(appParam.rpm > 10){
//          appParam.period_ms = 60./appParam.rpm *1000;
//          appParam.period = lround(appParam.period_ms*2.048);
//          appParam.rpm_idle_seconds = 0;
//        }else{
//          appParam.period = 0;
//          if(appParam.rpm_idle_seconds > moduleParam.idleTimeout){
//            // enter sleep mode
//            if(appParam.state == ST_IDLE || appParam.state == ST_ADVING)
//              nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_STAY_IN_SYSOFF);
//          }
//        }
//        appParam.imu.radSum = 0;
//        appParam.imu.time = 0;
//      }
      
//      if(appParam.imu.count < 8){
////        appParam.imu.history[appParam.imu.count] = -asinf(newVal.ax) ;
//        appParam.imu.history[appParam.imu.count] = -asinf(gx) ;
//        appParam.imu.count++;
//        //appParam.period_ms=3.1;
//      }
//      else{
//        bool pos = true;
//        bool neg = true;
//        float da,sum_da = 0;
//        int32_t *dp = (int32_t*)&da;
//        uint8_t cnts = 0;
//        for(int8_t i=0;i<7;i++){
//          da = appParam.imu.history[i+1] - appParam.imu.history[i];
//          (*dp) &= 0x7fffffff;
//          if(da < 1){
//            sum_da += da;
//            cnts++;
//          }
//          appParam.imu.history[i] = appParam.imu.history[i+1];
//        }
//        appParam.imu.history[7] = -asin(newVal.ax);
//        
//        if(cnts){
//          float r = sum_da*60/(appParam.imu.sampleIntervalMs*cnts)/6.28;
//          m_rpm.f(&m_rpm.filter,r);
//          appParam.rpm = m_rpm.filter.y[0]>10?m_rpm.filter.y[0]:0;
//          if(appParam.rpm > 10){
//            appParam.period_ms = 60./appParam.rpm *1000;
//            appParam.period = lround(appParam.period_ms*2.048);
//            appParam.rpm_idle_seconds = 0;
//          }else{
//            appParam.period = 0;
////            appParam.rpm_zero_count++;
//            if(appParam.rpm_idle_seconds > moduleParam.idleTimeout){
//              // enter sleep mode
//              if(appParam.state == ST_IDLE || appParam.state == ST_ADVING)
//                nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_STAY_IN_SYSOFF);
//            }
//          }
//        }
//        appParam.imu.count = 0;
//      }
  }
     
}

void sensorDataInit(uint16_t imuRate)
{
  float x = 0.2,y=0.05;

  m_imuSampleInterval = 1000./(float)imuRate;
  appParam.imu.sampleIntervalMs = 1./(float)imuRate;
  appParam.imt_sample_interval = 1000./(float)imuRate;
  
    if(moduleParam.torqueRatio[0] == 0.)
    moduleParam.torqueRatio[0] = 58;

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
  ana_ch0.scale *= moduleParam.torqueRatio[0]; // convert to N-m
  ana_ch0.filter.a[0] = x;
  ana_ch0.filter.b[0] = 1-x;
  
//  m_pitch.offset = 0;
//  m_pitch.scale = 180./3.14159; // convert to degree
//  m_pitch.filter.a[0] = x;
//  m_pitch.filter.b[0] = 1-x;
  
  
  
  
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
  
  appParam.imu.isPos = false;
  appParam.imu.isNeg = false;
  
  appParam.rpm_zero_count = 0;
}