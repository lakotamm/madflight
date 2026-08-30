// wind.h - Pressure-probe (5-hole pitot) module for madflight
// Follows the same Config/State/module pattern as bar.h and gps.h.
// Phase 1 scope: raw per-sensor differential pressure + temperature only.
// Phase 2 (later): add alpha, beta, q_true, tas, vwind fields once the
// Cp(alpha)/Cp(beta)/Cp(q) calibration curves and rotation math are ready.

#pragma once

#include "./hal/MF_I2C.h"
#include "./tbx/tbx.h" //MsgTopic

// SDP3x I2C addresses (set via ADDR pin resistor, see datasheet)
#ifndef WIND_I2C_ADR_CENTER
  #define WIND_I2C_ADR_CENTER 0x21 //SDP32, top-vs-bottom -> alpha channel
#endif
#ifndef WIND_I2C_ADR_TOPBOT
  #define WIND_I2C_ADR_TOPBOT 0x22 //SDP31, total-vs-static -> dynamic pressure 
#endif
#ifndef WIND_I2C_ADR_LEFTRIGHT
  #define WIND_I2C_ADR_LEFTRIGHT 0x23 //SDP32, left-vs-right -> beta channel
#endif

//raw sensor data (Phase 1) - one struct per physical SDP3x
struct SdpRaw {
  float dp = 0;     //differential pressure [Pa], NAN if read failed
  float temp = 0;   //temperature [C]
  bool valid = false;
};

struct __attribute__((aligned(4))) WindState {
public:
  SdpRaw center;   //total-vs-static (dynamic pressure channel)
  SdpRaw topbot;   //top-vs-bottom (alpha channel)
  SdpRaw leftright;//left-vs-right (beta channel)
  uint32_t ts = 0;  //sample timestamp [us]
  float dt = 0;     //time since last sample [s]

  //--- Phase 2 fields, added once calibration/rotation math is in place ---
  //float alpha = 0;   //angle of attack [deg]
  //float beta = 0;    //sideslip [deg]
  //float q_true = 0;  //corrected dynamic pressure [Pa]
  //float tas = 0;     //true airspeed [m/s]
  //float vwind = 0;   //vertical wind [m/s]
};

struct WindConfig {
public:
  MF_I2C *i2c_bus = nullptr;
  uint8_t i2c_adr_center = WIND_I2C_ADR_CENTER;
  uint8_t i2c_adr_topbot = WIND_I2C_ADR_TOPBOT;
  uint8_t i2c_adr_leftright = WIND_I2C_ADR_LEFTRIGHT;
};

class Wind : public WindState {
public:
  WindConfig config;
  MsgTopic<WindState> topic = MsgTopic<WindState>("wind");

  int setup();    //sends start-continuous-measurement command to all 3 sensors. returns 0 on success.
  bool update();  //reads all 3 sensors, publishes to topic. returns true if all 3 reads succeeded.
  bool installed() { return (config.i2c_bus != nullptr); }

private:
  bool readSensor(uint8_t adr, SdpRaw *out);
};

//Global module instance
extern Wind wind;
