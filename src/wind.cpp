// wind.cpp - Pressure-probe (5-hole pitot) module for madflight
// Phase 1: raw per-sensor read + publish. No rotation/calibration math yet.

#include <Arduino.h>
#include "wind.h"

//create global module instance
Wind wind;

//SDP3x commands (see datasheet section "Start Continuous Measurement")
#define SDP3X_CMD_START_DP_AVG 0x3615  //differential pressure, average till read
#define SDP3X_CMD_SOFT_RESET   0x0006  //general call soft reset (optional, not used per-address here)

//Sensirion common CRC-8: polynomial 0x31, init 0xFF, no reflect
static uint8_t sdp3x_crc8(const uint8_t *data, int len) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

int Wind::setup() {
  if (!config.i2c_bus) return -1;

  config.i2c_bus->setClock(1000000); //1MHz - I2C1 is dedicated to this sensor array

  uint8_t adrs[3] = {config.i2c_adr_center, config.i2c_adr_topbot, config.i2c_adr_leftright};
  bool ok = true;
  for (int i = 0; i < 3; i++) {
    uint8_t cmd[2] = {(uint8_t)(SDP3X_CMD_START_DP_AVG >> 8), (uint8_t)(SDP3X_CMD_START_DP_AVG & 0xFF)};
    const int MAX_TRIES = 5;
    uint8_t rv = 1; //nonzero = not-yet-success
    int tries = 0;
    while (rv != 0 && tries < MAX_TRIES) {
      config.i2c_bus->beginTransmission(adrs[i]);
      config.i2c_bus->write(cmd, 2);
      rv = config.i2c_bus->endTransmission(true);
      tries++;
      if (rv != 0) delay(2); //short backoff before retry
    }
    if (rv != 0) {
      Serial.printf("[WIND] start command failed for adr=0x%02X after %d tries (rv=%d)\n", adrs[i], tries, rv);
      ok = false;
    } else if (tries > 1) {
      Serial.printf("[WIND] start command for adr=0x%02X needed %d tries\n", adrs[i], tries);
    }
  }

  delayMicroseconds(8000); //datasheet: allow >= a few ms before first read after start command

  ts = micros();
  dt = 0;
  return ok ? 0 : -2;
}

// Reads one SDP3x: 9 bytes = dp(2)+crc(1) + temp(2)+crc(1) + scale(2)+crc(1)
// Scale factor is read live rather than hardcoded, per datasheet.
bool Wind::readSensor(uint8_t adr, SdpRaw *out) {
  uint8_t buf[9];
  uint32_t n = config.i2c_bus->requestFrom(adr, 9, true);
  if (n != 9) {
    out->valid = false;
    return false;
  }
  config.i2c_bus->read(buf, 9);

  if (sdp3x_crc8(&buf[0], 2) != buf[2]) { out->valid = false; return false; }
  if (sdp3x_crc8(&buf[3], 2) != buf[5]) { out->valid = false; return false; }
  if (sdp3x_crc8(&buf[6], 2) != buf[8]) { out->valid = false; return false; }

  int16_t dp_raw   = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t temp_raw = (int16_t)((buf[3] << 8) | buf[4]);
  int16_t scale    = (int16_t)((buf[6] << 8) | buf[7]);

  if (scale == 0) { out->valid = false; return false; } //avoid div by zero

  out->dp = (float)dp_raw / (float)scale;   //[Pa]
  out->temp = (float)temp_raw / 200.0f;     //[C], fixed scale factor per datasheet
  out->valid = true;
  return true;
}

bool Wind::update() {
  bool ok = true;
  ok &= readSensor(config.i2c_adr_center, &center);
  ok &= readSensor(config.i2c_adr_topbot, &topbot);
  ok &= readSensor(config.i2c_adr_leftright, &leftright);

  uint32_t now = micros();
  dt = (now - ts) / 1000000.0f;
  ts = now;

  topic.publish(this); //publish even on partial failure - SdpRaw.valid flags show what to trust

  return ok;
}