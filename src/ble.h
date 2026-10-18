// ble.h - HC-42 BLE telemetry over UART1 (GPIO24 TX -> HC-42 RX, GPIO25 RX <- HC-42 TX).
// UART0 (GPIO12/13) is the GPS receiver. Power the HC-42 from the 3.3V rail.
//
// Sends, over the Nordic UART Service the HC-42 exposes by default:
//   $LK8EX1  10 Hz  altitude + vario + temp + battery
//   $LXWP0    2 Hz  TAS + heading + wind
//   $XCTOD    2 Hz  netto + wind as plain text for XCTrack's "External data" widget
//   $GNRMC    5 Hz  position, ground speed, course, date/time   (from madflight's gps module)
//   $GNGGA    5 Hz  position, fix quality, satellites, HDOP, altitude MSL
// Any value left as NAN is sent as an empty field ("not available"), never as 0.

#pragma once
#include <math.h>

struct BleData {
  float tas_ms        = NAN; // true airspeed [m/s]
  float te_vario_ms   = NAN; // total energy vario [m/s]; NAN -> LK8EX1 falls back to alt.getV()
  float netto_ms      = NAN; // vertical air velocity [m/s] (the pitot-derived vertical wind)
  float wind_from_deg = NAN; // direction the wind comes FROM [deg] - verify convention in the field
  float wind_speed_ms = NAN; // horizontal wind speed [m/s]
};

// Written by the wind/vario code, read by ble_task. Independent scalars, so an
// unlocked read can at worst mix two adjacent samples - fine for telemetry.
extern BleData ble_data;

void ble_task(void *pvParameters);
void ble_task_start();