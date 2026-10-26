// blelink.h - RP2350 side of the UART link to the ESP32-C3 BLE bridge (see mf_ble_proto.h).
// Replaces the earlier HC-42 ble.h / ble.cpp - delete those, they use UART0 which is now the GPS.
//
// Wiring (3.3 V logic, common GND):
//   RP2350 GPIO24 (UART1 TX)  ->  ESP32-C3 GPIO4 (RX)
//   RP2350 GPIO25 (UART1 RX)  <-  ESP32-C3 GPIO3 (TX)
// madflight_config needs:   pin_ser1_tx 24    pin_ser1_rx 25

#pragma once
#include <stdint.h>

// Starts the link task on core 0. Call once from setup(), after madflight_setup().
void blelink_task_start();

// ---- Hooks for the code that computes the air data (wind.cpp Phase 2, GPS wind vector) ----
// All SI units. Pass NAN for a value you don't have: that field is then reported as "no
// data" instead of a fake zero. Values older than ~2 s are dropped automatically, so a
// stalled producer can't leave stale numbers on the phone.

// tek_ms: total-energy-compensated vario. When it is fresh the link reports vario_mode=TEK,
//         otherwise it falls back to the classic vario from the altitude Kalman filter.
// netto_ms: vertical air movement (your "vertical wind").
void blelink_set_airdata(float tas_ms, float ias_ms, float netto_ms, float tek_ms);

// from_deg: direction the wind blows FROM, 0..360 (meteorological convention).
void blelink_set_wind(float speed_ms, float from_deg);

// For when a humidity sensor exists. Until then it is simply never valid.
void blelink_set_humidity(float rh_percent);

// What the ESP32 last told us (1 Hz). Read it from loop() if you want to print it.
struct BleLinkStatus {
  volatile uint8_t  connected = 0;   // a BLE central (XCTrack) is connected
  volatile uint16_t mtu = 0;         // negotiated ATT MTU - the GPS characteristic needs >= 186
  volatile uint16_t rx_ok = 0;
  volatile uint16_t rx_err = 0;
  volatile uint32_t last_rx_ms = 0;  // millis() of the last STATUS frame, 0 = ESP32 never answered
};
extern BleLinkStatus blelink_status;
