// mf_ble_proto.h - UART link between the RP2350 (madflight) and the ESP32-C3 BLE bridge.
//
// One header, included unchanged by BOTH firmwares (copy it into each project).
// Pure C++11, no Arduino or madflight dependency, so it can be unit tested on a PC.
//
// DESIGN RULES
//  1. The RP2350 speaks SI units as IEEE-754 floats (m/s, deg, Pa, degC, %).
//     The ESP32 alone owns the BLE encodings (dm/s int32, 0.01 deg uint16, ...).
//     Changing what a BLE characteristic looks like never touches the RP2350.
//  2. Every field has a bit in `valid`. A field whose bit is clear means "no data",
//     never a fake 0. (A 0 would read as "confirmed still air".)
//  3. Mostly one-way: RP2350 -> ESP32. The ESP32 answers with a 1 Hz STATUS frame so
//     the RP2350 can see that the link and the BLE connection are alive.
//  4. Little-endian everywhere. RP2350 (Cortex-M33) and ESP32-C3 (RISC-V) are both LE,
//     so the packed structs go on the wire as they are.
//
// FRAME
//   0xAA 0x55 | type u8 | len u8 | payload[len] | crc16 u16 (LE)
//   crc16 = CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over  type, len, payload.
//   len <= MF_MAX_PAYLOAD. A receiver resynchronises on the next 0xAA 0x55.
//
// MESSAGES
//   0x01 INFO   RP2350 -> ESP32  u8 field_id + ASCII text (no NUL). Sent at boot and every 5 s.
//   0x02 STATE  RP2350 -> ESP32  struct MfState, 20 Hz. Vario, netto, wind, attitude, ...
//   0x03 GPS    RP2350 -> ESP32  struct MfGps, once per new GPS fix (the ESP32 builds the NMEA).
//   0x81 STATUS ESP32 -> RP2350  struct MfStatus, 1 Hz.
//
// EXTENDING: append fields at the END of a struct and bump MF_PROTO_VERSION. A receiver
// accepts any payload with len >= sizeof(its struct) and ignores the extra tail.

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define MF_PROTO_VERSION 1
#define MF_SYNC1 0xAA
#define MF_SYNC2 0x55
#define MF_MAX_PAYLOAD 200
#define MF_MAX_FRAME (6 + MF_MAX_PAYLOAD)
#define MF_LINK_BAUD 460800   // ~1.8 kB/s of traffic, so 115200 has >5x headroom. Raise later if wanted.

enum : uint8_t {
  MF_MSG_INFO   = 0x01,
  MF_MSG_STATE  = 0x02,
  MF_MSG_GPS    = 0x03,
  MF_MSG_STATUS = 0x81,
};

// INFO field ids -> Device Information Service characteristics
enum : uint8_t {
  MF_INFO_MANUFACTURER = 1,  // 0x2A29
  MF_INFO_MODEL        = 2,  // 0x2A24
  MF_INFO_SERIAL       = 3,  // 0x2A25
  MF_INFO_FIRMWARE     = 4,  // 0x2A26
  MF_INFO_HARDWARE     = 5,  // 0x2A27
};

// Bits of MfState.valid
enum : uint32_t {
  MF_V_VARIO    = 1u << 0,   // vario + vario_mode
  MF_V_NETTO    = 1u << 1,
  MF_V_TAS      = 1u << 2,
  MF_V_IAS      = 1u << 3,
  MF_V_HEADING  = 1u << 4,
  MF_V_ROLL     = 1u << 5,
  MF_V_PITCH    = 1u << 6,
  MF_V_GLIDE    = 1u << 7,
  MF_V_LOAD     = 1u << 8,
  MF_V_WIND     = 1u << 9,   // wind_speed + wind_from together
  MF_V_PRESSURE = 1u << 10,
  MF_V_TEMP     = 1u << 11,
  MF_V_HUMIDITY = 1u << 12,
  MF_V_BATTERY  = 1u << 13,
};

#pragma pack(push, 1)

struct MfState {
  uint32_t valid;            // MF_V_* bits
  uint32_t ts_ms;            // RP2350 millis() when the frame was assembled
  float vario;               // [m/s] up. Total-energy if vario_mode==1, else classic
  float netto;               // [m/s] up. Vertical air movement
  float tas;                 // [m/s] true airspeed
  float ias;                 // [m/s] indicated airspeed
  float heading;             // [deg] 0..360, relative to TRUE north
  float roll;                // [deg] -180..180, right wing down positive
  float pitch;               // [deg] -90..90, nose up positive
  float glide_ratio;         // ground speed / sink rate. >0 descending, <0 climbing, clamped +-99.9
  float load_factor;         // [g]
  float wind_speed;          // [m/s]
  float wind_from;           // [deg] 0..360, direction the wind blows FROM
  float static_pressure;     // [Pa]
  float air_temp;            // [degC]
  float humidity;            // [% RH]
  uint8_t vario_mode;        // 0 = classic, 1 = TEK (what `vario` actually contains)
  uint8_t battery_pct;       // 0..100
};

struct MfGps {
  uint8_t  fix;              // madflight GPS_Status: 0 none, 1 no fix, 2 2D, 3 3D, 4 DGPS
  uint8_t  sat;
  uint16_t hdop;             // x100
  uint16_t gps_week;
  uint32_t tow_ms;           // GPS time of week [ms]
  int32_t  lat_e7;           // 1e-7 deg
  int32_t  lon_e7;           // 1e-7 deg
  int32_t  alt_mm;           // altitude above mean sea level [mm]
  int32_t  sog_mms;          // ground speed [mm/s]
  int32_t  cog_e5;           // course over ground [1e-5 deg]
};

struct MfStatus {            // ESP32 -> RP2350
  uint8_t  proto;            // MF_PROTO_VERSION the ESP32 was built with
  uint8_t  ble_connected;    // 1 if a BLE central is connected
  uint16_t mtu;              // negotiated ATT MTU (the GPS characteristic needs >= 186)
  uint16_t rx_ok;            // valid frames received (wraps)
  uint16_t rx_err;           // CRC failures (wraps)
};

#pragma pack(pop)

static_assert(sizeof(MfState)  == 66, "MfState layout changed - bump MF_PROTO_VERSION");
static_assert(sizeof(MfGps)    == 30, "MfGps layout changed - bump MF_PROTO_VERSION");
static_assert(sizeof(MfStatus) == 8,  "MfStatus layout changed - bump MF_PROTO_VERSION");

// CRC-16/CCITT-FALSE. Check value for "123456789" is 0x29B1.
static inline uint16_t mf_crc16(const uint8_t* d, size_t n, uint16_t crc = 0xFFFF) {
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// Writes one frame to `out` (needs MF_MAX_FRAME bytes). Returns the frame length, 0 if len is too big.
static inline size_t mf_frame_encode(uint8_t* out, uint8_t type, const void* payload, size_t len) {
  if (len > MF_MAX_PAYLOAD) return 0;
  out[0] = MF_SYNC1; out[1] = MF_SYNC2; out[2] = type; out[3] = (uint8_t)len;
  if (len) memcpy(out + 4, payload, len);
  uint16_t crc = mf_crc16(out + 2, 2 + len);
  out[4 + len] = (uint8_t)(crc & 0xFF);
  out[5 + len] = (uint8_t)(crc >> 8);
  return 6 + len;
}

// Byte-at-a-time frame parser. feed() returns true when a complete, CRC-valid frame is
// available in type / len / payload. Those stay valid until the next call to feed().
class MfParser {
 public:
  uint8_t  type = 0;
  uint8_t  len = 0;
  uint8_t  payload[MF_MAX_PAYLOAD];
  uint32_t ok = 0;        // frames accepted
  uint32_t crc_err = 0;   // frames rejected by CRC

  bool feed(uint8_t b) {
    switch (st_) {
      case S1:   if (b == MF_SYNC1) st_ = S2; break;
      case S2:   if (b == MF_SYNC2) st_ = TYPE; else if (b != MF_SYNC1) st_ = S1; break;
      case TYPE: type = b; st_ = LEN; break;
      case LEN:
        len = b;
        if (len > MF_MAX_PAYLOAD) { st_ = S1; break; }   // corrupt length - resync
        idx_ = 0;
        st_ = len ? PAY : CRC1;
        break;
      case PAY:  payload[idx_++] = b; if (idx_ >= len) st_ = CRC1; break;
      case CRC1: crc_rx_ = b; st_ = CRC2; break;
      case CRC2: {
        crc_rx_ |= (uint16_t)b << 8;
        uint8_t hdr[2] = { type, len };
        uint16_t c = mf_crc16(hdr, 2);
        c = mf_crc16(payload, len, c);
        st_ = S1;
        if (c == crc_rx_) { ok++; return true; }
        crc_err++;
        break;
      }
    }
    return false;
  }

 private:
  enum State : uint8_t { S1, S2, TYPE, LEN, PAY, CRC1, CRC2 };
  State    st_ = S1;
  uint8_t  idx_ = 0;
  uint16_t crc_rx_ = 0;
};