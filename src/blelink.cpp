// blelink.cpp - RP2350 side of the UART link to the ESP32-C3 BLE bridge.
// Protocol: mf_ble_proto.h.  Not compiled on the build machine used to write it: the
// shared headers are host-tested, this file was checked against the madflight sources.

#include "madflight_modules.h"   // alt, ahr, bar, bat, gps, hal, MsgSubscription (same header cli.cpp uses)
#include "mf_ble_proto.h"
#include "blelink.h"
#include <math.h>

BleLinkStatus blelink_status;

// ---------------------------------------------------------------- configuration
#ifndef LINK_SER_BUS
#define LINK_SER_BUS 1                 // madflight serial bus 1 = pin_ser1_tx / pin_ser1_rx
#endif
#define LINK_STATE_HZ 20               // STATE frame rate. The ESP32 decides what actually goes out over BLE.
#define LINK_HOOK_MAX_AGE_MS 2000      // hook values older than this are reported as "no data"
#define LINK_INFO_PERIOD_MS 5000       // INFO is repeated so an ESP32 reset recovers on its own

// Heading must be relative to TRUE north, the AHRS gives MAGNETIC north (and indoors it
// is meaningless). Set the local declination (east positive). 0 = uncorrected.
#ifndef LINK_MAG_DECLINATION_DEG
#define LINK_MAG_DECLINATION_DEG 0.0f
#endif

#ifndef LINK_MANUFACTURER
#define LINK_MANUFACTURER "DIY"
#endif
#ifndef LINK_MODEL
#define LINK_MODEL "WindVario"
#endif
#ifndef LINK_SERIAL
#define LINK_SERIAL "0001"             // set per unit
#endif
#ifndef LINK_HARDWARE
#define LINK_HARDWARE "FC3v2-proto"
#endif

// ---------------------------------------------------------------- hook storage
// Plain scalars written by one task and read by another. Each float is individually valid
// even under a rare torn read - the same tolerance madflight relies on elsewhere.
static volatile float s_tas = NAN, s_ias = NAN, s_netto = NAN, s_tek = NAN;
static volatile uint32_t s_air_ms = 0;
static volatile float s_wind_speed = NAN, s_wind_from = NAN;
static volatile uint32_t s_wind_ms = 0;
static volatile float s_rh = NAN;
static volatile uint32_t s_rh_ms = 0;

void blelink_set_airdata(float tas_ms, float ias_ms, float netto_ms, float tek_ms) {
  s_tas = tas_ms; s_ias = ias_ms; s_netto = netto_ms; s_tek = tek_ms;
  s_air_ms = millis();
}
void blelink_set_wind(float speed_ms, float from_deg) {
  s_wind_speed = speed_ms; s_wind_from = from_deg;
  s_wind_ms = millis();
}
void blelink_set_humidity(float rh_percent) {
  s_rh = rh_percent;
  s_rh_ms = millis();
}

// ---------------------------------------------------------------- helpers
static bool fresh(uint32_t stamp_ms, uint32_t now) {
  return stamp_ms != 0 && (uint32_t)(now - stamp_ms) < LINK_HOOK_MAX_AGE_MS;
}

// Rough 1S LiPo state of charge from resting voltage. Under load the voltage sags and this
// reads low. It also assumes the INA226 sees the CELL voltage - if it sees the 5 V boost
// rail instead, this will always say 100 %, so check what your bat.v actually reports.
static uint8_t lipo_percent(float v) {
  static const float tv[10] = { 4.20f, 4.10f, 4.00f, 3.92f, 3.85f, 3.80f, 3.75f, 3.70f, 3.60f, 3.30f };
  static const float tp[10] = { 100,   90,    75,    60,    50,    40,    25,    15,    5,     0 };
  if (v >= tv[0]) return 100;
  for (int i = 1; i < 10; i++) {
    if (v >= tv[i]) {
      float f = (v - tv[i]) / (tv[i - 1] - tv[i]);
      return (uint8_t)lroundf(tp[i] + f * (tp[i - 1] - tp[i]));
    }
  }
  return 0;
}

static void send_frame(MF_Serial* ser, uint8_t type,
                       const void* payload, size_t len) {
  uint8_t buf[MF_MAX_FRAME];
  size_t n = mf_frame_encode(buf, type, payload, len);
  if (!n) return;

  int written = ser->write(buf, (int)n);
  if (written != (int)n) {
    Serial.printf("[LINK] TX failed: type=%02X len=%u written=%d\n",
                  type, (unsigned)n, written);
  }
    //Debug
  /*Serial.print("[TX] ");
  for (size_t i = 0; i < n; i++) {
    Serial.printf("%02X ", buf[i]);
  }
  Serial.println();*/
}

static void send_info(MF_Serial* ser) {
  struct Item { uint8_t id; const char* text; };
  const Item items[] = {
    { MF_INFO_MANUFACTURER, LINK_MANUFACTURER },
    { MF_INFO_MODEL,        LINK_MODEL },
    { MF_INFO_SERIAL,       LINK_SERIAL },
    { MF_INFO_FIRMWARE,     MADFLIGHT_VERSION },
    { MF_INFO_HARDWARE,     LINK_HARDWARE },
  };
  for (const Item& it : items) {
    uint8_t p[1 + 40];
    size_t n = strlen(it.text);
    if (n > 40) n = 40;
    p[0] = it.id;
    memcpy(p + 1, it.text, n);
    send_frame(ser, MF_MSG_INFO, p, 1 + n);
  }
}

static void fill_state(MfState& s) {
  memset(&s, 0, sizeof(s));
  const uint32_t now = millis();
  s.ts_ms = now;
  uint32_t valid = 0;

  const bool air_fresh = fresh(s_air_ms, now);
  const float climb = alt.getV();   // ground-relative climb rate from the altitude Kalman filter

  // Vario: total-energy if the air-data producer supplies it, otherwise classic.
  // vario_mode always tells the truth about what the number is.
  const float tek = s_tek;
  if (air_fresh && __builtin_isfinite(tek)) { s.vario = tek;   s.vario_mode = 1; }
  else                            { s.vario = climb; s.vario_mode = 0; }
  valid |= MF_V_VARIO;

  if (air_fresh) {
    const float tas = s_tas, ias = s_ias, netto = s_netto;
    if (__builtin_isfinite(tas))   { s.tas = tas;     valid |= MF_V_TAS; }
    if (__builtin_isfinite(ias))   { s.ias = ias;     valid |= MF_V_IAS; }
    if (__builtin_isfinite(netto)) { s.netto = netto; valid |= MF_V_NETTO; }
  }

  if (fresh(s_wind_ms, now)) {
    const float ws = s_wind_speed, wd = s_wind_from;
    if (__builtin_isfinite(ws) && __builtin_isfinite(wd)) {
      s.wind_speed = ws;
      s.wind_from = fmodf(fmodf(wd, 360.0f) + 360.0f, 360.0f);
      valid |= MF_V_WIND;
    }
  }

  // Attitude (madflight: roll right positive, pitch up positive, yaw -180..180 clockwise).
  float h = fmodf(ahr.yaw + LINK_MAG_DECLINATION_DEG + 360.0f, 360.0f);
  if (h < 0) h += 360.0f;
  s.heading = h;  valid |= MF_V_HEADING;
  s.roll  = ahr.roll;   valid |= MF_V_ROLL;
  s.pitch = ahr.pitch;  valid |= MF_V_PITCH;

  // Specific-force magnitude in g (1.0 at rest, whatever the orientation).
  s.load_factor = sqrtf(ahr.ax * ahr.ax + ahr.ay * ahr.ay + ahr.az * ahr.az);
  valid |= MF_V_LOAD;

  // Ground glide ratio = ground speed / sink rate. >0 descending, <0 climbing.
  // Needs a 3D fix and some ground speed; clamped so a level glide doesn't give infinity.
  if (gps.fix >= GPS_OK_FIX_3D) {
    const float sog = gps.sog * 0.001f;
    if (sog > 1.0f) {
      float sink = -climb;
      if (fabsf(sink) < 0.05f) sink = (sink < 0) ? -0.05f : 0.05f;
      float gr = sog / sink;
      if (gr > 99.9f) gr = 99.9f;
      if (gr < -99.9f) gr = -99.9f;
      s.glide_ratio = gr;
      valid |= MF_V_GLIDE;
    }
  }

  // Environment. Static pressure is the raw barometer value for now.
  if (bar.press > 1000.0f) {
    s.static_pressure = bar.press; valid |= MF_V_PRESSURE;
    s.air_temp = bar.temp;         valid |= MF_V_TEMP;
  }
  if (fresh(s_rh_ms, now) && __builtin_isfinite(s_rh)) { s.humidity = s_rh; valid |= MF_V_HUMIDITY; }

  if (bat.v > 2.5f) { s.battery_pct = lipo_percent(bat.v); valid |= MF_V_BATTERY; }

  s.valid = valid;
}

static void fill_gps(MfGps& m, const GpsState& g) {
  m.fix = (uint8_t)g.fix;
  m.sat = g.sat;
  m.hdop = g.hdop;
  m.gps_week = g.time_week;
  m.tow_ms = g.time;
  m.lat_e7 = g.lat;
  m.lon_e7 = g.lon;
  m.alt_mm = g.alt;
  m.sog_mms = g.sog;
  m.cog_e5 = g.cog;
}

static void handle_rx(const MfParser& p) {
  if (p.type == MF_MSG_STATUS && p.len >= sizeof(MfStatus)) {
    MfStatus st; memcpy(&st, p.payload, sizeof(st));
    const bool was_connected = blelink_status.connected;
    blelink_status.connected = st.ble_connected;
    blelink_status.mtu = st.mtu;
    blelink_status.rx_ok = st.rx_ok;
    blelink_status.rx_err = st.rx_err;
    blelink_status.last_rx_ms = millis() | 1;   // never 0, so 0 can mean "never heard"
    if (st.ble_connected != was_connected) {
      Serial.printf("[LINK] BLE %s (mtu %u)\n", st.ble_connected ? "connected" : "disconnected", st.mtu);
    }
  }
}

// ---------------------------------------------------------------- task
static void blelink_task(void* pvParameters) {
  (void)pvParameters;

  MF_Serial* ser = hal_get_ser_bus(LINK_SER_BUS, MF_LINK_BAUD);
  if (!ser) {
    Serial.println("[LINK] serial bus unavailable - check pin_ser1_tx / pin_ser1_rx in madflight_config");
    vTaskDelete(NULL);
    return;
  }
  Serial.printf("[LINK] started on serial bus %d, %d baud\n", LINK_SER_BUS, MF_LINK_BAUD);

  MsgSubscription<GpsState> gps_sub("blelink_gps", &gps.topic);
  MfParser parser;
  uint32_t next_info = 0;
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / LINK_STATE_HZ);

  for (;;) {
    // 1. whatever the ESP32 has said
    while (ser->available() > 0) {
      int b = ser->read();
      if (b < 0) break;
      if (parser.feed((uint8_t)b)) handle_rx(parser);
    }

    // 2. identity strings, now and then
    const uint32_t now = millis();
    if ((int32_t)(now - next_info) >= 0) {
      send_info(ser);
      next_info = now + LINK_INFO_PERIOD_MS;
    }
    
    // 3. state
    MfState st;
    fill_state(st);
    send_frame(ser, MF_MSG_STATE, &st, sizeof(st));

    // 4. GPS, only when a new fix has actually arrived (pull_latest is false otherwise)
    GpsState g;
    if (gps_sub.pull_latest(&g)) {
      MfGps m;
      fill_gps(m, g);
      send_frame(ser, MF_MSG_GPS, &m, sizeof(m));
    }

    vTaskDelayUntil(&last_wake, period);   // fixed cadence, same idiom as wind_task
  }
}

void blelink_task_start() {
  hal_xTaskCreate(blelink_task, "mf_LINK", MF_FREERTOS_DEFAULT_STACK_SIZE * 2, NULL, uxTaskPriorityGet(NULL), NULL, 0);
}