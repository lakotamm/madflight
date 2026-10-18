#include "madflight_modules.h" // NOT madflight.h - that one may only be included once, in main.cpp
#include <math.h>
#include "ble.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>

BleData ble_data;

static uint32_t ble_tx_failures = 0;
static uint32_t ble_format_errors = 0;
static uint32_t ble_tx_bytes = 0;


// GPS-UTC offset. GpsState.time is GPS time; NMEA wants UTC. This has been 18 s
// since the 2016-12-31 leap second - revisit if another one is ever announced.
#define GPS_UTC_LEAP_SECONDS 18

// Standard NMEA checksum: XOR all bytes between '$' and '*' (or the terminating NUL).
static uint8_t nmea_checksum(const char *s) {
  uint8_t cs = 0;

  if (s == nullptr || *s != '$') {
    return 0;
  }

  for (const char *p = s + 1; *p && *p != '*'; ++p) {
    cs ^= (uint8_t)*p;
  }

  return cs;
}

// Safely append "value,". NAN produces an empty field.
static bool appendField(
    char *buf,
    size_t bufsize,
    float value,
    const char *fmt = "%.1f"
) {
  if (buf == nullptr || bufsize == 0) {
    return false;
  }

  size_t len = strlen(buf);

  if (len >= bufsize) {
    return false;
  }

  int n = 0;

  if (isnan(value)) {
    n = snprintf(buf + len, bufsize - len, ",");
  } else {
    n = snprintf(buf + len, bufsize - len, fmt, value);

    if (n < 0 || (size_t)n >= bufsize - len) {
      return false;
    }

    len += (size_t)n;
    n = snprintf(buf + len, bufsize - len, ",");
  }

  return n >= 0 && (size_t)n < bufsize - len;
}

// Append the optional checksum and CRLF, then transmit.
// Returns the number of bytes accepted by MF_Serial::write().
static uint16_t sendRaw(
    MF_Serial *ser,
    char *buf,
    size_t bufsize,
    bool checksum
) {
  if (ser == nullptr || buf == nullptr || bufsize == 0) {
    ++ble_tx_failures;
    return 0;
  }

  // Find the terminating NUL safely.
  char *end = (char *)memchr(buf, '\0', bufsize);

  if (end == nullptr) {
    ++ble_format_errors;
    return 0;
  }

  size_t len = (size_t)(end - buf);
  char tail[8];

  int tail_len;

  if (checksum) {
    tail_len = snprintf(
        tail, sizeof(tail), "*%02X\r\n", nmea_checksum(buf));
  } else {
    tail_len = snprintf(tail, sizeof(tail), "\r\n");
  }

  if (tail_len < 0 ||
      (size_t)tail_len >= sizeof(tail) ||
      len + (size_t)tail_len >= bufsize) {
    ++ble_format_errors;
    return 0;
  }

  // Append the tail, including its terminating NUL.
  memcpy(buf + len, tail, (size_t)tail_len + 1);

  size_t total_len = len + (size_t)tail_len;

  // MF_Serial::write() takes uint8_t* and returns int.
  if (total_len > INT_MAX) {
    ++ble_format_errors;
    return 0;
  }

  int written = ser->write(
      reinterpret_cast<uint8_t *>(buf),
      (int)total_len
  );

  if (written < 0 || (size_t)written != total_len) {
    ++ble_tx_failures;
  }

  if (written > 0) {
    ble_tx_bytes += (uint32_t)written;
  }

  return written > 0 ? (uint16_t)written : 0;
}

// Remove appendField()'s final comma, then transmit.
static uint16_t sendSentence(
    MF_Serial *ser,
    char *buf,
    size_t bufsize,
    bool checksum
) {
  if (buf == nullptr || bufsize == 0) {
    ++ble_format_errors;
    return 0;
  }

  char *end = (char *)memchr(buf, '\0', bufsize);

  if (end == nullptr) {
    ++ble_format_errors;
    return 0;
  }

  if (end > buf && end[-1] == ',') {
    end[-1] = '\0';
  }

  return sendRaw(ser, buf, bufsize, checksum);
}

// $LK8EX1,pressure,altitude,vario,temperature,battery,*CS
// pressure=999999 means "unavailable", so the receiver uses our fused altitude
// instead of recomputing a baro-only one from raw pressure.
static void send_LK8EX1(MF_Serial *ser, float altitude_m, float vario_ms, float temp_c, float batt_v) {
  char buf[80*2] = "$LK8EX1,999999,";
  appendField(buf, sizeof(buf), altitude_m, "%.0f");
  appendField(buf, sizeof(buf), vario_ms * 100.0f, "%.0f"); //m/s -> cm/s
  appendField(buf, sizeof(buf), temp_c, "%.0f");
  appendField(buf, sizeof(buf), batt_v, "%.1f");
  sendSentence(ser, buf, sizeof(buf), true);
  //Serial.println(buf);
}

// $LXWP0,Y,TAS_kph,altitude,vario,,,,,,heading,windcourse,windspeed_kph*CS
// altitude and vario stay blank: LK8EX1 owns them, and two sentences carrying
// the same quantity risks the receiver preferring the wrong one.
static void send_LXWP0(MF_Serial *ser, float tas_ms, float heading_deg, float wind_from_deg, float wind_speed_ms) {
  char buf[96*2] = "$LXWP0,Y,";
  appendField(buf, sizeof(buf), tas_ms * 3.6f, "%.1f");   //NAN stays NAN
  strncat(buf, ",,,,,,,", sizeof(buf) - strlen(buf) - 1); //altitude, vario + 5 unused fields
  appendField(buf, sizeof(buf), heading_deg, "%.0f");
  appendField(buf, sizeof(buf), wind_from_deg, "%.0f");
  appendField(buf, sizeof(buf), wind_speed_ms * 3.6f, "%.1f");
  sendSentence(ser, buf, sizeof(buf), true);
  //Serial.println(buf);
}

// $XCTOD,field1,field2,field3  (no checksum, fields shown "as is" in XCTrack's
// "External data" widget - text only: no gauge, no audio).
//   field1 = netto [m/s], field2 = wind speed [km/h], field3 = wind from [deg]
static void send_XCTOD(MF_Serial *ser, float netto_ms, float wind_speed_ms, float wind_from_deg) {
  char buf[64*2] = "$XCTOD,";
  appendField(buf, sizeof(buf), netto_ms, "%+.1f");
  appendField(buf, sizeof(buf), wind_speed_ms * 3.6f, "%.0f");
  appendField(buf, sizeof(buf), wind_from_deg, "%.0f");
  sendSentence(ser, buf, sizeof(buf), false);
  //Serial.println(buf);
}

//==============================================================================
// GPS: $GNRMC + $GNGGA built from madflight's GpsState
//==============================================================================

// Days since 1970-01-01 -> civil date (Howard Hinnant's algorithm)
static void civil_from_days(int64_t z, int &y, int &m, int &d) {
  z += 719468;
  int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  unsigned doe = (unsigned)(z - era * 146097);
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  y = (int)yoe + (int)era * 400;
  unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned mp = (5 * doy + 2) / 153;
  d = (int)(doy - (153 * mp + 2) / 5 + 1);
  m = (int)(mp < 10 ? mp + 3 : mp - 9);
  if (m <= 2) y++;
}

// GPS week + time-of-week -> UTC date/time. Returns false if no week number yet.
static bool gps_to_utc(const GpsState &g, int &hh, int &mm, int &ss, int &cs, int &day, int &mon, int &year) {
  if (g.time_week == 0) return false;
  int64_t unix_s = 315964800LL                       //1980-01-06, the GPS epoch
                 + (int64_t)g.time_week * 604800LL
                 + (int64_t)(g.time / 1000)
                 - GPS_UTC_LEAP_SECONDS;
  int64_t sod = unix_s % 86400;
  hh = (int)(sod / 3600);
  mm = (int)((sod % 3600) / 60);
  ss = (int)(sod % 60);
  cs = (int)((g.time % 1000) / 10);
  civil_from_days(unix_s / 86400, year, mon, day);
  return true;
}

// 1E-7 degrees -> "ddmm.mmmmm,N" / "dddmm.mmmmm,E". Integer math only: a float
// has ~7 digits, which is not enough for 5 decimals of minutes.
static void fmt_latlon(char *out, size_t n, int32_t v, bool is_lat) {
  int64_t a = (v < 0) ? -(int64_t)v : (int64_t)v;
  int deg = (int)(a / 10000000);
  int64_t min_e7 = (a % 10000000) * 60;              //minutes * 1E7
  int min_i = (int)(min_e7 / 10000000);
  int min_f = (int)((min_e7 % 10000000) / 100);      //5 decimals
  if (is_lat) snprintf(out, n, "%02d%02d.%05d,%c", deg, min_i, min_f, v < 0 ? 'S' : 'N');
  else        snprintf(out, n, "%03d%02d.%05d,%c", deg, min_i, min_f, v < 0 ? 'W' : 'E');
}

// $GNRMC,hhmmss.ss,A,lat,N,lon,E,speed_kn,course,ddmmyy,,,A*CS
// Without a fix: status V and empty position/speed/course, like a real receiver.
static void send_GNRMC(MF_Serial *ser, const GpsState &g) {
  char t[16*2] = "", d[8] = "";
  int hh, mm, ss, cs, day, mon, year;
  if (gps_to_utc(g, hh, mm, ss, cs, day, mon, year)) {
    snprintf(t, sizeof(t), "%02d%02d%02d.%02d", hh, mm, ss, cs);
    snprintf(d, sizeof(d), "%02d%02d%02d", day, mon, year % 100);
  }
  char buf[100*2];
  if (g.fix >= GPS_OK_FIX_2D) {
    char la[20], lo[20];
    fmt_latlon(la, sizeof(la), g.lat, true);
    fmt_latlon(lo, sizeof(lo), g.lon, false);
    float knots = (float)g.sog * 0.001f * 1.943844f;   //mm/s -> knots
    float cog = (float)g.cog * 1e-5f;                  //1E-5 deg -> deg
    if (cog < 0) cog += 360.0f;
    snprintf(buf, sizeof(buf), "$GNRMC,%s,A,%s,%s,%.1f,%.1f,%s,,,A", t, la, lo, knots, cog, d);
  } else {
    snprintf(buf, sizeof(buf), "$GNRMC,%s,V,,,,,,,%s,,,N", t, d);
  }
  sendRaw(ser, buf, sizeof(buf), true);
  //Serial.println(buf);
}

// $GNGGA,hhmmss.ss,lat,N,lon,E,quality,numSV,hdop,alt_msl,M,,M,,*CS
// Geoid separation is left blank on purpose: GpsState.undulation is documented as
// "height WGS84 is above AMSL", the opposite sign convention to NMEA's, and
// receivers take altitude MSL straight from the altitude field anyway.
static void send_GNGGA(MF_Serial *ser, const GpsState &g) {
  char t[16] = "";
  int hh, mm, ss, cs, day, mon, year;
  if (gps_to_utc(g, hh, mm, ss, cs, day, mon, year)) {
    snprintf(t, sizeof(t), "%02d%02d%02d.%02d", hh, mm, ss, cs);
  }
  char buf[100*2];
  if (g.fix >= GPS_OK_FIX_2D) {
    int quality = 1;                                   //GPS fix
    if (g.fix == GPS_OK_FIX_3D_DGPS) quality = 2;
    else if (g.fix == GPS_OK_FIX_3D_RTK_FIXED) quality = 4;
    else if (g.fix == GPS_OK_FIX_3D_RTK_FLOAT) quality = 5;
    char la[20], lo[20];
    fmt_latlon(la, sizeof(la), g.lat, true);
    fmt_latlon(lo, sizeof(lo), g.lon, false);
    snprintf(buf, sizeof(buf), "$GNGGA,%s,%s,%s,%d,%02d,%.2f,%.1f,M,,M,,",
             t, la, lo, quality, (int)g.sat, (float)g.hdop / 100.0f, (float)g.alt / 1000.0f);
  } else {
    snprintf(buf, sizeof(buf), "$GNGGA,%s,,,,,0,%02d,,,M,,M,,", t, (int)g.sat);
  }
  sendRaw(ser, buf, sizeof(buf), true);
  //Serial.println(buf);
}

//==============================================================================

void ble_task(void *pvParameters) {
  (void)pvParameters;

  //UART1 - GPIO24 TX / GPIO25 RX. Baud must match the HC-42's configured baud.
  //At 115200 an ~80 byte sentence takes ~7 ms on the wire.
  MF_Serial *ser = hal_get_ser_bus(1, 115200);

  ScheduleFreq vario_schedule(10); //XCTrack gains nothing from faster than ~10 Hz
  ScheduleFreq wind_schedule(5);   //wind is GPS-rate and averaged; Vector Vario sends it at 0.5 Hz
  ScheduleFreq gps_schedule(2);    //GPS arrives at 10 Hz; 5 Hz halves the BLE traffic - raise it if XCTrack feels laggy
  uint32_t last_gps_time = 0;

  for (;;) {
    if (vario_schedule.expired()) {
      BleData d = ble_data; //snapshot of independent scalars
      float vario = isnan(d.te_vario_ms) ? alt.getV() : d.te_vario_ms;
      send_LK8EX1(ser, alt.getH(), vario, bar.temp, bat.v);
    }
    if (wind_schedule.expired()) {
      BleData d = ble_data;
      send_LXWP0(ser, d.tas_ms, ahr.yaw, d.wind_from_deg, d.wind_speed_ms);
      if (!isnan(d.netto_ms) || !isnan(d.wind_speed_ms)) {
        send_XCTOD(ser, d.netto_ms, d.wind_speed_ms, d.wind_from_deg);
      }
    }
    if (gps_schedule.expired()) {
      GpsState g; //consistent snapshot from the message broker, not a field-by-field read
      //only send when the receiver produced a NEW solution - repeating a stale one
      //would make a dead GPS look alive to XCTrack
      if (gps.topic.pull_latest(&g) && g.fix != NO_GPS && g.time != last_gps_time) {
        last_gps_time = g.time;
        send_GNRMC(ser, g);
        send_GNGGA(ser, g);
      }
    }
    
    /*char buf[100*2];
    snprintf(buf, sizeof(buf),
         "$GNRMC,123519.00,A,4807.038,N,01131.000,E,022.4,084.4,171026,003.1,E,A");
    Serial.print(buf);
    delay(20);
    sendRaw(ser, buf, sizeof(buf), true);
    */
 /*   static uint32_t last_stack_report = 0;

if (millis() - last_stack_report >= 5000) {
    last_stack_report = millis();

    UBaseType_t free_words =
        uxTaskGetStackHighWaterMark(NULL);

    Serial.printf(
        "[BLE] stack min free: %u words (%u bytes)\n",
        (unsigned)free_words,
        (unsigned)(free_words * sizeof(StackType_t))
    );
}*/
    vTaskDelay(2);
  }
}

void ble_task_start() {
  //2x the default stack: several 100-byte sentence buffers plus float snprintf()
  hal_xTaskCreate(ble_task, "mf_BLE", 2 * MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, uxTaskPriorityGet(NULL), NULL, 0);
}