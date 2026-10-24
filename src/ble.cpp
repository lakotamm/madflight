#include "madflight_modules.h" // NOT madflight.h - that one may only be included once, in main.cpp
#include <math.h>
#include "ble.h"

BleData ble_data;

// Diagnostics, printed once per second to the USB serial (disable: BLE_DEBUG 0)
// Also prints the last RMC and GGA sentence exactly as sent, so they can be pasted into an NMEA validator.
#define BLE_DEBUG 0
static uint32_t cnt_ok = 0, cnt_drop = 0, cnt_rmc = 0, cnt_gga = 0, cnt_pull_ok = 0, cnt_new_fix = 0;
static uint32_t cnt_no_time = 0; // GPS solutions skipped because no UTC date is available (time_week == 0)
static uint32_t cnt_long = 0;    // sentences longer than the NMEA maximum of 82 characters
#if BLE_DEBUG
static char dbg_rmc[100] = "", dbg_gga[100] = "";
#endif

// GPS-UTC offset. GpsState.time is GPS time; NMEA wants UTC. This has been 18 s
// since the 2016-12-31 leap second - revisit if another one is ever announced.
#define GPS_UTC_LEAP_SECONDS 18

// NMEA 0183: a sentence is at most 82 characters including '$' and CRLF
#define NMEA_MAX_LEN 82

// Standard NMEA checksum: XOR of all bytes between '$' and '*'
static uint8_t nmea_checksum(const char *s) {
  uint8_t cs = 0;
  for (const char *p = s + 1; *p && *p != '*'; p++) cs ^= (uint8_t)*p;
  return cs;
}

//==============================================================================
// Float formatting WITHOUT newlib's printf float support.
// snprintf("%f") uses newlib's dtoa, which keeps a shared, unlocked free-list. Two tasks formatting floats at the
// same time corrupt it and the MCU hard faults in mprec.c. Integer-only formatting has no shared state.
//==============================================================================

// Formats v with 'decimals' decimals (0..4), optional '+' sign for positive numbers. Never produces "-0.0".
static void fmt_fixed(char *out, size_t n, float v, int decimals, bool plus = false) {
  static const uint32_t p10[] = {1, 10, 100, 1000, 10000};
  if (decimals < 0) decimals = 0;
  if (decimals > 4) decimals = 4;
  bool neg = v < 0;
  float a = neg ? -v : v;
  if (a > 1.0e5f) a = 1.0e5f;                                   // clamp: keeps scaled value inside 32 bit
  uint32_t scaled = (uint32_t)(a * (float)p10[decimals] + 0.5f);
  if (scaled == 0) neg = false;                                 // rounded to zero -> no negative zero
  const char *sign = neg ? "-" : (plus ? "+" : "");
  uint32_t ip = scaled / p10[decimals];
  uint32_t fp = scaled % p10[decimals];
  if (decimals == 0) {
    snprintf(out, n, "%s%lu", sign, (unsigned long)ip);
  } else {
    char frac[6];
    for (int i = decimals - 1; i >= 0; i--) { frac[i] = '0' + (fp % 10); fp /= 10; }
    frac[decimals] = '\0';
    snprintf(out, n, "%s%lu.%s", sign, (unsigned long)ip, frac);
  }
}

// Appends "value," or just "," if value is NAN (field not available).
static void appendField(char *buf, size_t bufsize, float value, int decimals = 1, bool plus = false) {
  char tmp[16];
  if (isnan(value)) tmp[0] = '\0';
  else fmt_fixed(tmp, sizeof(tmp), value, decimals, plus);
  strncat(buf, tmp, bufsize - strlen(buf) - 1);
  strncat(buf, ",", bufsize - strlen(buf) - 1);
}

// Optionally appends *CS, then CRLF, and writes the buffer out as-is.
// Returns false if the UART refused it: the RP2 DMA serial write() is all-or-nothing
// and returns 0 when the whole sentence doesn't fit in the free TX buffer space.
static bool sendRaw(MF_Serial *ser, char *buf, size_t bufsize, bool checksum) {
  char tail[8];
  if (checksum) snprintf(tail, sizeof(tail), "*%02X\r\n", nmea_checksum(buf));
  else snprintf(tail, sizeof(tail), "\r\n");
  strncat(buf, tail, bufsize - strlen(buf) - 1);
  int len = (int)strlen(buf);
  if (len > NMEA_MAX_LEN) cnt_long++;
  if (ser->write((uint8_t*)buf, len) == len) {
    cnt_ok++;
#if BLE_DEBUG
    if (strncmp(buf + 1, "GNRMC", 5) == 0) strncpy(dbg_rmc, buf, sizeof(dbg_rmc) - 1);
    else if (strncmp(buf + 1, "GNGGA", 5) == 0) strncpy(dbg_gga, buf, sizeof(dbg_gga) - 1);
#endif
    return true;
  }
  cnt_drop++;
  return false;
}

// Like sendRaw, but first drops the trailing comma left by appendField().
static bool sendSentence(MF_Serial *ser, char *buf, size_t bufsize, bool checksum) {
  size_t len = strlen(buf);
  if (len > 0 && buf[len - 1] == ',') buf[len - 1] = '\0';
  return sendRaw(ser, buf, bufsize, checksum);
}

// $LK8EX1,pressure,altitude,vario,temperature,battery,*CS
// pressure=999999 means "unavailable", so the receiver uses our fused altitude
// instead of recomputing a baro-only one from raw pressure.
static bool send_LK8EX1(MF_Serial *ser, float altitude_m, float vario_ms, float temp_c, float batt_v) {
  char buf[80] = "$LK8EX1,999999,";
  appendField(buf, sizeof(buf), altitude_m, 0);
  appendField(buf, sizeof(buf), vario_ms * 100.0f, 0); //m/s -> cm/s
  appendField(buf, sizeof(buf), temp_c, 0);
  appendField(buf, sizeof(buf), batt_v, 1);
  return sendSentence(ser, buf, sizeof(buf), true);
}

// $LXWP0,Y,TAS_kph,altitude,vario,,,,,,heading,windcourse,windspeed_kph*CS
// altitude and vario stay blank: LK8EX1 owns them, and two sentences carrying
// the same quantity risks the receiver preferring the wrong one.
static bool send_LXWP0(MF_Serial *ser, float tas_ms, float heading_deg, float wind_from_deg, float wind_speed_ms) {
  char buf[96] = "$LXWP0,Y,";
  appendField(buf, sizeof(buf), tas_ms * 3.6f, 1);        //NAN stays NAN
  strncat(buf, ",,,,,,,", sizeof(buf) - strlen(buf) - 1); //altitude, vario + 5 unused fields
  appendField(buf, sizeof(buf), heading_deg, 0);
  appendField(buf, sizeof(buf), wind_from_deg, 0);
  appendField(buf, sizeof(buf), wind_speed_ms * 3.6f, 1);
  return sendSentence(ser, buf, sizeof(buf), true);
}

// $XCTOD,field1,field2,field3  (no checksum, fields shown "as is" in XCTrack's
// "External data" widget - text only: no gauge, no audio).
//   field1 = netto [m/s], field2 = wind speed [km/h], field3 = wind from [deg]
static bool send_XCTOD(MF_Serial *ser, float netto_ms, float wind_speed_ms, float wind_from_deg) {
  char buf[64] = "$XCTOD,";
  appendField(buf, sizeof(buf), netto_ms, 1, true);
  appendField(buf, sizeof(buf), wind_speed_ms * 3.6f, 0);
  appendField(buf, sizeof(buf), wind_from_deg, 0);
  return sendSentence(ser, buf, sizeof(buf), false);
}

//==============================================================================
// GPS: $GNRMC + $GNGGA built from madflight's GpsState
//
// XCTrack needs BOTH sentences, with a valid UTC date (only RMC carries the date) and matching timestamps:
// per its own log, a GGA without a known date is dropped ("no date from RMC line"), and an RMC without an
// altitude from GGA is incomplete ("Missing altitude fix"). So: RMC first, then GGA, same time, valid date.
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
static bool send_GNRMC(MF_Serial *ser, const GpsState &g) {
  char t[16] = "", d[8] = "";
  int hh, mm, ss, cs, day, mon, year;
  if (gps_to_utc(g, hh, mm, ss, cs, day, mon, year)) {
    snprintf(t, sizeof(t), "%02d%02d%02d.%02d", hh, mm, ss, cs);
    snprintf(d, sizeof(d), "%02d%02d%02d", day, mon, year % 100);
  }
  char buf[100];
  if (g.fix >= GPS_OK_FIX_2D) {
    char la[20], lo[20], kn[16], cg[16];
    fmt_latlon(la, sizeof(la), g.lat, true);
    fmt_latlon(lo, sizeof(lo), g.lon, false);
    float knots = (float)g.sog * 0.001f * 1.943844f;   //mm/s -> knots
    float cog = (float)g.cog * 1e-5f;                  //1E-5 deg -> deg
    if (cog < 0) cog += 360.0f;
    fmt_fixed(kn, sizeof(kn), knots, 1);
    fmt_fixed(cg, sizeof(cg), cog, 1);
    snprintf(buf, sizeof(buf), "$GNRMC,%s,A,%s,%s,%s,%s,%s,,,A", t, la, lo, kn, cg, d);
  } else {
    snprintf(buf, sizeof(buf), "$GNRMC,%s,V,,,,,,,%s,,,N", t, d);
  }
  return sendRaw(ser, buf, sizeof(buf), true);
}

// $GNGGA,hhmmss.ss,lat,N,lon,E,quality,numSV,hdop,alt_msl,M,,M,,*CS
// Geoid separation is left blank on purpose: GpsState.undulation is documented as
// "height WGS84 is above AMSL", and it is unclear whether that is the same sign as NMEA's
// separation, so it is not sent. Receivers take altitude MSL straight from the altitude field.
static bool send_GNGGA(MF_Serial *ser, const GpsState &g) {
  char t[16] = "";
  int hh, mm, ss, cs, day, mon, year;
  if (gps_to_utc(g, hh, mm, ss, cs, day, mon, year)) {
    snprintf(t, sizeof(t), "%02d%02d%02d.%02d", hh, mm, ss, cs);
  }
  char buf[100];
  if (g.fix >= GPS_OK_FIX_2D) {
    int quality = 1;                                   //GPS fix
    if (g.fix == GPS_OK_FIX_3D_DGPS) quality = 2;
    else if (g.fix == GPS_OK_FIX_3D_RTK_FIXED) quality = 4;
    else if (g.fix == GPS_OK_FIX_3D_RTK_FLOAT) quality = 5;
    char la[20], lo[20], hd[16], al[16];
    fmt_latlon(la, sizeof(la), g.lat, true);
    fmt_latlon(lo, sizeof(lo), g.lon, false);
    fmt_fixed(hd, sizeof(hd), (float)g.hdop / 100.0f, 2);
    fmt_fixed(al, sizeof(al), (float)g.alt / 1000.0f, 1);
    snprintf(buf, sizeof(buf), "$GNGGA,%s,%s,%s,%d,%02d,%s,%s,M,,M,,", t, la, lo, quality, (int)g.sat, hd, al);
  } else {
    snprintf(buf, sizeof(buf), "$GNGGA,%s,,,,,0,%02d,,,M,,M,,", t, (int)g.sat);
  }
  return sendRaw(ser, buf, sizeof(buf), true);
}

//==============================================================================

void ble_task(void *pvParameters) {
  (void)pvParameters;

  //UART1 - GPIO24 TX / GPIO25 RX. Baud must match the HC-42's configured baud.
  //At 115200 an ~80 byte sentence takes ~7 ms on the wire.
  MF_Serial *ser = hal_get_ser_bus(1, 115200);

  ScheduleFreq vario_schedule(10); //XCTrack gains nothing from faster than ~10 Hz
  ScheduleFreq wind_schedule(2);   //wind is GPS-rate and averaged; Vector Vario sends it at 0.5 Hz
  ScheduleFreq gps_schedule(5);    //GPS arrives at 10 Hz; 5 Hz halves the BLE traffic. For testing try 1 Hz: the HC-42 BLE link may not keep up
  ScheduleFreq debug_schedule(1);
  uint32_t last_gps_time = 0;

  // A sentence becomes "pending" when its schedule fires and stays pending until the
  // UART accepts it. At most ONE sentence is written per loop pass, so we never dump
  // ~220 bytes into the 256 byte DMA buffer (and the HC-42) in a single burst.
  bool pend_vario = false, pend_wind = false, pend_xctod = false, pend_rmc = false, pend_gga = false;
  GpsState gps_snap; //consistent snapshot, shared by the RMC + GGA of one fix

  for (;;) {
    if (vario_schedule.expired()) pend_vario = true;
    if (wind_schedule.expired()) { pend_wind = true; pend_xctod = true; }
    if (gps_schedule.expired()) {
      GpsState g; //consistent snapshot from the message broker
      if (gps.topic.pull_latest(&g)) {
        cnt_pull_ok++;
        //only send a NEW solution - repeating a stale one would make a dead GPS look alive
        if (g.fix != NO_GPS && g.time != last_gps_time) {
          last_gps_time = g.time;
          int hh, mm, ss, cs, day, mon, year;
          if (gps_to_utc(g, hh, mm, ss, cs, day, mon, year)) {
            gps_snap = g;
            pend_rmc = pend_gga = true;
            cnt_new_fix++;
          } else {
            cnt_no_time++; //no UTC date yet: XCTrack would drop these sentences anyway
          }
        }
      }
    }

    if (pend_rmc) {
      if (send_GNRMC(ser, gps_snap)) { pend_rmc = false; cnt_rmc++; }
    } else if (pend_gga) {
      if (send_GNGGA(ser, gps_snap)) { pend_gga = false; cnt_gga++; }
    } else if (pend_vario) {
      BleData d = ble_data; //snapshot of independent scalars
      float vario = isnan(d.te_vario_ms) ? alt.getV() : d.te_vario_ms;
      if (send_LK8EX1(ser, alt.getH(), vario, bar.temp, bat.v)) pend_vario = false;
    } else if (pend_wind) {
      BleData d = ble_data;
      if (send_LXWP0(ser, d.tas_ms, ahr.yaw, d.wind_from_deg, d.wind_speed_ms)) pend_wind = false;
    } else if (pend_xctod) {
      BleData d = ble_data;
      if (!isnan(d.netto_ms) || !isnan(d.wind_speed_ms)) send_XCTOD(ser, d.netto_ms, d.wind_speed_ms, d.wind_from_deg);
      pend_xctod = false;
    }

#if BLE_DEBUG
    if (debug_schedule.expired()) {
      // integer-only format strings on purpose (no float printf, see fmt_fixed)
      Serial.printf("[BLE] gps fix=%d sat=%d week=%u time=%lu | pull_ok=%lu new_fix=%lu no_time=%lu rmc=%lu gga=%lu | uart ok=%lu DROPPED=%lu long=%lu txfree=%d\n",
        (int)gps.fix, (int)gps.sat, (unsigned)gps.time_week, (unsigned long)gps.time,
        (unsigned long)cnt_pull_ok, (unsigned long)cnt_new_fix, (unsigned long)cnt_no_time, (unsigned long)cnt_rmc, (unsigned long)cnt_gga,
        (unsigned long)cnt_ok, (unsigned long)cnt_drop, (unsigned long)cnt_long, (int)ser->availableForWrite());
      if (dbg_rmc[0]) Serial.print(dbg_rmc);
      if (dbg_gga[0]) Serial.print(dbg_gga);
    }
#endif

    delay(10); //10 ms between sentences also gives the HC-42 time to forward them over BLE
  }
}

void ble_task_start() {
  //2x the default stack: several 100-byte sentence buffers plus snprintf()
  hal_xTaskCreate(ble_task, "mf_BLE", 2 * MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, uxTaskPriorityGet(NULL), NULL, 0);
}