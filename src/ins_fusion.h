/*==========================================================================================
ins_fusion.h - INSLIB nav_suite adapter for madflight (variometer / wind project)

Feeds madflight's IMU, magnetometer, barometer and GPS into INSLIB's nav_suite
and publishes the solution on a MsgTopic ("ins").

LICENSE: INSLIB is AGPL-3.0. This file links against it, so it and the firmware
built with it fall under AGPL-3.0 as well (madflight is MIT, which is compatible).

STATUS: first draft, written against the madflight and INSLIB sources without being
able to compile it with the real toolchain. Items marked VERIFY need a bench check.
==========================================================================================*/
#pragma once

#include <Arduino.h>
#include <madflight_modules.h> // modules only - madflight.h must be included exactly once (in main.cpp)
#include <math.h>

#include "inslib/nav_suite.h" // INSLIB (has its own extern "C" guards)

//---------------------------------------------------------------------------------
// Published solution
//---------------------------------------------------------------------------------
struct __attribute__((aligned(4))) InsFusionState {
  uint32_t ts = 0;          // IMU timestamp (low 32 bit) of this solution [us]
  uint8_t mode = 0;         // nav_suite_mode_t: 0 none, 1 attitude only, 2 coasting, 3 full
  bool ready = false;       // INS position/velocity solution usable
  bool gnss_used = false;   // a GNSS fix was fused in the last epoch

  float vel_ned[3] = {0};   // INS velocity NED [m/s]
  float vel_up = 0;         // INS climb rate = -vel_ned[2] [m/s]
  bool  vel_up_valid = false;
  float baro_v_up = 0;      // baro_alt vertical-channel climb rate [m/s]
  bool  baro_v_up_valid = false;
  float baro_h = 0;         // baro_alt filter height relative to start [m] (same filter as baro_v_up)

  float pos_ned[3] = {0};   // local NED position relative to start [m]
  float height = 0;         // best local height above start, positive up [m]
  bool  height_valid = false;

  float q[4] = {1, 0, 0, 0}; // body->NED quaternion (w,x,y,z)
  float roll = 0, pitch = 0, yaw = 0; // [deg]
  bool  att_valid = false;

  float acc_bias[3] = {0};  // estimated accelerometer bias, body FRD [m/s^2]
};

//---------------------------------------------------------------------------------
// Configuration
//---------------------------------------------------------------------------------
struct InsFusionConfig {
  uint32_t samples_per_epoch = 1; // IMU samples per INS epoch. 1 = every IMU sample goes straight to the filter (no averaging).
  uint32_t period_ms = 10;        // only for the optional INS task (start_task): how often it drains the IMU topic
  float gnss_rx_latency_ms = 100; // receiver internal latency before the PVT message is sent. VERIFY/tune
  uint8_t gnss_min_sats = 6;
  float gnss_max_vel_acc_mps = 2.0f;
  float mag_year = 2026.8f;       // decimal year for the World Magnetic Model
  bool use_mag = true;
};

//---------------------------------------------------------------------------------
// Module
//---------------------------------------------------------------------------------
class InsFusion {
public:
  InsFusionConfig config;
  MsgTopic<InsFusionState> topic = MsgTopic<InsFusionState>("ins");
  InsFusionState state;

  // diagnostics
  uint32_t n_epochs = 0;
  uint32_t n_imu_samples = 0;
  uint32_t n_gnss = 0;
  uint32_t last_step_us = 0, max_step_us = 0;
  uint32_t max_dt_us = 0, n_gap = 0;   // largest epoch spacing seen, and number of epochs later than 1.5x the running mean
  float dbg_acc_ins[3] = {0}; // last averaged accel in INSLIB convention (expect {0,0,-9.8} when level and still)

  // Returns 0 on success
  int setup() {
    if (!imu.installed()) return -1;

    // nav_suite_t is large (history ring buffers + several Kalman filters): keep it static, zero-initialised.
    memset(&ns, 0, sizeof(ns));

    ins_init_t init;
    memset(&init, 0, sizeof(init));
    init.pos_init_stddev_m = 50.0f;     // coarse: auto_init takes the first GNSS fix
    init.vel_init_stddev_mps = 5.0f;
    for (int i = 0; i < 3; i++) init.rpy_init_stddev_rad[i] = 0.17453293f; // 10 deg

    ins_options_t opt;
    memset(&opt, 0, sizeof(opt));
    opt.auto_init = true;               // level + position from the first IMU/GNSS data
    // VERIFY: later tuning candidates (see ins.h ins_options_t): kalman_update_dt_sec (default 0.05 s),
    // gnss_max_*_stddev, imu_*_misalignment, imu_acc_fixed_bias (use your accel calibration), etc.

    if (nav_suite_init(&ns, &init, &opt) != 0) return -2;

    imu_sub = new MsgSubscription<ImuState>("ins_imu", &imu.topic);
    started = true;
    Serial.printf("INS: nav_suite_t = %u bytes\n", (unsigned)sizeof(nav_suite_t));
    return 0;
  }

  // Drain the IMU topic and feed every sample to the filter. Use this from the optional INS task (start_task).
  // Returns true if at least one epoch was processed.
  bool step() {
    if (!started) return false;
    if (!imu_sub) imu_sub = new MsgSubscription<ImuState>("ins_imu", &imu.topic);
    ImuState s;
    bool any = false;
    while (imu_sub->pull_next(&s)) any |= feed(s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.ts);
    return any;
  }

  // Feed ONE raw IMU sample (madflight units/frame: [g], [deg/s], ts in [us]).
  // With config.samples_per_epoch == 1 every sample is one filter epoch (no averaging); with N > 1 the samples are
  // averaged over N. Call from exactly one context (the IMU task via the AHR gizmo / imu_loop(), OR the INS task).
  // Returns true if an epoch was processed.
  bool feed(float ax, float ay, float az, float gx, float gy, float gz, uint32_t ts) {
    if (!started) return false;
    acc_sum[0] += ax; acc_sum[1] += ay; acc_sum[2] += az;
    gyr_sum[0] += gx; gyr_sum[1] += gy; gyr_sum[2] += gz;
    n_acc++;
    n_imu_samples++;
    if (n_acc < config.samples_per_epoch) return false;

    uint32_t t0 = micros();
    float sum_a[3] = {acc_sum[0], acc_sum[1], acc_sum[2]};
    float sum_g[3] = {gyr_sum[0], gyr_sum[1], gyr_sum[2]};
    const int n = (int)n_acc;
    const uint32_t ts_end = ts;
    for (int i = 0; i < 3; i++) acc_sum[i] = gyr_sum[i] = 0;
    n_acc = 0;

    int64_t t_end64 = unwrap(ts_end);
    if (!have_t_prev) { // first epoch: just establish time base
      t_prev64 = t_end64;
      have_t_prev = true;
      return false;
    }
    float dt = (float)(t_end64 - t_prev64) * 1e-6f;
    uint32_t dt_us = (uint32_t)(t_end64 - t_prev64);
    t_prev64 = t_end64;
    if (dt <= 0.0f || dt > 0.5f) return false;

    // spacing diagnostics: a late epoch means the IMU task was blocked (e.g. a long covariance update)
    if (dt_us > max_dt_us) max_dt_us = dt_us;
    if (dt_mean_us > 0 && dt_us > 1.5f * dt_mean_us) n_gap++;
    dt_mean_us = (dt_mean_us == 0) ? (float)dt_us : dt_mean_us + 0.01f * ((float)dt_us - dt_mean_us);

    const float inv_n = 1.0f / (float)n;
    const float G = 9.80665f;
    const float D2R = 0.017453292519943295f;

    ins_measurements_t m;
    memset(&m, 0, sizeof(m));
    m.timestamp = t_end64;
    m.strapdown_dt_sec = dt;

    // madflight: acceleration in [g], +1 g on the Down axis when level (it is -specific force).
    // INSLIB: specific force in [m/s^2], FRD, -9.8 on z when level. => negate. VERIFY with dbg_acc_ins.
    m.acc.is_valid = true;
    for (int i = 0; i < 3; i++) {
      m.acc.data[i] = -sum_a[i] * inv_n * G;
      dbg_acc_ins[i] = m.acc.data[i];
    }
    m.gyr.is_valid = true;
    for (int i = 0; i < 3; i++) m.gyr.data[i] = sum_g[i] * inv_n * D2R; // deg/s -> rad/s

    // ---- 2. magnetometer (new sample only) ----
    if (config.use_mag) {
      MagState ms;
      if (mag.topic.pull_latest(&ms) && ms.ts != last_mag_ts && !(ms.mx == 0 && ms.my == 0 && ms.mz == 0)) {
        last_mag_ts = ms.ts;
        m.mag.is_valid = true; // VERIFY: sign/axes of mag vs. IMU (nose north, level: mx large positive, mz positive in the northern hemisphere)
        m.mag.data[0] = ms.mx; m.mag.data[1] = ms.my; m.mag.data[2] = ms.mz;
      }
    }

    // ---- 3. barometer (new sample only) ----
    {
      BarState bs;
      if (bar.topic.pull_latest(&bs) && bs.ts != last_bar_ts && bs.press > 30000.0f && bs.press < 110000.0f) {
        last_bar_ts = bs.ts;
        m.baro.is_valid = true;
        m.baro.pressure_pa = bs.press;
        m.baro.stddev_m = 0.0f; // consumer default
      }
    }

    // ---- 4. GNSS (new fix only) ----
    state.gnss_used = false;
    {
      GpsState g;
      if (gps.installed() && gps.topic.pull_latest(&g) && g.time != last_gps_itow && g.fix >= GPS_OK_FIX_3D) {
        last_gps_itow = g.time;
        float vacc = g.have_vel_acc ? g.vel_acc * 1e-3f : 1.0f; // mm/s -> m/s
        if (g.sat >= config.gnss_min_sats && vacc > 0 && vacc < config.gnss_max_vel_acc_mps) {
          float hacc = g.have_hacc ? g.hacc * 1e-3f : 5.0f;
          float vac2 = g.have_vacc ? g.vacc * 1e-3f : 8.0f;
          float undulation_mm = g.have_undulation ? (float)g.undulation : 0.0f;

          m.gnss_pos.is_valid = true;
          m.gnss_pos.llh[0] = (double)g.lat * 1e-7 * (M_PI / 180.0);
          m.gnss_pos.llh[1] = (double)g.lon * 1e-7 * (M_PI / 180.0);
          // madflight stores MSL altitude and undulation = MSL - ellipsoid  =>  ellipsoid = MSL - undulation
          m.gnss_pos.llh[2] = ((double)g.alt - (double)undulation_mm) * 1e-3;
          memset(m.gnss_pos.Qll_ned, 0, sizeof(m.gnss_pos.Qll_ned)); // 3x3 column-major, diagonal only
          m.gnss_pos.Qll_ned[0] = hacc * hacc;
          m.gnss_pos.Qll_ned[4] = hacc * hacc;
          m.gnss_pos.Qll_ned[8] = vac2 * vac2;

          if (g.have_veld) {
            m.gnss_vel.is_valid = true;
            m.gnss_vel.vel_ned[0] = g.veln * 1e-3f;
            m.gnss_vel.vel_ned[1] = g.vele * 1e-3f;
            m.gnss_vel.vel_ned[2] = g.veld * 1e-3f;
            memset(m.gnss_vel.Qll_ned, 0, sizeof(m.gnss_vel.Qll_ned));
            m.gnss_vel.Qll_ned[0] = vacc * vacc;
            m.gnss_vel.Qll_ned[4] = vacc * vacc;
            m.gnss_vel.Qll_ned[8] = vacc * vacc; // one 3D speed accuracy for all axes (conservative for the vertical axis)
          }

          // How old is the fix relative to this epoch? Message arrival is last_gps_time_ms (system ms, VERIFY it is
          // stamped when the PVT message is parsed), plus receiver latency before it was sent.
          uint32_t arrival_us = g.last_gps_time_ms * 1000u;
          int32_t age_us = (int32_t)(ts_end - arrival_us) + (int32_t)(config.gnss_rx_latency_ms * 1000.0f);
          if (age_us < 0) age_us = 0;
          m.gnss_delay_ms = age_us / 1000;
          m.gnss_delay_us = age_us % 1000;
          m.gps_week = g.time_week;
          m.gps_itow_ms = (int)g.time;

          if (!mag_model_set) {
            nav_suite_set_magnetic_model_position(&ns, m.gnss_pos.llh[0], m.gnss_pos.llh[1], config.mag_year);
            mag_model_set = true;
          }
          state.gnss_used = true;
          n_gnss++;
        }
      }
    }

    // ---- 5. run the filters ----
    nav_suite_update(&ns, &m);
    n_epochs++;

    // ---- 6. publish ----
    state.ts = ts_end;
    state.mode = (uint8_t)nav_suite_get_mode(&ns);
    state.ready = ins_is_ready(&ns.ins);

    float v[3];
    state.vel_up_valid = state.ready && ins_get_velocity_ned(&ns.ins, v);
    if (state.vel_up_valid) {
      for (int i = 0; i < 3; i++) state.vel_ned[i] = v[i];
      state.vel_up = -v[2];
    }
    float hb, vb;
    state.baro_v_up_valid = nav_suite_get_baro_alt(&ns, &hb, &vb);
    if (state.baro_v_up_valid) { state.baro_v_up = vb; state.baro_h = hb; }

    float p[3];
    if (state.ready && ins_get_position_local(&ns.ins, p)) for (int i = 0; i < 3; i++) state.pos_ned[i] = p[i];
    state.height_valid = nav_suite_get_height(&ns, &state.height);

    float r, pt, y;
    state.att_valid = nav_suite_get_rpy(&ns, &r, &pt, &y);
    if (state.att_valid) {
      state.roll = r * 57.2957795f; state.pitch = pt * 57.2957795f; state.yaw = y * 57.2957795f;
    }
    float q[4];
    if (state.ready && ins_get_quaternion(&ns.ins, q)) {
      for (int i = 0; i < 4; i++) state.q[i] = q[i];
    } else if (state.att_valid) { // attitude-only mode (no GNSS yet): build q from roll/pitch/yaw (ZYX, body->NED)
      float cr = cosf(r * 0.5f), sr = sinf(r * 0.5f), cp = cosf(pt * 0.5f), sp = sinf(pt * 0.5f), cy = cosf(y * 0.5f), sy = sinf(y * 0.5f);
      state.q[0] = cr * cp * cy + sr * sp * sy;
      state.q[1] = sr * cp * cy - cr * sp * sy;
      state.q[2] = cr * sp * cy + sr * cp * sy;
      state.q[3] = cr * cp * sy - sr * sp * cy;
    }
    float ab[3];
    if (state.ready && ins_get_bias_acc(&ns.ins, ab)) for (int i = 0; i < 3; i++) state.acc_bias[i] = ab[i];

    topic.publish(&state);

    last_step_us = micros() - t0;
    if (last_step_us > max_step_us) max_step_us = last_step_us;
    return true;
  }

  // FreeRTOS task: 16 KB+ of stack needed (INSLIB reports ~13 KB worst case on x86-64, VERIFY with uxTaskGetStackHighWaterMark)
  static void task(void* pv) {
    InsFusion* self = (InsFusion*)pv;
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(self->config.period_ms);
    for (;;) {
      self->step();
      vTaskDelayUntil(&last, period);
    }
  }

  // Best available climb rate [m/s]: GNSS-aided INS when it is ready and in FULL mode, else the baro/accel filter.
  float getV() { return (state.vel_up_valid && state.mode == 3) ? state.vel_up : state.baro_v_up; }
  // Best available height relative to start [m]
  float getH() { return state.height_valid ? state.height : state.baro_h; }
  const char* vSource() { return (state.vel_up_valid && state.mode == 3) ? "GNSS-INS" : (state.baro_v_up_valid ? "BARO-INS" : "none"); }

  // One-line orientation self-test string for printing (use your safe printer, e.g. mfSerial.printf("%s\n", buf)).
  // Compares the INS roll/pitch/yaw with the angles recomputed from the published quaternion (they must agree - this
  // validates the quaternion convention), and shows the accelerometer as INSLIB sees it (level + still: {0,0,-9.8}).
  int orientationString(char* buf, size_t n) {
    const float r2d = 57.2957795f;
    const float* q = state.q;
    float qr = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) * r2d;
    float sp = 2 * (q[0] * q[2] - q[3] * q[1]); if (sp > 1) sp = 1; if (sp < -1) sp = -1;
    float qp = asinf(sp) * r2d;
    float qy = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) * r2d;
    return snprintf(buf, n,
      "INS rpy %7.2f %7.2f %7.2f | from q %7.2f %7.2f %7.2f | acc(INS frame) %6.2f %6.2f %6.2f | mode=%d ready=%d att=%d | "
      "bias %5.2f %5.2f %5.2f | step max %uus, epoch dt max %uus, gaps %u",
      state.roll, state.pitch, state.yaw, qr, qp, qy,
      dbg_acc_ins[0], dbg_acc_ins[1], dbg_acc_ins[2], state.mode, state.ready, state.att_valid,
      state.acc_bias[0], state.acc_bias[1], state.acc_bias[2], (unsigned)max_step_us, (unsigned)max_dt_us, (unsigned)n_gap);
  }

  // Start the INS task. core: 0 or 1 - check which core mf_IMU runs on (FreeRTOS task table, "Core" column) and
  // prefer the other one if it has idle time. priority: keep it BELOW the IMU task so the IMU is never starved.
  void start_task(UBaseType_t priority, int core = 0, uint32_t stack_words = 5120) {
    hal_xTaskCreate(task, "mf_INS", stack_words, this, priority, &task_handle, core);
  }

  UBaseType_t stack_high_water_words() { return task_handle ? uxTaskGetStackHighWaterMark(task_handle) : 0; }

private:
  nav_suite_t ns; // lives inside the object: declare the InsFusion instance as a global, never on a task stack
  MsgSubscription<ImuState>* imu_sub = nullptr;
  TaskHandle_t task_handle = nullptr;
  bool started = false, have_t_prev = false, mag_model_set = false;
  int64_t t_prev64 = 0, t_unwrapped = 0;
  uint32_t t_last32 = 0, last_mag_ts = 0, last_bar_ts = 0, last_gps_itow = 0xFFFFFFFFu;
  bool unwrap_init = false;
  float acc_sum[3] = {0}, gyr_sum[3] = {0};
  uint32_t n_acc = 0;
  float dt_mean_us = 0;

  int64_t unwrap(uint32_t t) {
    if (!unwrap_init) { t_unwrapped = t; t_last32 = t; unwrap_init = true; return t_unwrapped; }
    t_unwrapped += (int32_t)(t - t_last32);
    t_last32 = t;
    return t_unwrapped;
  }
};

//---------------------------------------------------------------------------------
// madflight AHR gizmo: lets INSLIB act as madflight's attitude estimator.
//
// Ahr::update() (called from imu_loop() at the IMU rate) calls AhrGizmoIns::update(), which feeds the raw IMU sample to
// InsFusion and copies the INS quaternion into ahr.q. madflight then derives ahr.roll/pitch/yaw from it as usual, so the
// rest of madflight (PID, logging, CLI 'pahr') sees the INSLIB attitude.
//
// Use in main.cpp, AFTER madflight_setup() and AFTER insf.setup():
//     ahr.gizmo = new AhrGizmoIns(&ahr, &insf);   // keep imu_loop() { ahr.update(); }
// Do NOT also call insf.step() or start_task() in that case: the IMU samples would be fed twice.
// The old gizmo is deliberately not deleted: the IMU task may be inside it right now.
//---------------------------------------------------------------------------------
class AhrGizmoIns : public AhrGizmo {
public:
  AhrGizmoIns(Ahr* ahr, InsFusion* ins) : ahr_(ahr), ins_(ins) {}

  bool update() override {
    Imu* i = ahr_->config.pimu; // raw (calibrated, unfiltered) IMU sample
    ins_->feed(i->ax, i->ay, i->az, i->gx, i->gy, i->gz, i->ts);
    if (ins_->state.att_valid) for (int k = 0; k < 4; k++) ahr_->q[k] = ins_->state.q[k];
    return true;
  }

  void setInitalOrientation(float* qnew) override { (void)qnew; } // INSLIB initialises itself (auto_init)

private:
  Ahr* ahr_;
  InsFusion* ins_;
};

extern InsFusion insf; // define once in the sketch: InsFusion insf;