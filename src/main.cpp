/*#########################################################################################################################
This is cpp file for variometer based on a 5 port pitot tube
Upload, connect Serial Monitor at 115200 baud and send 'help' to see available commands
See http://madflight.com for detailed description
MIT license - Copyright (c) 2023-2026 https://madflight.com
##########################################################################################################################*/
#include "main.h"      // MF_BOARD + madflight_config - must come BEFORE madflight.h
#include <madflight.h> // Include the library, do this after madflight_config
#include "wind.h"      // adjust path to wherever you place wind.h/wind.cpp

// --- forward declarations, only needed because setup()/loop() (below) call/use
//     these before their definitions appear later in this same file ---
void wind_task(void *pvParameters);
void wind_task_start();

ScheduleFreq wind_print_schedule = ScheduleFreq(10); //scheduled sensor print to 10Hz

// Shared dt diagnostics - simple scalars, read/reset from loop() without a lock.
// Each is independently valid even under a rare torn read (same tolerance the
// framework itself relies on for e.g. GPS lat/lon) - fine for a diagnostic print.
volatile float wind_dt_min = 1e9f, wind_dt_max = 0, wind_dt_sum = 0;
volatile uint32_t wind_dt_count = 0;

void setup() {
  // Setup madflight modules, start madflight RTOS tasks, Serial.begin(11520)
  madflight_setup();

  // --- pressure probe: config + start the SDP3x array, then spawn the wind task ---
  wind.config.i2c_bus = hal_get_i2c_bus(1); //I2C1 - matches bus used for sensor bring-up
  if (wind.setup() != 0) {
    Serial.println("[WIND] setup FAILED - check wiring/addresses");
  } else {
    Serial.println("[WIND] setup OK");
  }
  wind_task_start();

  Serial.println("Setup completed, CLI started - Type 'help' for help, or 'diff' to debug");
}

void loop() {
  // Printing lives here, not in wind_task - Serial I/O can block for a few ms
  // without touching the 500Hz read loop, since this runs as its own thing.
  if (wind_print_schedule.expired()) {
    WindState w;
    if (wind.topic.pull_latest(&w)) {
      //snapshot + reset the dt stats (simple scalars, no lock needed - see wind_task)
      float dt_min = wind_dt_min, dt_max = wind_dt_max, dt_sum = wind_dt_sum;
      uint32_t dt_count = wind_dt_count;
      wind_dt_min = 1e9f; wind_dt_max = 0; wind_dt_sum = 0; wind_dt_count = 0;
      float dt_avg = (dt_count > 0) ? (dt_sum / dt_count) : 0;

      Serial.printf(
        "[WIND] center dp=%7.2fPa t=%5.1fC (%s)  topbot dp=%7.2fPa (%s)  leftright dp=%7.2fPa (%s)  "
        "dt min/avg/max=%.4f/%.4f/%.4f n=%lu\n",
        w.center.dp, w.center.temp, w.center.valid ? "ok" : "FAIL",
        w.topbot.dp, w.topbot.valid ? "ok" : "FAIL",
        w.leftright.dp, w.leftright.valid ? "ok" : "FAIL",
        dt_min, dt_avg, dt_max, (unsigned long)dt_count
      );

      Serial.printf("AHR Accel in earth frame %7.2f \n", ahr.getAccelUp());
    }
  }
  vTaskDelay(20); //cheap idle - print schedule (5Hz) still governs actual output rate
}

// This function is called from the IMU task when fresh IMU data is available.
void imu_loop() {
  // Toggle led on every 800 samples (E.g. 1 second period at 800Hz sample rate)
  if(imu.update_cnt % 800 == 0) led.toggle();

  // AHRS sensor fusion - type 'pahr' in CLI to see results
  ahr.update();
}

//=====================================================================
// Pressure probe (5-hole pitot) task - core 0, alongside mf_SENSOR.
// Phase 1: raw read + print only. Phase 2 (Cp lookup, rotation using
// ahr.roll/ahr.pitch) slots into wind.update() without changing this task.
//=====================================================================

void wind_task(void *pvParameters) {
  (void)pvParameters;

  // vTaskDelayUntil() targets an absolute future tick, not "now + period" -
  // this is what actually fixed the jitter, not the busy-poll/notification
  // schemes tried earlier. See surrounding discussion for why.
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(2); //500Hz target

  for (;;) {
    wind.update(); //reads + publishes to wind.topic - no Serial I/O here, ever

    //accumulate dt stats only - no printing in this task
    if (wind.dt < wind_dt_min) wind_dt_min = wind.dt;
    if (wind.dt > wind_dt_max) wind_dt_max = wind.dt;
    wind_dt_sum += wind.dt;
    wind_dt_count++;

    vTaskDelayUntil(&lastWakeTime, period);
  }
}

void wind_task_start() {
  hal_xTaskCreate(wind_task, "mf_WIND", MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, uxTaskPriorityGet(NULL) + 1, NULL, 0);
}