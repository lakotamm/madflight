/*#########################################################################################################################

This is cpp file for variometer based on a 5 port pitot tube

Upload, connect Serial Monitor at 115200 baud and send 'help' to see available commands

See http://madflight.com for detailed description

MIT license - Copyright (c) 2023-2026 https://madflight.com
##########################################################################################################################*/

#include "main.h"
#include <madflight.h> // Include the library, do this after madflight_config 
#include "wind.h" //adjust path to wherever you place wind.h/wind.cpp

ScheduleFreq wind_print_schedule = ScheduleFreq(5); //scheduled sensor print to 5Hz
 
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
  // Nothing to do here for madflight, you can add your code here.
  delay(1000); //this delay() prevents empty loop wasting processor time, give this time to other tasks
}
 
// This function is called from the IMU task when fresh IMU data is available.
void imu_loop() {
  // Toggle led on every 1000 samples (E.g. 1 second period at 1000Hz sample rate)
  if(imu.update_cnt % 1000 == 0) led.toggle();
 
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
 
  ScheduleFreq wind_schedule = ScheduleFreq(1000); //target 1000Hz read rate
 
  float dt_min = 1e9f, dt_max = 0, dt_sum = 0;
  uint32_t dt_count = 0;
 
  for (;;) {
    if (wind_schedule.expired()) {
      wind.update();
 
      //accumulate dt stats for this print window
      if (wind.dt < dt_min) dt_min = wind.dt;
      if (wind.dt > dt_max) dt_max = wind.dt;
      dt_sum += wind.dt;
      dt_count++;
 
      if (wind_print_schedule.expired()) {
        float dt_avg = (dt_count > 0) ? (dt_sum / dt_count) : 0;
        Serial.printf(
          "[WIND] center dp=%7.2fPa t=%5.1fC (%s)  topbot dp=%7.2fPa (%s)  leftright dp=%7.2fPa (%s)  "
          "dt min/avg/max=%.4f/%.4f/%.4f n=%lu\n",
          wind.center.dp, wind.center.temp, wind.center.valid ? "ok" : "FAIL",
          wind.topbot.dp, wind.topbot.valid ? "ok" : "FAIL",
          wind.leftright.dp, wind.leftright.valid ? "ok" : "FAIL",
          dt_min, dt_avg, dt_max, (unsigned long)dt_count
        );
        //reset stats for next window
        dt_min = 1e9f; dt_max = 0; dt_sum = 0; dt_count = 0;
      }
    }
    portYIELD(); //let other tasks on this core run
  }
}
 
void wind_task_start() {
  hal_xTaskCreate(wind_task, "mf_WIND", MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, uxTaskPriorityGet(NULL), NULL, 0);
}
