/*#########################################################################################################################
This is cpp file for variometer based on a 5 port pitot tube
Upload, connect Serial Monitor at 115200 baud and send 'help' to see available commands
See http://madflight.com for detailed description
MIT license - Copyright (c) 2023-2026 https://madflight.com
##########################################################################################################################*/
#include "main.h"      // MF_BOARD + madflight_config - must come BEFORE madflight.h
#include <madflight.h> // Include the library, do this after madflight_config
#include "mf_print.h"  // Safe printf
#include "wind.h"      // wind measurement
#include "blelink.h"       // bluetooth reporting

#include "hardware/watchdog.h"

MfPrint mfSerial(Serial); //safe printing to serial

// --- forward declarations, only needed because setup()/loop() (below) call/use
//     these before their definitions appear later in this same file ---
void wind_task(void *pvParameters);
void wind_task_start();

// Shared dt diagnostics for wind task
volatile float wind_dt_min = 1e9f, wind_dt_max = 0, wind_dt_sum = 0;
volatile uint32_t wind_dt_count = 0;

  // Accumulate for averaging
  static float sum_bar_height = 0;
  static float sum_ahr_accel  = 0;
  static float sum_speed_up   = 0;
  static uint32_t sample_count = 0;

extern "C" void hardfault_c(uint32_t *sp) {
  watchdog_hw->scratch[0] = 0xDEADFA17;
  watchdog_hw->scratch[1] = sp[6];                          // stacked PC
  watchdog_hw->scratch[2] = sp[5];                          // stacked LR
  watchdog_hw->scratch[3] = *(volatile uint32_t*)0xE000ED28; // CFSR (Cortex-M33 / RP2350 only)
  watchdog_reboot(0, 0, 10);
  while (1) {}
}

extern "C" __attribute__((naked)) void isr_hardfault(void) {
  __asm volatile(
    "tst lr, #4 \n"
    "ite eq \n"
    "mrseq r0, msp \n"
    "mrsne r0, psp \n"
    "b hardfault_c \n"
  );
}

void printAltEstimator(){
    // Read vertical measurements
  float bar_height       = bar.alt;
  float ahr_acceleration = ahr.getAccelUp();
  float speed_up         = alt.getV();

  sum_bar_height += bar_height;
  sum_ahr_accel  += ahr_acceleration;
  sum_speed_up   += speed_up;
  sample_count++;

  // Print average every 10 samples
  if (sample_count >= 10) {
    mfSerial.printf(
      "Bar height: %8.2f m | AHR vertical accel: %8.3f m/s^2 | Alt vertical speed: %8.2f m/s\n",
      sum_bar_height / sample_count,
      sum_ahr_accel / sample_count,
      sum_speed_up / sample_count
    );

    sum_bar_height = 0;
    sum_ahr_accel  = 0;
    sum_speed_up   = 0;
    sample_count   = 0;
  }
}

void printWindSensors(){
   WindState w;
   if (wind.topic.pull_latest(&w)) {
     //snapshot + reset the dt stats (simple scalars, no lock needed - see wind_task)
     float dt_min = wind_dt_min, dt_max = wind_dt_max, dt_sum = wind_dt_sum;
     uint32_t dt_count = wind_dt_count;
     wind_dt_min = 1e9f; wind_dt_max = 0; wind_dt_sum = 0; wind_dt_count = 0;
     float dt_avg = (dt_count > 0) ? (dt_sum / dt_count) : 0;
     mfSerial.printf(
       "[WIND] center dp=%7.2fPa t=%5.1fC (%s)  topbot dp=%7.2fPa (%s)  leftright dp=%7.2fPa (%s)  "
       "dt min/avg/max=%.4f/%.4f/%.4f n=%lu\n",
       w.center.dp, w.center.temp, w.center.valid ? "ok" : "FAIL",
       w.topbot.dp, w.topbot.valid ? "ok" : "FAIL",
       w.leftright.dp, w.leftright.valid ? "ok" : "FAIL",
       dt_min, dt_avg, dt_max, (unsigned long)dt_count
     );
  }
}

void setup() {
  // Setup madflight modules, start madflight RTOS tasks, Serial.begin(11520)
  madflight_setup();

  // Print out it there has been a hard fault and 
  if (watchdog_hw->scratch[0] == 0xDEADFA17) {
  Serial.printf("HARDFAULT pc=%08lx lr=%08lx cfsr=%08lx\n",
    (unsigned long)watchdog_hw->scratch[1], (unsigned long)watchdog_hw->scratch[2], (unsigned long)watchdog_hw->scratch[3]);
  watchdog_hw->scratch[0] = 0;
}

  // --- pressure probe: config + start the SDP3x array, then spawn the wind task ---
  wind.config.i2c_bus = hal_get_i2c_bus(1); //I2C1 - matches bus used for sensor bring-up
  if (wind.setup() != 0) {
    Serial.println("[WIND] setup FAILED - check wiring/addresses");
  } else {
    Serial.println("[WIND] setup OK");
  }
  wind_task_start();
  blelink_task_start();

  Serial.println("Setup completed, CLI started - Type 'help' for help, or 'diff' to debug");
}

void loop() {
  //mostly used for printing
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10); //100Hz target

  printAltEstimator(); 

  //printWindSensors();

  vTaskDelayUntil(&lastWakeTime, period);
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

  // vTaskDelayUntil() targets an absolute future tick

  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(2); //500Hz target

  for (;;) {
    wind.update(); //reads + publishes to wind.topic - no Serial I/O here, ever

    //accumulate dt stats only - no printing in this task
    if (wind.dt < wind_dt_min) wind_dt_min = wind.dt;
    if (wind.dt > wind_dt_max) wind_dt_max = wind.dt;
    wind_dt_sum += wind.dt;
    wind_dt_count++;


    //If the hardware of the sensor board is alright, the reading should be done in less than 2ms/2ticks. 
    //However, in case there is some form of a failure, we still need to give other threads time to run

    if(xTaskGetTickCount()- lastWakeTime >=2){
      vTaskDelay(1); // prioritise giving other threads time to run in case of a failure
    } else {
    vTaskDelayUntil(&lastWakeTime, period); // in most cases, run without a delay
    }
  }
}

void wind_task_start() {
  hal_xTaskCreate(wind_task, "mf_WIND", MF_FREERTOS_DEFAULT_STACK_SIZE, NULL, uxTaskPriorityGet(NULL) + 1, NULL, 0);
}