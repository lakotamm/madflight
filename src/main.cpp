#include "main.h"      // MF_BOARD + madflight_config - must come BEFORE madflight.h
#include <madflight.h> // Include the library, do this after madflight_config
#include "ins_fusion.h"

InsFusion insf;

// --- forward declarations, only needed because setup()/loop() (below) call/use
//     these before their definitions appear later in this same file ---
void wind_task(void *pvParameters);
void wind_task_start();


  // Accumulate for averaging
  static float sum_bar_height = 0;
  static float sum_insf_height = 0;
  static float sum_speed_up   = 0;
  static float sum_insf_speed_up = 0;
  static float sum_insf_baro_speed_up = 0;

  static uint32_t sample_count = 0;


void printAltEstimator(){
    // Read vertical measurements
  float bar_height       = bar.alt;
  float insf_height      = insf.getH();
  float speed_up         = alt.getV();
  float height           = alt.getH();
  float insf_speed_up    = insf.state.vel_up;
  float insf_baro_speed_up = insf.state.baro_v_up;

  sum_bar_height += bar_height;
  sum_speed_up   += speed_up;
  sum_insf_speed_up += insf_speed_up;
  sum_insf_height += insf_height;
  sum_insf_baro_speed_up += insf_baro_speed_up;
  sample_count++;

  // Print average every 10 samples
  if (sample_count >= 10) {
    Serial.printf(
      "Bar height %6.2f m | INS(height) %6.2f m | Alt vert speed: %6.2f m/s |INS(baro) %6.2f v=%d | INS(gnss) %6.2f ready=%d mode=%d sats=%d\n",
      sum_bar_height / sample_count,
      sum_insf_height / sample_count,
      sum_speed_up / sample_count,
      sum_insf_baro_speed_up / sample_count, 
      insf.state.baro_v_up_valid,
      sum_insf_speed_up / sample_count,  
      insf.state.ready, 
      insf.state.mode, 
      gps.sat
    );

    sum_bar_height = 0;
    sum_insf_height = 0;
    sum_speed_up   = 0;
    sum_insf_speed_up = 0;
    sum_insf_baro_speed_up = 0;
    sample_count   = 0;
  }
}

void setup() {
  // Setup madflight modules, start madflight RTOS tasks, Serial.begin(11520)
  madflight_setup();

  insf.setup();
  ahr.gizmo = new AhrGizmoIns(&ahr, &insf);

  Serial.println("Setup completed, CLI started - Type 'help' for help, or 'diff' to debug");
}

void loop() {
  //mostly used for printing
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10); //100Hz target

  printAltEstimator(); 

  vTaskDelayUntil(&lastWakeTime, period);
}

// This function is called from the IMU task when fresh IMU data is available.
void imu_loop() {
  // Toggle led on every 800 samples (E.g. 1 second period at 800Hz sample rate)
  if(imu.update_cnt % 400 == 0) led.toggle();

  // AHRS sensor fusion - type 'pahr' in CLI to see results
  ahr.update();
}