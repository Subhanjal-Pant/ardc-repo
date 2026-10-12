#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>
#include "policy_infer.h"   // provides policy_forward(), OBS_DIM, ACTION_DIM, ACTION_LOW/HIGH

// --- Pin Definitions (confirmed from working fuzzy firmware) ---
const int enA = 25; const int in1 = 16; const int in2 = 17; // Exhaust Valve
const int enB = 26; const int in3 = 19; const int in4 = 18; // Inlet Valve
const int inputPin = 33;   // supply/tank pressure (P_In)
const int springPin = 32;  // spring pressure (P_Sp)

// --- Objects ---
VL53L0X sensor;
MPU6050 imu(0x68);

// --- Pressure calibration (confirmed from working fuzzy firmware) ---
// Formula outputs GAUGE bar -- confirmed via the fuzzy controller's
// exhaust-side logic expecting pSpringBar to range [-0.3, 1.2] bar near
// vented, which only makes sense for gauge pressure.
const float CAL_SLOPE = 0.001845f;
const float CAL_OFFSET = -1.1914f;
const float P_ATM = 101325.0f;  // Pa, matches gauge_bar_to_abs_pa() in pneumatic_env.py

// --- Physical bounds, matching pneumatic_env.py exactly ---
const float HEIGHT_MIN_M = 0.140f;
const float HEIGHT_MAX_M = 0.230f;
const float E_MAX = 0.050f;
const float V_MAX = 0.500f;
const float P_MIN_PA = 101325.0f;
const float P_MAX_PA = 600000.0f;
const float P_SUPPLY_MIN_PA = 200000.0f;
const float P_SUPPLY_MAX_PA = 500000.0f;

// --- PWM / duty-cycle constants, matching pneumatic_env.py exactly ---
const float PWM_CYCLE_TIME_MS = 143.0f;  // ~7 Hz base period, same as fuzzy firmware's CYCLE_TIME
const float MIN_DUTY = 0.20f;
const float ACTION_DEADZONE = 0.05f;

// --- Control loop timing: 25 Hz (40ms), matching training dt and fuzzy firmware's LOOP_INTERVAL ---
const unsigned long LOOP_INTERVAL = 40;
unsigned long lastLoopTime = 0;

// --- IMU/ToF Sensor Fusion (2-state Kalman filter: [height, velocity]) ---
// Prediction step is driven by IMU vertical acceleration (kinematic
// integration between ToF samples); correction step uses the ToF range.
// This replaces differentiating a filtered position signal (which was
// amplifying noise into velocity, same class of problem flagged during
// Cd system-ID) with a filter that estimates velocity as a genuine state,
// consistent with how pneumatic_env.py's training velocity comes straight
// from the physics integrator rather than a finite difference.

float kf_height_m = 0.185f;   // state[0]
float kf_velocity_m_s = 0.0f; // state[1]
float kf_P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};  // state covariance

// Tuning: sigma_a is acceleration process noise (m/s^2), sigma_h is ToF
// measurement noise (m). Start here, retune against logged data if the
// estimate looks over/under-smoothed.
const float KF_SIGMA_A = 0.8f;      // accel uncertainty, m/s^2
const float KF_SIGMA_H = 0.003f;    // ToF measurement noise, m (3mm)

// Accelerometer bias, in raw LSB, at rest (vertical axis). Set this from
// your own static calibration routine before the control loop starts.
float accelBiasZ_lsb = 0.0f;  // TODO: set via your calibration step

// MPU6050 default range +/-2g -> 16384 LSB/g
const float ACCEL_LSB_PER_G = 16384.0f;
const float GRAVITY_MPS2 = 9.81f;

// Convert raw az (with bias removed) to free vertical acceleration in m/s^2.
// Assumes az's "at rest" reading corresponds to 1g on this axis (pure
// vertical translation, no tilt) -- bias calibration should capture that.
float rawAccelToMps2(int16_t az_raw) {
  float az_corrected_lsb = (float)az_raw - accelBiasZ_lsb;
  return (az_corrected_lsb / ACCEL_LSB_PER_G) * GRAVITY_MPS2;
}

// Predict step: propagate [height, velocity] using acceleration as input,
// matching a constant-acceleration process model.
void kfPredict(float accel_mps2, float dt) {
  // State propagation
  float h_pred = kf_height_m + kf_velocity_m_s * dt + 0.5f * accel_mps2 * dt * dt;
  float v_pred = kf_velocity_m_s + accel_mps2 * dt;

  // Covariance propagation: P = F*P*F^T + Q
  // F = [[1, dt], [0, 1]]
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;
  float sigma_a2 = KF_SIGMA_A * KF_SIGMA_A;

  float Q00 = 0.25f * dt4 * sigma_a2;
  float Q01 = 0.5f * dt3 * sigma_a2;
  float Q11 = dt2 * sigma_a2;

  float P00 = kf_P[0][0], P01 = kf_P[0][1], P10 = kf_P[1][0], P11 = kf_P[1][1];

  float FP00 = P00 + dt * P10;
  float FP01 = P01 + dt * P11;
  float FP10 = P10;
  float FP11 = P11;

  float newP00 = FP00 + dt * FP01 + Q00;
  float newP01 = FP01 + Q01;
  float newP10 = FP10 + dt * FP11 + Q01;
  float newP11 = FP11 + Q11;

  kf_height_m = h_pred;
  kf_velocity_m_s = v_pred;
  kf_P[0][0] = newP00; kf_P[0][1] = newP01;
  kf_P[1][0] = newP10; kf_P[1][1] = newP11;
}

// Correct step: fuse in a ToF height measurement (meters).
void kfCorrect(float measured_height_m) {
  float R = KF_SIGMA_H * KF_SIGMA_H;

  // Innovation
  float y = measured_height_m - kf_height_m;

  // Innovation covariance: S = H*P*H^T + R, H = [1, 0]
  float S = kf_P[0][0] + R;

  // Kalman gain: K = P*H^T / S
  float K0 = kf_P[0][0] / S;
  float K1 = kf_P[1][0] / S;

  kf_height_m += K0 * y;
  kf_velocity_m_s += K1 * y;

  // Covariance update: P = (I - K*H) * P
  float P00 = kf_P[0][0], P01 = kf_P[0][1], P10 = kf_P[1][0], P11 = kf_P[1][1];
  kf_P[0][0] = P00 - K0 * P00;
  kf_P[0][1] = P01 - K0 * P01;
  kf_P[1][0] = P10 - K1 * P00;
  kf_P[1][1] = P11 - K1 * P01;
}

// --- State carried between control steps (mirrors pneumatic_env.py's
//     self.current_valve_state / self.last_duty_cmd) ---
float targetHeight_m = 0.185f;   // default target, matches env's default
float prevHeight_m = 0.185f;
float lastDutyCmd = 0.0f;
float valveState0 = 0.0f;   // inlet (S1) applied last step
float valveState1 = 0.0f;   // exhaust (S2) applied last step

// Velocity, now sourced directly from the fusion filter's velocity state
// (see kfPredict/kfCorrect below) rather than a finite-difference + EMA.
float smoothedVelocity_m_s = 0.0f;

// Safety bounds (match pneumatic_env.py's terminated condition, 5mm margin)
const float SAFETY_MIN_M = HEIGHT_MIN_M - 0.005f;
const float SAFETY_MAX_M = HEIGHT_MAX_M + 0.005f;

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  Wire.begin(21, 22);

  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  analogReadResolution(12);

  if (!sensor.init()) {
    Serial.println("VL53L0X Fail");
    while (1);
  }
  sensor.setTimeout(500);
  sensor.startContinuous();
  imu.initialize();

  Serial.println("PPO Height Control Initialized. Send T<value in mm> to set target.");
  lastLoopTime = millis();
}

void loop() {
  // 1. Target update, same input protocol as fuzzy firmware (e.g. "T185")
  if (Serial.available() > 0) {
    char type = Serial.read();
    float val = Serial.parseFloat();
    if (type == 'T') targetHeight_m = val / 1000.0f;
    while (Serial.available() > 0) Serial.read();
  }

  unsigned long now = millis();
  if (now - lastLoopTime < LOOP_INTERVAL) {
    runBinaryPulsing(valveState0 > 0.5f, valveState1 > 0.5f, lastDutyCmd);
    return;
  }
  lastLoopTime = now;

  // 2. IMU read -- now used for fusion prediction, not just telemetry
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float accel_mps2 = rawAccelToMps2(az);

  // 3. Fusion predict step (every tick, using latest IMU sample)
  kfPredict(accel_mps2, LOOP_INTERVAL / 1000.0f);

  // 4. Height sensor -- ToF is the correction/measurement input to the filter
  uint16_t rawDist = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) {
    Serial.println("SENSOR TIMEOUT");
    valveState0 = 0.0f; valveState1 = 0.0f;
    runBinaryPulsing(false, false, 0.0f);
    return;
  }
  kfCorrect(rawDist / 1000.0f);

  float currentHeight_m = kf_height_m;
  smoothedVelocity_m_s = kf_velocity_m_s;  // KF velocity state, no separate smoothing needed
  prevHeight_m = currentHeight_m;

  // 5. Pressure readings (confirmed pins/calibration from fuzzy firmware)
  int pInRaw = analogRead(inputPin);
  int pSpRaw = analogRead(springPin);
  float supplyPressure_pa = (pInRaw * CAL_SLOPE + CAL_OFFSET) * 1e5f + P_ATM;
  float springPressure_pa = (pSpRaw * CAL_SLOPE + CAL_OFFSET) * 1e5f + P_ATM;

  // 6. Safety cutoff -- mirrors pneumatic_env.py's terminated condition,
  //    independent of what the policy outputs.
  if (currentHeight_m <= SAFETY_MIN_M || currentHeight_m >= SAFETY_MAX_M) {
    Serial.println("SAFETY LIMIT HIT -- valves closed, PPO output ignored this step.");
    valveState0 = 0.0f; valveState1 = 0.0f; lastDutyCmd = 0.0f;
    runBinaryPulsing(false, false, 0.0f);
    return;
  }

  // 7. Build the 8-dim observation vector, EXACTLY matching _get_obs() in
  //    pneumatic_env.py -- same order, same normalization, same clipping.
  //    NOTE: entries are normalized (0-1 or -1..1), not raw physical units.
  float error_m = targetHeight_m - currentHeight_m;

  float norm_h = (currentHeight_m - HEIGHT_MIN_M) / (HEIGHT_MAX_M - HEIGHT_MIN_M);
  float norm_v = smoothedVelocity_m_s / V_MAX;
  float norm_p = (springPressure_pa - P_MIN_PA) / (P_MAX_PA - P_MIN_PA);
  float norm_err = error_m / E_MAX;
  float norm_last_duty = clampf(lastDutyCmd, -1.0f, 1.0f);
  float norm_p_supply = (supplyPressure_pa - P_SUPPLY_MIN_PA) / (P_SUPPLY_MAX_PA - P_SUPPLY_MIN_PA);

  float obs[OBS_DIM] = {
    clampf(norm_h, 0.0f, 1.0f),
    clampf(norm_v, -1.0f, 1.0f),
    clampf(norm_p, 0.0f, 1.0f),
    clampf(norm_err, -1.0f, 1.0f),
    valveState0,
    valveState1,
    norm_last_duty,
    clampf(norm_p_supply, 0.0f, 1.0f)
  };

  // 8. Run the policy
  float action_out[ACTION_DIM];
  policy_forward(obs, action_out);
  float duty_cmd = action_out[0];  // already clamped to ACTION_LOW/HIGH = [-1, 1]
  lastDutyCmd = duty_cmd;

  // 9. Resolve duty_cmd into this tick's binary valve state, EXACTLY
  //    matching _resolve_pwm_valve_state() in pneumatic_env.py:
  //    deadzone -> min-duty floor -> 143ms phase-based pulsing.
  bool inflateOn = false;
  bool exhaustOn = false;

  if (fabsf(duty_cmd) >= ACTION_DEADZONE) {
    float duty_fraction = fabsf(duty_cmd);
    if (duty_fraction < MIN_DUTY) duty_fraction = MIN_DUTY;
    if (duty_fraction > 1.0f) duty_fraction = 1.0f;

    float phase = fmodf((float)millis(), PWM_CYCLE_TIME_MS);
    float onTime = duty_fraction * PWM_CYCLE_TIME_MS;
    bool valveOpen = phase < onTime;

    if (valveOpen) {
      if (duty_cmd > 0.0f) inflateOn = true;
      else exhaustOn = true;
    }
  }

  valveState0 = inflateOn ? 1.0f : 0.0f;
  valveState1 = exhaustOn ? 1.0f : 0.0f;

  runBinaryPulsing(inflateOn, exhaustOn, duty_cmd);

  // 10. CSV telemetry -- matches your standard hardware-log schema:
  //     PC_Time, ESP_MS, Target, Height, Status, P_In_Raw, P_Sp_Raw, ax,ay,az, gx,gy,gz
  //     Status = signed PWM duty (-100..100; positive=inlet, negative=exhaust)
  int statusPct = (int)(duty_cmd * 100.0f);
  Serial.print(now); Serial.print(",");
  Serial.print(targetHeight_m * 1000.0f, 0); Serial.print(",");
  Serial.print(currentHeight_m * 1000.0f, 1); Serial.print(",");
  Serial.print(statusPct); Serial.print(",");
  Serial.print(pInRaw); Serial.print(",");
  Serial.print(pSpRaw); Serial.print(",");
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.println(gz);
}

// --- Valve pulsing: EXACTLY matches _resolve_pwm_valve_state() phase logic,
//     called every loop() tick (not just every 40ms) so the valve state
//     stays correctly asserted between control-step updates, same pattern
//     as the fuzzy firmware's runBinaryPulsing(). ---
void runBinaryPulsing(bool inflateOn, bool exhaustOn, float duty_cmd) {
  if (inflateOn) {
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW); digitalWrite(enB, HIGH);
    digitalWrite(enA, LOW);
  } else if (exhaustOn) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); digitalWrite(enA, HIGH);
    digitalWrite(enB, LOW);
  } else {
    digitalWrite(enA, LOW); digitalWrite(enB, LOW);
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  }
}
