#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>

// --- Pins ---
const int enA = 26; const int in1 = 16; const int in2 = 17; // Exhaust Valve
const int enB = 25; const int in3 = 18; const int in4 = 19; // Inlet Valve
const int inputPin = 33;
const int springPin = 32;

// --- Objects ---
VL53L0X sensor;
MPU6050 imu(0x68);

// --- Variables ---
float targetHeight = 130.0;
float currentHeight = 0.0;
const float DEADZONE = 4.0;           // Kept at 4mm as requested
const unsigned long CYCLE_TIME = 143; // 7Hz Valve Pulsing
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL = 40; // 25Hz Refresh Rate

// Global control variables
int sysDutyCycle = 0;
int sysStatusFlag = 0; // 0:Idle, 1:Filling, 2:Draining

// --- IMU (vertical axis) ---
// az is used as the vertical axis. 
// ACC_SIGN: +1 if a positive reading toward INCREASING height, otherwise -1.
const float ACC_SIGN = 1.0;
const float ACC_SCALE = 9806.65 / 16384.0; // mm/s^2 per LSB (default +/-2 g range)
float accRest = 0.0;                        // vertical reading at rest (gravity + bias), measured in setup()

// --- Kalman Filter ---
// State x = [height (mm), velocity (mm/s), accelerometer bias (mm/s^2)]
// Prediction uses the IMU acceleration, update uses the ToF height.
const float KF_R_TOF  = 16.0;     // ToF noise variance (mm^2)
const float KF_Q_ACC  = 90000.0;  // IMU noise variance ((mm/s^2)^2)
const float KF_Q_BIAS = 25.0;     // bias random walk ((mm/s^2)^2 per s)
float XK[3];
float PK[3][3];

void kalmanInit(float h0) {
  XK[0] = h0; XK[1] = 0.0; XK[2] = 0.0;
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) PK[i][j] = 0.0;
  PK[0][0] = KF_R_TOF; PK[1][1] = 100.0; PK[2][2] = 100.0;
}

void kalmanPredict(float aMeas, float dt) {
  float a = aMeas - XK[2];
  float dt2 = 0.5 * dt * dt;
  XK[0] += XK[1] * dt + dt2 * a;
  XK[1] += dt * a;

  float F[3][3] = {{1.0, dt, -dt2}, {0.0, 1.0, -dt}, {0.0, 0.0, 1.0}};
  float FP[3][3], Pn[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      FP[i][j] = F[i][0] * PK[0][j] + F[i][1] * PK[1][j] + F[i][2] * PK[2][j];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      Pn[i][j] = FP[i][0] * F[j][0] + FP[i][1] * F[j][1] + FP[i][2] * F[j][2];
  float g[3] = {dt2, dt, 0.0};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) Pn[i][j] += KF_Q_ACC * g[i] * g[j];
  Pn[2][2] += KF_Q_BIAS * dt;
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) PK[i][j] = Pn[i][j];
}

void kalmanUpdate(float zHeight) {
  float S = PK[0][0] + KF_R_TOF;
  float K[3] = {PK[0][0] / S, PK[1][0] / S, PK[2][0] / S};
  float innov = zHeight - XK[0];
  for (int i = 0; i < 3; i++) XK[i] += K[i] * innov;
  float row0[3] = {PK[0][0], PK[0][1], PK[0][2]};
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) PK[i][j] -= K[i] * row0[j];
}

// --- ADRC (second order) ---
// Plant model: y'' = b0*u + f   (y = height in mm, u = duty in -1..1, f = total disturbance)
// Identified b0 on the rig and tuned WC/WO
const float WC = 4.0;             // controller bandwidth (rad/s)
const float WO = 16.0;            // observer bandwidth (rad/s), typically 3-5 x WC
const float B0_INLET   = 200.0;   // mm/s^2 per unit duty, inflating
const float B0_EXHAUST = 200.0;   // mm/s^2 per unit duty, exhausting

// ESO states: z1 = height, z2 = velocity, z3 = total disturbance
float z1 = 0.0, z2 = 0.0, z3 = 0.0;
float uApplied = 0.0;             // duty the valves actually received, -1..1

void esoUpdate(float y, float uApp, float dt) {
  float bu = uApp * ((uApp >= 0.0) ? B0_INLET : B0_EXHAUST);
  const int SUB = 4;
  float h = dt / SUB;
  for (int s = 0; s < SUB; s++) {
    float e = y - z1;
    float d1 = z2 + 3.0 * WO * e;
    float d2 = z3 + bu + 3.0 * WO * WO * e;
    float d3 = WO * WO * WO * e;
    z1 += h * d1; z2 += h * d2; z3 += h * d3;
  }
}

// Returns the ADRC command as a signed percentage (-100..100)
int computeAdrc() {
  float u0 = WC * WC * (targetHeight - z1) + 2.0 * WC * (0.0 - z2);
  float cmd = u0 - z3;                       // cancel the estimated total disturbance
  float b0 = (cmd >= 0.0) ? B0_INLET : B0_EXHAUST;
  float u = constrain(cmd / b0, -1.0, 1.0);
  return (int)(u * 100.0);
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

  // Rest calibration (about 1 s): keep the platform still and the valves closed.
  // Measures the vertical IMU offset (gravity + bias) and the starting height.
  long aSum = 0;
  float hSum = 0.0;
  int hN = 0;
  for (int i = 0; i < 200; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    aSum += az;
    if (i % 20 == 0) {
      uint16_t d = sensor.readRangeContinuousMillimeters();
      if (d < 1000) { hSum += d; hN++; }
    }
    delay(5);
  }
  accRest = (float)aSum / 200.0;
  float h0 = (hN > 0) ? hSum / hN : targetHeight;

  kalmanInit(h0);
  currentHeight = h0;
  z1 = h0; z2 = 0.0; z3 = 0.0;
  lastLoopTime = millis();
}

void loop() {
  if (Serial.available() > 0) {
    char type = Serial.read();
    float val = Serial.parseFloat();
    if (type == 'T') targetHeight = val;
    while(Serial.available() > 0) Serial.read(); 
  }

  unsigned long now = millis();

  if (now - lastLoopTime >= LOOP_INTERVAL) {
    float dt = (now - lastLoopTime) * 0.001;
    if (dt > 0.1) dt = 0.1;
    lastLoopTime = now;

    uint16_t rawDist = sensor.readRangeContinuousMillimeters();
    int pInRaw = analogRead(inputPin);
    int pSpRaw = analogRead(springPin);

    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // ToF + IMU fusion (ToF timeouts/out-of-range reads skip the correction)
    float aMeas = ACC_SIGN * ((float)az - accRest) * ACC_SCALE;
    kalmanPredict(aMeas, dt);
    if (rawDist < 1000) kalmanUpdate((float)rawDist);
    currentHeight = XK[0];

    // ADRC: observer update with the duty actually applied, then control law
    esoUpdate(currentHeight, uApplied, dt);
    int adrcOutput = computeAdrc();

    float error = targetHeight - currentHeight;

    if (abs(error) > DEADZONE) {
      // COASTING: Turn off early to let momentum carry into the deadzone
      if (adrcOutput > 10) { 
        sysStatusFlag = 1;
        sysDutyCycle = constrain(adrcOutput, 15, 100);
      } else if (adrcOutput < -10) {
        sysStatusFlag = 2;
        sysDutyCycle = constrain(abs(adrcOutput), 15, 100);
      } else {
        sysStatusFlag = 0;
        sysDutyCycle = 0;
      }
    } else {
      sysStatusFlag = 0;
      sysDutyCycle = 0;
    }

    uApplied = (sysStatusFlag == 1) ? sysDutyCycle / 100.0
             : (sysStatusFlag == 2) ? -sysDutyCycle / 100.0 : 0.0;

    // CSV Telemetry output (Matches your original format exactly)
    Serial.print(now); Serial.print(",");
    Serial.print(targetHeight, 0); Serial.print(",");
    Serial.print(currentHeight, 1); Serial.print(",");
    Serial.print(sysStatusFlag == 2 ? -sysDutyCycle : sysDutyCycle); Serial.print(",");
    Serial.print(pInRaw); Serial.print(",");
    Serial.print(pSpRaw); Serial.print(",");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);
  }

  runBinaryPulsing(sysStatusFlag, sysDutyCycle);
}

void runBinaryPulsing(int status, int dc) {
  unsigned long elapsed = millis() % CYCLE_TIME;
  unsigned long onTime = (dc * CYCLE_TIME) / 100;
  bool valveOpen = (elapsed < onTime);

  if (status == 1 && valveOpen) { // Filling
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW); digitalWrite(enB, HIGH);
    digitalWrite(enA, LOW);
  } 
  else if (status == 2 && valveOpen) { // Draining
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); digitalWrite(enA, HIGH);
    digitalWrite(enB, LOW);
  } 
  else { // Off
    digitalWrite(enA, LOW); digitalWrite(enB, LOW);
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  }
}
