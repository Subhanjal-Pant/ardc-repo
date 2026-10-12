#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>

// --- Pin Definitions (same as adrc_height_control_strict.ino) ---
const int enA = 26; const int in1 = 16; const int in2 = 17; // Exhaust Valve
const int enB = 25; const int in3 = 18; const int in4 = 19; // Inlet Valve
const int inputPin = 33;
const int springPin = 32;

// --- Objects ---
VL53L0X sensor;
MPU6050 imu(0x68);

unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL = 40; // 25 Hz, same as the controller

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // Valves forced closed for the whole test
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  digitalWrite(enA, LOW); digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(enB, LOW); digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  analogReadResolution(12);

  if (!sensor.init()) {
    Serial.println("VL53L0X Fail");
    while (1);
  }
  sensor.setTimeout(500);
  sensor.startContinuous();
  imu.initialize();

  // CSV header (one line, starts with # so it is easy to skip when loading)
  Serial.println("#ms,rawDist,ax,ay,az,gx,gy,gz,pIn,pSp");

  lastLoopTime = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastLoopTime >= LOOP_INTERVAL) {
    lastLoopTime = now;

    uint16_t rawDist = sensor.readRangeContinuousMillimeters();
    int pInRaw = analogRead(inputPin);
    int pSpRaw = analogRead(springPin);

    int16_t ax, ay, az, gx, gy, gz;
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Serial.print(now); Serial.print(",");
    Serial.print(rawDist); Serial.print(",");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print(",");
    Serial.print(pInRaw); Serial.print(",");
    Serial.println(pSpRaw);
  }
}
