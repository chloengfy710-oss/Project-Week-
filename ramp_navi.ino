#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Motor pins
const int ENA_PIN = 3;
const int IN1_PIN = 2;
const int IN2_PIN = 1;
const int ENB_PIN = 11;
const int IN3_PIN = 12;
const int IN4_PIN = 13;

Adafruit_MPU6050 mpu;
sensors_event_t accelEvent, gyroEvent, tempEvent;

// State machine
enum State { DRIVE_TO_RAMP, CLIMBING_TO_TOP, AT_TOP_ROTATING, DESCENDING, FINISHED };
State state = DRIVE_TO_RAMP;

// thresholds
float RAMP_DETECT_ANGLE = 20.0; // deg
float TOP_FLAT = 4.0;            
float BOTTOM_FLAT = 2.0;
float PITCH_ALPHA = 0.28;
int REQ_CONSEC = 1;              
int ROT_PWM = 200;
float ROT_TOLERANCE_DEG = 8.0;
unsigned long ROT_TIMEOUT_MS = 90UL * 1000UL;

// gyro bias
float gyroZBiasDegPerSec = 0.0;
const unsigned long GYRO_CAL_MS = 1000;

// runtime
float pitchFiltered = 0.0;
float maxPitch = 0.0; // store maximum ramp angle

void setup() {
  Wire.begin();
  lcd.begin(16, 2);

  // motor pins
  pinMode(ENA_PIN, OUTPUT); pinMode(ENB_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT); pinMode(IN4_PIN, OUTPUT);
  stopMotors();

  if (!mpu.begin()) {
    while (1) delay(100); // MPU fail
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(50);

  // seed filter and calibrate gyro
  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
  pitchFiltered = calcPitch(accelEvent);
  calibrateGyroZ();
}

void loop() {
  // read sensors
  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
  float pitch = calcPitch(accelEvent);
  pitchFiltered = PITCH_ALPHA * pitch + (1.0 - PITCH_ALPHA) * pitchFiltered;

  // update maximum pitch
  if (pitchFiltered > maxPitch) maxPitch = pitchFiltered;

  // LCD displays ONLY the maximum ramp angle
  lcd.setCursor(0, 0);
  lcd.print(maxPitch, 1);  // 1 decimal
  lcd.print("   ");        // clear leftover digits

  // --- State machine logic ---
  static int consec = 0;
  static bool startedDescent = false;

  switch (state) {
    case DRIVE_TO_RAMP:
      moveForward();
      if (pitchFiltered > RAMP_DETECT_ANGLE) consec++;
      else consec = 0;
      if (consec >= REQ_CONSEC) {
        stopMotors();
        consec = 0;
        state = CLIMBING_TO_TOP;
      }
      break;

    case CLIMBING_TO_TOP:
      moveForward();
      if (fabs(pitchFiltered) < TOP_FLAT) consec++;
      else consec = 0;
      if (consec >= REQ_CONSEC) {
        stopMotors();
        consec = 0;
        state = AT_TOP_ROTATING;
      }
      break;

    case AT_TOP_ROTATING:
      rotate360_withGyro();  // rotation happens, LCD still shows maxPitch
      state = DESCENDING;
      break;

    case DESCENDING:
      moveForward();
      if (!startedDescent && pitchFiltered < -(RAMP_DETECT_ANGLE * 0.6)) startedDescent = true;
      if (startedDescent && fabs(pitchFiltered) < BOTTOM_FLAT) {
        stopMotors();
        state = FINISHED;
      }
      break;

    case FINISHED:
      stopMotors();
      break;
  }

  delay(50);  // frequent updates
}

/* ---------- Helpers ---------- */

float calcPitch(const sensors_event_t &a) {
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  return atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, 0); analogWrite(ENB_PIN, 0);
}

void moveForward() {
  digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);  digitalWrite(IN4_PIN, HIGH);
  analogWrite(ENA_PIN, 250); analogWrite(ENB_PIN, 250);
}

void calibrateGyroZ() {
  unsigned long start = millis();
  unsigned long end = start + GYRO_CAL_MS;
  double sum = 0.0; unsigned long cnt = 0;
  while (millis() < end) {
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    sum += gyroEvent.gyro.z * 57.295779513F;
    cnt++; delay(4);
  }
  gyroZBiasDegPerSec = (cnt>0) ? float(sum / (double)cnt) : 0.0f;
}

void rotate360_withGyro() {
  stopMotors(); delay(80);
  float integrated = 0.0f;
  unsigned long last = millis();
  unsigned long start = last;

  // in-place rotate
  digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, ROT_PWM); analogWrite(ENB_PIN, ROT_PWM);

  while (true) {
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    unsigned long now = millis();
    float dt = (now - last) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    last = now;

    float gz_corr = gyroEvent.gyro.z * 57.295779513F - gyroZBiasDegPerSec;
    integrated += gz_corr * dt;

    // LCD continues to show maxPitch
    lcd.setCursor(0, 0);
    lcd.print(maxPitch, 1);
    lcd.print("   ");

    if (fabs(integrated) >= (360.0f - ROT_TOLERANCE_DEG)) break;
    if (now - start > ROT_TIMEOUT_MS) break;
    delay(8);
  }

  stopMotors(); delay(300);
}
