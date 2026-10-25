/* ESP32 Robotic Arm with MPU6050
   - Smooth synchronized servo motion
   - Computes Roll/Pitch from accelerometer
   - Maps orientation to servo targets
   - Forward kinematics -> End-effector (X,Y,Z)
   - Prints coordinates, movement vector, direction unit vector, speed and state

   Adjust LINK LENGTHS and sensitivity to match your hardware.
*/

#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>

// ---- Servos ----
Servo Spin, Shoulder, Elbow, Claw;

// Default MPU6050 I2C address
const uint8_t MPU_addr = 0x68;

// Raw MPU data
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// Current and target angles (degrees)
float spinA = 90, shoulderA = 120, elbowA = 90, clawA = 90;
float targetSpin = 90, targetShoulder = 120, targetElbow = 90, targetClaw = 90;

// Pins (change if needed)
const int SPIN_PIN = 13;
const int SHOULDER_PIN = 12;
const int ELBOW_PIN = 14;
const int CLAW_PIN = 27;

// Deadzone & sensitivity
const int ACC_DEADZONE = 1200;    // ignore small accel noise
const float ROLL_SENS = 60.0;     // degrees spin range mapped from roll
const float PITCH_SENS = 50.0;    // degrees shoulder/elbow range mapped from pitch

// smoothing factors (0..1) larger => faster movement
const float SMOOTH_SPIN = 0.12;
const float SMOOTH_SHOULDER = 0.10;
const float SMOOTH_ELBOW = 0.10;
const float SMOOTH_CLAW = 0.25;

// Link lengths for FK (in cm) — change to match real arm
const float L1 = 10.0;  // upper arm length
const float L2 = 12.0;  // forearm length

// Limits for servos (degrees)
const float SPIN_MIN = 10, SPIN_MAX = 170;
const float SHOULDER_MIN = 60, SHOULDER_MAX = 170;
const float ELBOW_MIN = 30, ELBOW_MAX = 150;
const float CLAW_MIN = 40, CLAW_MAX = 160;

// Serial output rate
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 200; // ms
const unsigned long CLAW_COMMAND_INTERVAL = 80; // ms

// Previous end-effector position (for delta/speed)
float prevX = NAN, prevY = NAN, prevZ = NAN;
unsigned long lastPositionTime = 0;
unsigned long lastClawCommand = 0;
unsigned long lastSensorError = 0;

String currentState = "Idle";

bool initializeMPU();
bool readMPU();
float smoothMove(float current, float target, float alpha);
String detectStateFromTargets();

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL default ESP32 pins

  // Init MPU6050
  if (!initializeMPU()) {
    Serial.println("ERROR: MPU6050 was not detected. Check wiring and address 0x68.");
  }

  // Attach servos
  Spin.attach(SPIN_PIN);
  Shoulder.attach(SHOULDER_PIN);
  Elbow.attach(ELBOW_PIN);
  Claw.attach(CLAW_PIN);

  // Write initial positions
  Spin.write(spinA);
  Shoulder.write(shoulderA);
  Elbow.write(elbowA);
  Claw.write(clawA);

  Serial.println("Robotic arm ready. Sending coordinates and state...");
}

void loop() {
  // Read accelerometer + gyro
  if (!readMPU()) {
    if (millis() - lastSensorError >= 1000) {
      Serial.println("ERROR: MPU6050 read failed; servos are holding their last targets.");
      lastSensorError = millis();
    }
    delay(10);
    return;
  }

  // Compute roll & pitch (radians)
  // roll: rotation around X axis, pitch: rotation around Y axis
  float ax = (float)AcX;
  float ay = (float)AcY;
  float az = (float)AcZ;

  // Basic accelerometer-based roll/pitch
  float roll  = atan2(ay, az); // radians
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)); // radians

  // Convert to degrees
  float rollDeg  = roll * 180.0 / M_PI;
  float pitchDeg = pitch * 180.0 / M_PI;

  // Apply deadzone to raw accel to avoid jitter
  if (abs(AcX) < ACC_DEADZONE && abs(AcY) < ACC_DEADZONE && abs(AcZ - 16384) < ACC_DEADZONE) {
    // near rest; avoid changing targets
  } else {
    // Map roll to base (spin)
    // roll ranges roughly -90..+90. Map to spin range.
    float mappedSpin = 90 + (rollDeg / 90.0) * (ROLL_SENS / 2.0);
    // Constrain to allowable range
    mappedSpin = constrain(mappedSpin, SPIN_MIN, SPIN_MAX);
    targetSpin = mappedSpin;

    // Map pitch to shoulder and elbow split
    // pitch ranges roughly -90..+90. We'll map positive pitch (tilt up) to lift shoulder,
    // negative pitch to lower. We'll also use part of pitch to extend/retract elbow.
    float pitchNormalized = constrain(pitchDeg / 90.0, -1.0, 1.0); // -1..1
    // shoulder target: center +/- portion of PITCH_SENS
    float mappedShoulder = 120 + pitchNormalized * (PITCH_SENS * 0.6); // shoulder more influential
    mappedShoulder = constrain(mappedShoulder, SHOULDER_MIN, SHOULDER_MAX);
    targetShoulder = mappedShoulder;

    // elbow target: invert or complement to shoulder so extension happens
    float mappedElbow = 90 - pitchNormalized * (PITCH_SENS * 0.4); // elbow moves opposite a bit
    mappedElbow = constrain(mappedElbow, ELBOW_MIN, ELBOW_MAX);
    targetElbow = mappedElbow;

    // Claw control from GyZ threshold (rotation)
    if (GyZ < -20000 && millis() - lastClawCommand >= CLAW_COMMAND_INTERVAL) {
      targetClaw = constrain(targetClaw + 1.0, CLAW_MIN, CLAW_MAX);
      currentState = "Claw Closing";
      lastClawCommand = millis();
    } else if (GyZ > 20000 && millis() - lastClawCommand >= CLAW_COMMAND_INTERVAL) {
      targetClaw = constrain(targetClaw - 1.0, CLAW_MIN, CLAW_MAX);
      currentState = "Claw Opening";
      lastClawCommand = millis();
    }
  }

  // Determine simple human-readable movement states (based on target differences)
  String state = detectStateFromTargets();

  // Apply smoothing (interpolate toward targets)
  spinA = smoothMove(spinA, targetSpin, SMOOTH_SPIN);
  shoulderA = smoothMove(shoulderA, targetShoulder, SMOOTH_SHOULDER);
  elbowA = smoothMove(elbowA, targetElbow, SMOOTH_ELBOW);
  clawA = smoothMove(clawA, targetClaw, SMOOTH_CLAW);

  // Write to servos
  Spin.write((int)spinA);
  Shoulder.write((int)shoulderA);
  Elbow.write((int)elbowA);
  Claw.write((int)clawA);

  // Compute forward kinematics -> end effector position (cm)
  // Use the following frame:
  // - Base at origin (0,0,0).
  // - Base rotation spinA rotates around Z (yaw).
  // - Shoulder angle measured from horizontal plane downward/upward? We'll assume shoulderA=90 is horizontal.
  // We'll convert servo angles to radians for FK. You may need to calibrate offsets.
  float baseRad = (spinA - 90.0f) * M_PI / 180.0f;      // yaw (0 when at 90deg)
  float shoulderRad = (shoulderA - 90.0f) * M_PI / 180.0f; // shoulder: 0 = horizontal
  float elbowRad = (elbowA - 90.0f) * M_PI / 180.0f;    // elbow: 0 = extended straight

  // Simple planar reach in shoulder-elbow plane:
  // Effective reach along that plane:
  float planarX = L1 * cos(shoulderRad) + L2 * cos(shoulderRad + elbowRad);
  float planarZ = L1 * sin(shoulderRad) + L2 * sin(shoulderRad + elbowRad);

  // Now rotate planar X around base yaw to produce X,Y
  float effX = planarX * cos(baseRad); // forward/back relative to base front
  float effY = planarX * sin(baseRad); // lateral offset (left/right)
  float effZ = planarZ;                // height in same units as L1, L2

  // Calculate movement delta and direction
  float dx = 0, dy = 0, dz = 0;
  float speed = 0;
  unsigned long now = millis();
  if (!isnan(prevX)) {
    dx = effX - prevX;
    dy = effY - prevY;
    dz = effZ - prevZ;
    float elapsedSeconds = (now - lastPositionTime) / 1000.0f;
    if (elapsedSeconds > 0.0f) {
      speed = sqrt(dx*dx + dy*dy + dz*dz) / elapsedSeconds;
    }
  }
  // Direction unit vector
  float mag = sqrt(dx*dx + dy*dy + dz*dz);
  float ux = 0, uy = 0, uz = 0;
  if (mag > 1e-6) {
    ux = dx / mag; uy = dy / mag; uz = dz / mag;
  }

  // Periodic printout: coordinates, movement vector, direction, speed, state
  if (millis() - lastPrint >= PRINT_INTERVAL) {
    Serial.print("State: "); Serial.print(state);
    Serial.print(" | Roll: "); Serial.print(rollDeg,1);
    Serial.print(" Pitch: "); Serial.print(pitchDeg,1);
    Serial.print(" | Servo targets(S,Sh,El,Cl): ");
    Serial.print((int)targetSpin); Serial.print(","); Serial.print((int)targetShoulder);
    Serial.print(","); Serial.print((int)targetElbow); Serial.print(","); Serial.print((int)targetClaw);
    Serial.print(" | Pos(cm) X:"); Serial.print(effX,2);
    Serial.print(" Y:"); Serial.print(effY,2);
    Serial.print(" Z:"); Serial.print(effZ,2);
    Serial.print(" | d(cm) dx:"); Serial.print(dx,2);
    Serial.print(" dy:"); Serial.print(dy,2);
    Serial.print(" dz:"); Serial.print(dz,2);
    Serial.print(" | dir: ["); Serial.print(ux,2); Serial.print(","); Serial.print(uy,2); Serial.print(","); Serial.print(uz,2);
    Serial.print("] | speed(cm/s): "); Serial.println(speed,2);

    lastPrint = now;
    // update prev positions
    prevX = effX; prevY = effY; prevZ = effZ;
    lastPositionTime = now;
  }

  // small loop delay
  delay(10);
}

bool initializeMPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0);     // wake up the sensor
  if (Wire.endTransmission(true) != 0) {
    return false;
  }

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x75);  // WHO_AM_I
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(MPU_addr, static_cast<size_t>(1), true) != 1) {
    return false;
  }

  uint8_t deviceId = Wire.read();
  return deviceId == 0x68 || deviceId == 0x69;
}

bool readMPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU_addr, static_cast<size_t>(14), true);
  if (Wire.available() < 14) return false;
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
  return true;
}

// Smooth interpolation helper
float smoothMove(float current, float target, float alpha) {
  // alpha in (0..1) proportion to move toward target each call
  return current + (target - current) * alpha;
}

// Decide string state from how different targets are from current positions
String detectStateFromTargets() {
  float dSpin = abs(targetSpin - spinA);
  float dSh   = abs(targetShoulder - shoulderA);
  float dEl   = abs(targetElbow - elbowA);
  float dCl   = abs(targetClaw - clawA);
  float maxd = max(max(dSpin, dSh), max(dEl, dCl));
  if (maxd < 0.8) return "Idle";
  String s = "";
  if (dSpin > 1.5) {
    if (targetSpin > spinA) s += "Rotating Right ";
    else s += "Rotating Left ";
  }
  if (dSh > 1.5) {
    if (targetShoulder > shoulderA) s += "Arm Up ";
    else s += "Arm Down ";
  }
  if (dEl > 1.5) {
    if (targetElbow > elbowA) s += "Elbow Extend ";
    else s += "Elbow Retract ";
  }
  if (dCl > 0.8) {
    if (targetClaw > clawA) s += "Claw Closing ";
    else s += "Claw Opening ";
  }
  s.trim();
  if (s.length() == 0) s = "Moving";
  return s;
}
