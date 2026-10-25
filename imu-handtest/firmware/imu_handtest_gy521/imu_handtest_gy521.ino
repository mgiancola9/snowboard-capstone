// IMU hand test: ESP32 + GY-521 (MPU-6050) over I2C.
// Streams one CSV line per sample at 100 Hz over USB serial.
// Lines starting with '#' are status messages; the logger ignores them.
//
// Wiring: GY-521 VCC -> 3V3, GND -> GND, SDA -> board SDA, SCL -> board SCL
//   (classic ESP32 DevKit: SDA = GPIO21, SCL = GPIO22). AD0 low/floating = address 0x68.
//
// Columns:
//   t_ms                   ESP32 millis() at the time of the sample
//   roll, pitch            complementary-filter tilt in degrees (gyro + accel)
//   yaw                    integrated gz in degrees; no magnetometer, so it drifts
//   roll_acc, pitch_acc    tilt from the accelerometer alone, in degrees
//   ax, ay, az             acceleration in m/s^2, gravity included
//   gx, gy, gz             angular rate in deg/s, startup bias removed
//   p0_mv, p1_mv, ...      FSR divider voltage in mV, one column per entry in FSR_PINS
//
// Keep the sensor still for the first ~2 s after boot (gyro bias calibration).
// Send 'z' over serial to recalibrate and zero the angles.
// No libraries needed: talks to the MPU-6050 registers directly through Wire.

#include <Wire.h>
#include <math.h>

const uint8_t MPU_ADDR = 0x68;
const uint32_t SAMPLE_US = 10000;   // 100 Hz
const float ALPHA = 0.98f;          // complementary filter: weight on gyro integration
const float ACCEL_LSB_PER_G = 4096.0f;   // +/-8 g
const float GYRO_LSB_PER_DPS = 65.5f;    // +/-500 deg/s
const float G = 9.80665f;

// Serial Monitor mode: slower, readable "label:value" lines (also works in Tools > Serial Plotter).
// Set READABLE = false and PRINT_EVERY = 1 for tools/live_imu.py, which expects full-rate CSV.
const bool READABLE = true;
const int PRINT_EVERY = 50;           // print 1 of every N samples: 50 -> 2 lines/s
int sampleCount = 0;

// FSR402 pressure sensors: 3V3 -> FSR -> pin, and pin -> 10k -> GND. More force = higher mV.
// ADC1 pins only (32-39), since ADC2 pins stop working once Bluetooth or Wi-Fi is on.
const int FSR_PINS[] = {34};          // one sensor; use {34, 35} for two
const int NUM_FSR = sizeof(FSR_PINS) / sizeof(FSR_PINS[0]);
const int FSR_OVERSAMPLE = 8;         // average several readings to cut ADC noise

float gBiasX = 0, gBiasY = 0, gBiasZ = 0;
float roll = 0, pitch = 0, yaw = 0;
uint32_t lastSampleUs = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// Reads accel (m/s^2) and raw gyro (deg/s, bias not removed). Returns false on an I2C error.
bool readImu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H; next 14 bytes are accel, temp, gyro
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;

  int16_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = (int16_t)((Wire.read() << 8) | Wire.read());

  ax = raw[0] / ACCEL_LSB_PER_G * G;
  ay = raw[1] / ACCEL_LSB_PER_G * G;
  az = raw[2] / ACCEL_LSB_PER_G * G;
  gx = raw[4] / GYRO_LSB_PER_DPS;
  gy = raw[5] / GYRO_LSB_PER_DPS;
  gz = raw[6] / GYRO_LSB_PER_DPS;
  return true;
}

void accelTilt(float ax, float ay, float az, float &rollAcc, float &pitchAcc) {
  rollAcc = atan2f(ay, az) * RAD_TO_DEG;
  pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

void calibrate() {
  Serial.println("# calibrating gyro, keep the sensor still...");
  const int N = 200;
  float sx = 0, sy = 0, sz = 0, ax, ay, az, gx, gy, gz;
  int got = 0;
  for (int i = 0; i < N; i++) {
    if (readImu(ax, ay, az, gx, gy, gz)) {
      sx += gx; sy += gy; sz += gz;
      got++;
    }
    delay(5);
  }
  if (got > 0) {
    gBiasX = sx / got; gBiasY = sy / got; gBiasZ = sz / got;
  }
  // Start the filter at the accelerometer's tilt so it doesn't have to converge from zero
  readImu(ax, ay, az, gx, gy, gz);
  accelTilt(ax, ay, az, roll, pitch);
  yaw = 0;
  Serial.printf("# gyro bias (deg/s): %.2f, %.2f, %.2f\n", gBiasX, gBiasY, gBiasZ);
}

int readFsrMv(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < FSR_OVERSAMPLE; i++) sum += analogReadMilliVolts(pin);
  return sum / FSR_OVERSAMPLE;
}

void printSample(float roll, float pitch, float yaw, float rollAcc, float pitchAcc,
                 float ax, float ay, float az, float gx, float gy, float gz) {
  if (++sampleCount % PRINT_EVERY != 0) return;
  if (READABLE) {
    Serial.printf("roll:%.1f,pitch:%.1f,yaw:%.1f,ax:%.2f,ay:%.2f,az:%.2f,gx:%.1f,gy:%.1f,gz:%.1f",
                  roll, pitch, yaw, ax, ay, az, gx, gy, gz);
    for (int i = 0; i < NUM_FSR; i++) Serial.printf(",p%d:%d", i, readFsrMv(FSR_PINS[i]));
  } else {
    Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f",
                  millis(), roll, pitch, yaw, rollAcc, pitchAcc,
                  ax, ay, az, gx, gy, gz);
    for (int i = 0; i < NUM_FSR; i++) Serial.printf(",%d", readFsrMv(FSR_PINS[i]));
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 3000) delay(10);  // native USB boards need a moment

  Wire.begin();           // board default SDA/SCL
  Wire.setClock(400000);
  Serial.printf("# I2C on SDA=%d SCL=%d\n", SDA, SCL);

  uint8_t who;
  while ((who = readReg(0x75)) == 0xFF || who == 0x00) {
    Serial.println("# ERROR: no MPU-6050 at 0x68. Check VCC/GND/SDA/SCL wiring. Retrying...");
    delay(1000);
  }
  // Genuine parts answer 0x68; common clones answer 0x70, 0x72 or 0x98 and still work
  Serial.printf("# MPU found, WHO_AM_I = 0x%02X\n", who);

  writeReg(0x6B, 0x01);  // PWR_MGMT_1: wake up, clock from gyro X PLL
  delay(100);
  writeReg(0x1A, 0x03);  // CONFIG: digital low-pass ~44 Hz, keeps vibration out
  writeReg(0x19, 0x09);  // SMPLRT_DIV: 1 kHz / (1 + 9) = 100 Hz
  writeReg(0x1B, 0x08);  // GYRO_CONFIG: +/-500 deg/s
  writeReg(0x1C, 0x10);  // ACCEL_CONFIG: +/-8 g

  calibrate();
  if (!READABLE) {
    Serial.print("t_ms,roll,pitch,yaw,roll_acc,pitch_acc,ax,ay,az,gx,gy,gz");
    for (int i = 0; i < NUM_FSR; i++) Serial.printf(",p%d_mv", i);
    Serial.println();
  }
  lastSampleUs = micros();
}

void loop() {
  if (Serial.available() && Serial.read() == 'z') {
    calibrate();
    lastSampleUs = micros();
  }

  uint32_t now = micros();
  if (now - lastSampleUs < SAMPLE_US) return;
  float dt = (now - lastSampleUs) * 1e-6f;
  lastSampleUs = now;

  float ax, ay, az, gx, gy, gz;
  if (!readImu(ax, ay, az, gx, gy, gz)) {
    Serial.println("# I2C read failed");
    return;
  }
  gx -= gBiasX; gy -= gBiasY; gz -= gBiasZ;

  float rollAcc, pitchAcc;
  accelTilt(ax, ay, az, rollAcc, pitchAcc);

  // Gyro tracks fast motion, accel slowly pulls out the drift
  roll = ALPHA * (roll + gx * dt) + (1 - ALPHA) * rollAcc;
  pitch = ALPHA * (pitch + gy * dt) + (1 - ALPHA) * pitchAcc;
  yaw += gz * dt;

  printSample(roll, pitch, yaw, rollAcc, pitchAcc, ax, ay, az, gx, gy, gz);
}
