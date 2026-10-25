// IMU hand test: classic ESP32 DevKit + Adafruit BNO085 over SPI.
// Wiring: VIN -> 3V3, GND -> GND, SCL -> GPIO18 (SCK), SDA -> GPIO19 (MISO),
// DI -> GPIO23 (MOSI), CS -> GPIO27, INT -> GPIO26, RST -> GPIO25.
// Tie P0 and P1 to 3V3 to select SPI mode. Power off before changing wiring.
// Same CSV output as imu_handtest_gy521, so tools/live_imu.py works with either.
// Lines starting with '#' are status messages; the logger ignores them.
//
// Columns:
//   t_ms                   ESP32 millis() at the time of the sample
//   roll, pitch, yaw       on-chip fused orientation in degrees (game rotation vector, no magnetometer)
//   roll_acc, pitch_acc    tilt from the accelerometer alone, in degrees
//   ax, ay, az             acceleration in m/s^2, gravity included
//   gx, gy, gz             angular rate in deg/s
//
// Send 'z' over serial to zero yaw at the current heading.
// Library: "Adafruit BNO08x" (Arduino Library Manager; install its dependencies when prompted).

#include <Adafruit_BNO08x.h>
#include <SPI.h>
#include <math.h>

const uint8_t BNO085_SCK = 18;
const uint8_t BNO085_MISO = 19;
const uint8_t BNO085_MOSI = 23;
const uint8_t BNO085_CS = 27;
const uint8_t BNO085_INT = 26;
const uint8_t BNO085_RESET = 25;
const uint32_t REPORT_INTERVAL_US = 10000;  // 100 Hz

// Serial Monitor mode: slower, readable "label:value" lines (also works in Tools > Serial Plotter).
// Set READABLE = false and PRINT_EVERY = 1 for tools/live_imu.py, which expects full-rate CSV.
const bool READABLE = true;
const int PRINT_EVERY = 50;           // print 1 of every N samples: 50 -> 2 lines/s
int sampleCount = 0;

Adafruit_BNO08x bno08x(BNO085_RESET);
sh2_SensorValue_t sensorValue;

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float yawOffset = 0, lastYaw = 0;

void enableReports() {
  bool ok = bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, REPORT_INTERVAL_US) &&
            bno08x.enableReport(SH2_ACCELEROMETER, REPORT_INTERVAL_US) &&
            bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_INTERVAL_US);
  Serial.println(ok ? "# reports enabled" : "# ERROR: could not enable reports");
}

void printSample(float roll, float pitch, float yaw, float rollAcc, float pitchAcc,
                 float ax, float ay, float az, float gx, float gy, float gz) {
  if (++sampleCount % PRINT_EVERY != 0) return;
  if (READABLE) {
    Serial.printf("roll:%.1f,pitch:%.1f,yaw:%.1f,ax:%.2f,ay:%.2f,az:%.2f,gx:%.1f,gy:%.1f,gz:%.1f\n",
                  roll, pitch, yaw, ax, ay, az, gx, gy, gz);
  } else {
    Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                  millis(), roll, pitch, yaw, rollAcc, pitchAcc,
                  ax, ay, az, gx, gy, gz);
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 3000) delay(10);  // native USB boards need a moment

  SPI.begin(BNO085_SCK, BNO085_MISO, BNO085_MOSI, BNO085_CS);
  while (!bno08x.begin_SPI(BNO085_CS, BNO085_INT, &SPI)) {
    Serial.println("# ERROR: BNO085 not found over SPI. Check wiring and P0/P1 = 3V3. Retrying...");
    delay(1000);
  }
  Serial.println("# BNO085 found over SPI");
  enableReports();
  if (!READABLE) Serial.println("t_ms,roll,pitch,yaw,roll_acc,pitch_acc,ax,ay,az,gx,gy,gz");
}

void loop() {
  if (Serial.available() && Serial.read() == 'z') {
    yawOffset += lastYaw;
    Serial.println("# yaw zeroed");
  }

  if (bno08x.wasReset()) {
    Serial.println("# sensor reset, re-enabling reports");
    enableReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) return;

  switch (sensorValue.sensorId) {
    case SH2_ACCELEROMETER:
      ax = sensorValue.un.accelerometer.x;
      ay = sensorValue.un.accelerometer.y;
      az = sensorValue.un.accelerometer.z;
      break;

    case SH2_GYROSCOPE_CALIBRATED:
      gx = sensorValue.un.gyroscope.x * RAD_TO_DEG;
      gy = sensorValue.un.gyroscope.y * RAD_TO_DEG;
      gz = sensorValue.un.gyroscope.z * RAD_TO_DEG;
      break;

    case SH2_GAME_ROTATION_VECTOR: {
      // The rotation vector drives the output rate; accel/gyro use their latest values
      float w = sensorValue.un.gameRotationVector.real;
      float x = sensorValue.un.gameRotationVector.i;
      float y = sensorValue.un.gameRotationVector.j;
      float z = sensorValue.un.gameRotationVector.k;

      float roll  = atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * RAD_TO_DEG;
      float sinp  = constrain(2 * (w * y - z * x), -1.0f, 1.0f);
      float pitch = asinf(sinp) * RAD_TO_DEG;
      float yaw   = atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * RAD_TO_DEG - yawOffset;
      if (yaw > 180) yaw -= 360;
      if (yaw < -180) yaw += 360;
      lastYaw = yaw;

      float rollAcc  = atan2f(ay, az) * RAD_TO_DEG;
      float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

      printSample(roll, pitch, yaw, rollAcc, pitchAcc, ax, ay, az, gx, gy, gz);
      break;
    }
  }
}
