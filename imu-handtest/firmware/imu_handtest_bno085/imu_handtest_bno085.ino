// IMU hand test: ESP32 + Adafruit BNO085 over I2C (STEMMA QT).
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
#include <math.h>

const uint32_t REPORT_INTERVAL_US = 10000;  // 100 Hz

Adafruit_BNO08x bno08x(-1);  // no reset pin wired
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

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 3000) delay(10);  // native USB boards need a moment

#if defined(PIN_I2C_POWER)
  // Some Adafruit Feathers gate STEMMA QT power behind this pin
  pinMode(PIN_I2C_POWER, OUTPUT);
  digitalWrite(PIN_I2C_POWER, HIGH);
  delay(10);
#endif

  while (!bno08x.begin_I2C()) {
    Serial.println("# ERROR: BNO085 not found at 0x4A. Check the STEMMA QT cable. Retrying...");
    delay(1000);
  }
  Serial.println("# BNO085 found");
  enableReports();
  Serial.println("t_ms,roll,pitch,yaw,roll_acc,pitch_acc,ax,ay,az,gx,gy,gz");
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

      Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                    millis(), roll, pitch, yaw, rollAcc, pitchAcc,
                    ax, ay, az, gx, gy, gz);
      break;
    }
  }
}
