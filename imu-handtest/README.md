# IMU hand test

Proof of concept: ESP32 + IMU streaming orientation over USB, plotted live on a laptop.

| Sketch | IMU | Notes |
|---|---|---|
| `firmware/imu_handtest_gy521` | GY-521 (MPU-6050) | No libraries. Complementary filter on the ESP32. Yaw drifts (no magnetometer). |
| `firmware/imu_handtest_bno085` | Adafruit BNO085 | Needs the "Adafruit BNO08x" library. Orientation fused on the chip. |

Both print the same CSV columns at 100 Hz:
`t_ms,roll,pitch,yaw,roll_acc,pitch_acc,ax,ay,az,gx,gy,gz` (degrees, m/s², deg/s).

## GY-521 wiring (ESP32 DevKit)

| GY-521 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO21 |
| SCL | GPIO22 |

Keep the sensor still for ~2 s after boot while it calibrates the gyro.

## Live plot

```
python3 -m venv .venv
.venv/bin/pip install -r imu-handtest/requirements.txt
.venv/bin/python imu-handtest/tools/live_imu.py --log handtest.csv
```

Close the Arduino Serial Monitor first, since only one program can hold the port.

Keys in the plot window: `0`-`9` tag samples in the CSV `label` column, space clears the tag,
`z` recalibrates (GY-521, hold it still) or zeroes yaw (BNO085).
