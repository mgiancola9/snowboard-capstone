# IMU hand test

Proof of concept: ESP32 + IMU streaming orientation over USB, plotted live on a laptop.

| Sketch | IMU | Notes |
|---|---|---|
| `firmware/imu_handtest_gy521` | GY-521 (MPU-6050) | No libraries. Complementary filter on the ESP32. Yaw drifts (no magnetometer). |
| `firmware/imu_handtest_bno085` | Adafruit BNO085 over SPI | Needs the "Adafruit BNO08x" library. Orientation fused on the chip. |

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

## BNO085 SPI wiring (classic ESP32 DevKit)

Disconnect USB and any other power before rewiring. Replace the old I2C connections
with the following connections; the STEMMA QT cable alone cannot carry SPI.

| Adafruit BNO085 | ESP32 |
|---|---|
| VIN | 3V3 |
| GND | GND |
| SCL (SPI clock) | GPIO18 (SCK) |
| SDA (SPI data out) | GPIO19 (MISO) |
| DI (SPI data in) | GPIO23 (MOSI) |
| CS | GPIO27 |
| INT | GPIO26 |
| RST | GPIO25 |
| P0 | 3V3 |
| P1 | 3V3 |

P0 and P1 must both be high when the sensor resets to select SPI mode. Connect both
INT and RST; the sketch uses GPIO25 to reset the sensor during initialization.
Leave other breakout pins disconnected. These GPIOs match the constants at the top
of `firmware/imu_handtest_bno085/imu_handtest_bno085.ino`.

SPI avoids the BNO085/ESP32 I2C compatibility issue documented in
[Adafruit's wiring guide](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/arduino#spi-wiring).
Install "Adafruit BNO08x" and its dependencies before compiling. Open the serial
monitor at 115200 baud; successful initialization prints `# BNO085 found over SPI`.

Current test status: compiled with ESP32 core 3.3.12 and Adafruit BNO08x 1.2.7,
and uploaded with flash verification. Hardware testing has not yet produced SPI
sensor readings: the ESP32 boots, but `# BNO085 found over SPI` does not appear.
Sensor initialization and wiring still need investigation.

## Output modes

At the top of each sketch:

- `READABLE = true`, `PRINT_EVERY = 50` (default): 2 lines/s of `roll:..,pitch:..,ax:..` for eyeballing in
  Serial Monitor at 115200, or as live graphs in Tools > Serial Plotter.
- `READABLE = false`, `PRINT_EVERY = 1`: full 100 Hz CSV for `tools/live_imu.py`.

The filter always runs at 100 Hz; `PRINT_EVERY` only thins what gets printed.

## Live plot

Flash with `READABLE = false` and `PRINT_EVERY = 1` first.

```
python3 -m venv .venv
.venv/bin/pip install -r imu-handtest/requirements.txt
.venv/bin/python imu-handtest/tools/live_imu.py --log handtest.csv
```

Close the Arduino Serial Monitor first, since only one program can hold the port.

Keys in the plot window: `0`-`9` tag samples in the CSV `label` column, space clears the tag,
`z` recalibrates (GY-521, hold it still) or zeroes yaw (BNO085).
