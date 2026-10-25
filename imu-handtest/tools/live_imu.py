"""Live plot of the ESP32 IMU + FSR pressure hand test, with optional CSV logging.

Works with either sketch (GY-521 or BNO085). Any columns after the 12 IMU columns are
treated as pressure channels (p0_mv, p1_mv, ...), so the FSR count is picked up automatically.

    python imu-handtest/tools/live_imu.py                 # auto-detect the ESP32 port
    python imu-handtest/tools/live_imu.py --port /dev/cu.usbmodem101 --log run1.csv

Keys in the plot window: 0-9 writes that digit into the CSV's `label` column
(for example, the reference angle you're holding), space clears it, and z
recalibrates the gyro and zeroes the angles on the GY-521 (hold it still), or zeroes
yaw on the BNO085.
"""

import argparse
import collections
import csv
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation
from serial.tools import list_ports

COLUMNS = ["t_ms", "roll", "pitch", "yaw", "roll_acc", "pitch_acc",
           "ax", "ay", "az", "gx", "gy", "gz"]
MAX_FSR = 4
WINDOW_S = 10
MAX_POINTS = WINDOW_S * 100 * 2  # headroom above 100 Hz


def find_port():
    ports = list(list_ports.comports())
    for p in ports:
        desc = f"{p.device} {p.description} {p.manufacturer or ''}".lower()
        if any(k in desc for k in ("usbmodem", "usbserial", "cp210", "ch34", "esp32", "adafruit", "wch")):
            return p.device
    raise SystemExit("No ESP32 port found. Ports seen: " + ", ".join(p.device for p in ports) +
                     "\nPass --port explicitly.")


class Reader(threading.Thread):
    """Reads serial on a background thread so the plot never falls behind the stream."""

    def __init__(self, port, baud, log_path):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, baud, timeout=1)
        self.buf = collections.deque(maxlen=MAX_POINTS)
        self.lock = threading.Lock()
        self.label = ""
        self.count = 0
        self.log_file = open(log_path, "w", newline="") if log_path else None
        self.writer = csv.writer(self.log_file) if self.log_file else None
        self.n_fsr = None  # set from the first data line

    def run(self):
        while True:
            line = self.ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            if line.startswith("#") or line.startswith("t_ms"):
                print(line)
                continue
            parts = line.split(",")
            n_fsr = len(parts) - len(COLUMNS)
            if not 0 <= n_fsr <= MAX_FSR or (self.n_fsr is not None and n_fsr != self.n_fsr):
                continue
            try:
                row = [float(v) for v in parts]
            except ValueError:
                continue
            if self.n_fsr is None:
                self.n_fsr = n_fsr
                print(f"# {n_fsr} pressure channel(s) detected")
                if self.writer:
                    self.writer.writerow(["host_time"] + COLUMNS +
                                         [f"p{i}_mv" for i in range(n_fsr)] + ["label"])
            with self.lock:
                self.buf.append(row)
                self.count += 1
            if self.writer:
                self.writer.writerow([f"{time.time():.4f}"] + parts + [self.label])

    def snapshot(self):
        with self.lock:
            return np.array(self.buf) if self.buf else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--log", help="CSV file to save every sample to")
    args = ap.parse_args()

    port = args.port or find_port()
    print(f"Opening {port}")
    reader = Reader(port, args.baud, args.log)
    reader.start()

    fig, (ax_ang, ax_p, ax_gyro, ax_acc) = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    fig.suptitle(f"IMU live: {port}   (z = zero)")

    lines = {
        "roll": ax_ang.plot([], [], label="roll (filtered)", color="tab:blue")[0],
        "pitch": ax_ang.plot([], [], label="pitch (filtered)", color="tab:orange")[0],
        "roll_acc": ax_ang.plot([], [], "--", label="roll (accel only)", color="tab:blue", alpha=0.5)[0],
        "pitch_acc": ax_ang.plot([], [], "--", label="pitch (accel only)", color="tab:orange", alpha=0.5)[0],
        "gx": ax_gyro.plot([], [], label="gx")[0],
        "gy": ax_gyro.plot([], [], label="gy")[0],
        "gz": ax_gyro.plot([], [], label="gz")[0],
        "ax": ax_acc.plot([], [], label="ax")[0],
        "ay": ax_acc.plot([], [], label="ay")[0],
        "az": ax_acc.plot([], [], label="az")[0],
        "amag": ax_acc.plot([], [], label="|a|", color="black", lw=0.8)[0],
    }
    p_lines = [ax_p.plot([], [], label=f"p{i}", lw=1.5)[0] for i in range(MAX_FSR)]
    ax_ang.set_ylabel("deg")
    ax_ang.set_ylim(-95, 95)
    ax_p.set_ylabel("pressure (mV)")
    ax_p.set_ylim(-50, 3350)
    ax_gyro.set_ylabel("deg/s")
    ax_acc.set_ylabel("m/s²")
    ax_acc.axhline(9.81, color="gray", lw=0.8, ls=":")
    ax_acc.set_xlabel("seconds (ESP32 clock)")
    for a in (ax_ang, ax_p, ax_gyro, ax_acc):
        a.legend(loc="upper left", fontsize=8)
        a.grid(alpha=0.3)
    readout = ax_ang.text(0.99, 0.95, "", transform=ax_ang.transAxes, ha="right", va="top",
                          family="monospace", fontsize=10,
                          bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"))

    idx = {c: i for i, c in enumerate(COLUMNS)}
    rate_state = {"t": time.time(), "n": 0, "hz": 0.0}
    legend_state = {"n": None}

    def on_key(event):
        if event.key and event.key.isdigit():
            reader.label = event.key
        elif event.key == " ":
            reader.label = ""
        elif event.key == "z":
            reader.ser.write(b"z")
            return
        print(f"label = {reader.label!r}")

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        data = reader.snapshot()
        if data is None:
            return
        t = data[:, idx["t_ms"]] / 1000.0
        keep = t >= t[-1] - WINDOW_S
        data, t = data[keep], t[keep]

        for name in ("roll", "pitch", "roll_acc", "pitch_acc", "gx", "gy", "gz", "ax", "ay", "az"):
            lines[name].set_data(t, data[:, idx[name]])
        amag = np.linalg.norm(data[:, [idx["ax"], idx["ay"], idx["az"]]], axis=1)
        lines["amag"].set_data(t, amag)
        n_fsr = data.shape[1] - len(COLUMNS)
        if legend_state["n"] != n_fsr:
            ax_p.legend(handles=p_lines[:n_fsr], loc="upper left", fontsize=8)
            legend_state["n"] = n_fsr
        for i, pl in enumerate(p_lines):
            pl.set_visible(i < n_fsr)
            if i < n_fsr:
                pl.set_data(t, data[:, len(COLUMNS) + i])

        ax_ang.set_xlim(t[-1] - WINDOW_S, t[-1])
        for a in (ax_gyro, ax_acc):
            a.relim()
            a.autoscale_view(scalex=False)

        now = time.time()
        if now - rate_state["t"] >= 1.0:
            rate_state["hz"] = (reader.count - rate_state["n"]) / (now - rate_state["t"])
            rate_state["t"], rate_state["n"] = now, reader.count

        last = data[-1]
        pressures = "  ".join(f"p{i} {last[len(COLUMNS) + i]:5.0f} mV" for i in range(n_fsr))
        readout.set_text(
            f"roll {last[idx['roll']]:7.1f}°  pitch {last[idx['pitch']]:7.1f}°  yaw {last[idx['yaw']]:7.1f}°\n"
            f"|a| {amag[-1]:5.2f} m/s²   {rate_state['hz']:5.1f} Hz   label '{reader.label}'\n"
            f"{pressures}"
        )

    _anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    plt.tight_layout()
    plt.show()

    if reader.log_file:
        reader.log_file.close()
        print(f"Saved {args.log}")


if __name__ == "__main__":
    main()
