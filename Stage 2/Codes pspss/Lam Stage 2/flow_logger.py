"""Logs the flow skid's serial output to CSV and plots the trends.

Usage:
    pip install pyserial matplotlib
    python flow_logger.py COM5            # Windows (check Device Manager)
    python flow_logger.py /dev/ttyUSB0    # Linux / macOS
Ctrl+C stops logging, saves flow_log_<time>.csv and shows the plots
(flow in L/min and mL/s vs time, totalised litres, pump duties).
Anything typed in the PlatformIO monitor can't be sent while this runs;
close the monitor first, the port can only be opened once.
"""

import csv
import sys
import time

import matplotlib.pyplot as plt
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
fields = [
    "t_ms",
    "state",
    "pulses",
    "f_Hz",
    "Q_Lmin",
    "Q_mLs",
    "total_L",
    "float_high",
    "p1",
    "p2",
    "p3",
    "sov",
]
rows = []
name = time.strftime("flow_log_%Y%m%d_%H%M%S.csv")

with serial.Serial(port, 115200, timeout=1) as ser, open(name, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(fields)
    print(
        f"Logging {port} to {name}. Press the ON/OFF button to start; Ctrl+C to finish."
    )
    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if line.startswith("D,"):
                parts = line[2:].split(",")
                if len(parts) == len(fields):
                    w.writerow(parts)
                    rows.append(parts)
            elif line:
                print(line)
    except KeyboardInterrupt:
        pass

if not rows:
    sys.exit("No data received.")

t0 = float(rows[0][0])
t = [(float(r[0]) - t0) / 1000 for r in rows]
col = lambda k: [float(r[fields.index(k)]) for r in rows]

fig, ax = plt.subplots(4, 1, sharex=True, figsize=(11, 10))
ax[0].plot(t, col("Q_Lmin"))
ax[0].set_ylabel("Flow (L/min)")
ax[0].axhline(3.0, ls="--", c="grey", lw=0.8)
ax[1].plot(t, col("Q_mLs"), c="tab:orange")
ax[1].set_ylabel("Flow (mL/s)")
ax[2].plot(t, col("total_L"), c="tab:green")
ax[2].set_ylabel("Transferred (L)")
for k in ("p1", "p2", "p3"):
    ax[3].plot(t, col(k), label=k.upper())
ax[3].plot(t, [100 * v for v in col("float_high")], "k:", label="Sump high")
ax[3].set_ylabel("Duty (%)")
ax[3].legend(loc="upper right")
ax[3].set_xlabel("Time (s)")
for a in ax:
    a.grid(alpha=0.3)
fig.suptitle(f"FT-01 flow vs time  (final total {rows[-1][6]} L)")
fig.tight_layout()
fig.savefig(name.replace(".csv", ".png"), dpi=150)
plt.show()
