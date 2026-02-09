import csv
import time
import serial
import matplotlib.pyplot as plt
from collections import deque

serial_port = "COM3" 
baud_rate = 115200 # 74880
csv_path = "motor_positions.csv"

# --- Serial ---
ser = serial.Serial(serial_port, baud_rate, timeout=0.01)

# --- CSV ---
csv_file = open(csv_path, "w", newline="")
writer = csv.writer(csv_file)
writer.writerow(["timestamp", "target", "current", "pwm"])  # he+ader

# --- Plot ---
plt.ion()  # interactive mode
fig, ax = plt.subplots()
ax.set_title("Motor Positions")
ax.set_xlabel("Samples")
ax.set_ylabel("Position")

max_points = 500  # keep plot responsive; adjust as needed
# target_data = []
# current_data = []
target_data = deque(maxlen=max_points)
current_data = deque(maxlen=max_points)

pwm_data = []

line_target, = ax.plot([], [], label="target")
line_current, = ax.plot([], [], label="current")
ax.legend()

ax.set_ylim(-270,270)
PLOT_HZ = 20
plot_period = 1.0/ PLOT_HZ
next_plot = time.time()


try:
    while True:
        # --- drain serial quickly ---
        # Read *all* available lines this loop iteration.
        while ser.in_waiting:
            raw = ser.readline()
            if not raw:
                break
            text = raw.decode("utf-8", errors="ignore").strip()
            if not text:
                continue

            parts = text.split()
            if len(parts) != 3:
                continue

            try:
                target = int(parts[0])
                current = int(parts[1])
                pwm = int(parts[2])
            except ValueError:
                continue

            ts = time.time()
            writer.writerow([ts, target, current, pwm])  # log every sample

            target_data.append(target)
            current_data.append(current)

        csv_file.flush()

        # --- plot at fixed rate ---
        now = time.time()
        if now >= next_plot and len(target_data) > 1:
            x = range(len(target_data))
            line_target.set_data(x, list(target_data))
            line_current.set_data(x, list(current_data))

            ax.set_xlim(0, max_points)  # fixed x-limits helps
            # Avoid autoscale unless needed; if you must, do it rarely:
            # ax.relim(); ax.autoscale_view()

            fig.canvas.draw_idle()
            plt.pause(0.001)  # lets GUI update
            next_plot = now + plot_period

except KeyboardInterrupt:
    print("Stopping...")
finally:
    ser.close()
    csv_file.close()