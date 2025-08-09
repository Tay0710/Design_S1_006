from dataclasses import dataclass
from scipy.signal import detrend
import imufusion
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons, CheckButtons, TextBox
import numpy as np
import json, csv, os, time

# === Load sensor data ===
data = np.genfromtxt("../sensor_logs/2025-08-08 19-14-49.csv", delimiter=",", skip_header=1)
timestamp     = data[:, 0]
gyroscope_raw = data[:, 1:4]
accelerometer = data[:, 4:7]

# === Sample rate ===
fs = 1.0 / np.mean(np.diff(timestamp))
print("Sample Rate:", fs)
dt = np.diff(timestamp, prepend=timestamp[0])

# ---------- Core compute ----------
def run_once(params):
    gyro = np.copy(gyroscope_raw)
    accel = np.copy(accelerometer)

    offset = imufusion.Offset(int(fs))
    ahrs   = imufusion.Ahrs()
    ahrs.settings = imufusion.Settings(
        imufusion.CONVENTION_NWU,
        float(params["gain"]),
        int(params["gyro_range"]),
        int(params["accel_rej"]),
        int(params["mag_rej"]),
        int(params["rej_timeout"]),
    )

    earth_acc = np.empty((len(timestamp), 3))
    for i in range(len(timestamp)):
        gyro[i] = offset.update(gyro[i])
        ahrs.update_no_magnetometer(gyro[i], accel[i], dt[i])
        earth_acc[i] = ahrs.earth_acceleration

    acc_norm = np.linalg.norm(earth_acc, axis=1)
    is_moving = acc_norm > float(params["motion_threshold"])
    sm = int(params["smoothing_margin"])
    if sm > 0:
        for i in range(0, len(is_moving) - sm):
            if not is_moving[i] and np.any(is_moving[i:i+sm]):
                is_moving[i] = True
        for i in range(len(is_moving) - 1, sm, -1):
            if not is_moving[i] and np.any(is_moving[i-sm:i]):
                is_moving[i] = True

    vel = np.zeros((len(timestamp), 3))
    for i in range(len(timestamp)):
        if is_moving[i]:
            vel[i] = vel[i-1] + dt[i] * earth_acc[i]
        else:
            vel[i] = 0.0

    for k in range(3):
        vel[:, k] = detrend(vel[:, k], type='linear')

    pos = np.zeros((len(timestamp), 3))
    for i in range(len(timestamp)):
        pos[i] = pos[i-1] + dt[i] * vel[i]

    err_xy = float(np.linalg.norm(pos[-1, :2]))
    return pos, err_xy

# ---------- Figure & controls ----------
fig = plt.figure(figsize=(11, 9))
ax = fig.add_axes([0.08, 0.30, 0.72, 0.65])
ax.set_title("Interactive XY Trajectory Tuner")
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
ax.set_aspect("equal", adjustable="datalim")
ax.grid(True)

active_line, = ax.plot([], [], lw=1.6, label="current")
ax.legend(loc="upper left", fontsize=8)

# Sliders
ax_gain   = fig.add_axes([0.08, 0.22, 0.50, 0.03])
ax_accrej = fig.add_axes([0.08, 0.18, 0.50, 0.03])
ax_mt     = fig.add_axes([0.08, 0.14, 0.50, 0.03])
ax_sm     = fig.add_axes([0.08, 0.10, 0.50, 0.03])
ax_rt     = fig.add_axes([0.08, 0.06, 0.50, 0.03])

s_gain   = Slider(ax_gain,   "gain",              0.2, 10.0,  valinit=1.0, valstep=0.05)
s_accrej = Slider(ax_accrej, "accel_rej",         4,   20,   valinit=12,  valstep=1)
s_mt     = Slider(ax_mt,     "motion_threshold",  0.05,3.0,  valinit=0.50, valstep=0.01)
s_sm     = Slider(ax_sm,     "smooth (×fs)",      0.05,0.60, valinit=0.15, valstep=0.01)
s_rt     = Slider(ax_rt,     "rej_timeout (×fs)", 0.5, 5.0,  valinit=1.5,  valstep=0.1)

# Radios
ax_gyro = fig.add_axes([0.82, 0.58, 0.16, 0.28])
r_gyro = RadioButtons(ax_gyro, labels=["200","500","1000","1500","2000"], active=2)
ax_gyro.set_title("gyro_range (dps)")

ax_mag = fig.add_axes([0.82, 0.52, 0.16, 0.06])
r_mag = RadioButtons(ax_mag, labels=["0","5","10"], active=0)
ax_mag.set_title("mag_rej")

# Check
ax_chk = fig.add_axes([0.82, 0.46, 0.16, 0.06])
c_overlay = CheckButtons(ax_chk, ["Overlay runs"], [False])

# Presets
ax_p1 = fig.add_axes([0.82, 0.39, 0.16, 0.05])
ax_p2 = fig.add_axes([0.82, 0.33, 0.16, 0.05])
ax_p3 = fig.add_axes([0.82, 0.27, 0.16, 0.05])
b_snap   = Button(ax_p1, "Preset: SNAP")
b_bal    = Button(ax_p2, "Preset: BAL")
b_noisy  = Button(ax_p3, "Preset: NOISY")

# Save/Best/Export
ax_note = fig.add_axes([0.82, 0.22, 0.16, 0.04])
t_note  = TextBox(ax_note, "Note:", initial="")

ax_save = fig.add_axes([0.82, 0.16, 0.16, 0.05])
ax_best = fig.add_axes([0.82, 0.10, 0.16, 0.05])
ax_csv  = fig.add_axes([0.82, 0.04, 0.16, 0.05])
b_save  = Button(ax_save, "Save Current")
b_best  = Button(ax_best, "Mark Best")
b_csv   = Button(ax_csv,  "Export CSV")

# Run/Reset
ax_run = fig.add_axes([0.62, 0.24, 0.18, 0.04])
ax_rst = fig.add_axes([0.62, 0.20, 0.18, 0.04])
b_run  = Button(ax_run, "RUN", color="#e6f2ff")
b_rst  = Button(ax_rst, "Reset view")

# Status
status = fig.text(0.08, 0.26, "", fontsize=10, family="monospace")
status2 = fig.text(0.08, 0.02, "", fontsize=9, family="monospace")

saved_rows = []

def collect_params():
    return {
        "gain":          float(s_gain.val),
        "gyro_range":    int(r_gyro.value_selected),
        "accel_rej":     int(s_accrej.val),
        "mag_rej":       int(r_mag.value_selected),
        "rej_timeout":   int(s_rt.val * fs),
        "motion_threshold": float(s_mt.val),
        "smoothing_margin": int(s_sm.val * fs),
    }

def params_label(params, err):
    return (f"g={params['gain']:.2f}, gr={params['gyro_range']}, ar={params['accel_rej']}, "
            f"rt={int(params['rej_timeout'])}smp, mt={params['motion_threshold']:.2f}, "
            f"sm={int(params['smoothing_margin'])}smp | errXY={err:.2f} m")

def apply_preset(kind):
    if kind == "SNAP":
        s_gain.set_val(1.3); r_gyro.set_active(2)
        s_accrej.set_val(12); r_mag.set_active(0)
        s_rt.set_val(1.0); s_mt.set_val(0.50); s_sm.set_val(0.12)
    elif kind == "BAL":
        s_gain.set_val(1.0); r_gyro.set_active(2)
        s_accrej.set_val(12); r_mag.set_active(0)
        s_rt.set_val(1.5); s_mt.set_val(0.50); s_sm.set_val(0.14)
    elif kind == "NOISY":
        s_gain.set_val(1.2); r_gyro.set_active(3)
        s_accrej.set_val(16); r_mag.set_active(1)
        s_rt.set_val(2.0); s_mt.set_val(0.60); s_sm.set_val(0.20)

def do_run(event=None, highlight=False):
    params = collect_params()
    pos, err = run_once(params)
    label = params_label(params, err)

    if not c_overlay.get_status()[0]:
        for ln in list(ax.lines):
            ln.remove()

    (line,) = ax.plot(
        pos[:,0], pos[:,1],
        lw=2.2 if highlight else 1.4,
        alpha=1.0 if highlight else 0.95,
        label=label
    )

    ax.legend(loc="upper left", fontsize=8)
    status.set_text(label)
    ax.relim(); ax.autoscale_view()
    ax.set_aspect("equal", adjustable="datalim")
    fig.canvas.draw_idle()
    return params, err

def reset_view(event=None):
    ax.relim(); ax.autoscale_view()
    ax.set_aspect("equal", adjustable="datalim")
    fig.canvas.draw_idle()

def save_row(params, err, note="", mark_best=False):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    row = {
        "timestamp": ts,
        "gain": params["gain"],
        "gyro_range": params["gyro_range"],
        "accel_rej": params["accel_rej"],
        "mag_rej": params["mag_rej"],
        "rej_timeout_samples": int(params["rej_timeout"]),
        "motion_threshold": params["motion_threshold"],
        "smoothing_margin_samples": int(params["smoothing_margin"]),
        "err_xy": err,
        "note": note,
        "best": bool(mark_best),
    }
    saved_rows.append(row)
    return row

def on_save(event=None):
    params = collect_params()
    _, err = run_once(params)
    note = t_note.text.strip()
    save_row(params, err, note, mark_best=False)
    status2.set_text(f"Saved current: errXY={err:.2f}  (rows: {len(saved_rows)})")

def on_best(event=None):
    params, err = do_run(highlight=True)
    note = t_note.text.strip()
    row = save_row(params, err, note, mark_best=True)
    with open("best_params.json", "w", encoding="utf-8") as f:
        json.dump(row, f, indent=2)
    status2.set_text(f"Marked BEST → best_params.json (errXY={err:.2f})")

def on_export_csv(event=None):
    if not saved_rows:
        status2.set_text("Nothing to export yet.")
        return
    with open("tuning_log.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(saved_rows[0].keys()))
        writer.writeheader()
        for r in saved_rows:
            writer.writerow(r)
    status2.set_text(f"Exported {len(saved_rows)} rows → tuning_log.csv")

# Buttons wiring
b_run.on_clicked(do_run)
b_rst.on_clicked(reset_view)
b_save.on_clicked(on_save)
b_best.on_clicked(on_best)
b_csv.on_clicked(on_export_csv)
b_snap.on_clicked(lambda e: (apply_preset("SNAP"), do_run()))
b_bal.on_clicked(lambda e: (apply_preset("BAL"), do_run()))
b_noisy.on_clicked(lambda e: (apply_preset("NOISY"), do_run()))

# Init
apply_preset("SNAP")
do_run()

plt.tight_layout()
plt.show()
