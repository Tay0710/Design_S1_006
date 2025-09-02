from dataclasses import dataclass
from scipy.signal import detrend
import imufusion
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons, CheckButtons, TextBox
import numpy as np
import json, csv, os, time, itertools

# === Load sensor data ===
data = np.genfromtxt("../optical_flow_method_data/ICM456XX_Walk.csv", delimiter=",", skip_header=1)
timestamp     = data[:, 0]
gyroscope_raw = data[:, 1:4]
accelerometer = data[:, 4:7]

# === Sample rate ===
fs = 1.0 / np.mean(np.diff(timestamp))
print("Sample Rate:", fs)
dt = np.diff(timestamp, prepend=timestamp[0])

# =========================================================
# Grid definition (tweak these to control the sweep size)
# =========================================================
GRID_GAIN        = np.round(np.arange(0.4, 10.0, 0.1), 2)     # e.g. 0.6 .. 1.5
GRID_GYRO_RANGE  = [2000]                     # dps
GRID_ACCEL_REJ   = list(range(10, 16, 2))                    # 10,12,14,16,18
GRID_MAG_REJ     = [0]                                       # keep 0 unless you have mags
GRID_RT_FACT     = [3.0]                       # ×fs → samples
GRID_MT          = np.round(np.arange(0.05, 0.2, 0.05), 2)  # 0.45..0.70
GRID_SM_FACT     = [0.3]                  # ×fs → samples

MAX_COMBOS       = 5000  # safety cap so you don't accidentally lock your machine
DEFAULT_SPEED_MS = 50   # default play speed (ms between steps)

# =========================================================
# Core compute
# =========================================================
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

# =========================================================
# UI setup
# =========================================================
fig = plt.figure(figsize=(12, 9))
ax = fig.add_axes([0.08, 0.30, 0.72, 0.65])
ax.set_title("Live Grid Iterator: watch params change, click Save/Best when you like one")
ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
ax.set_aspect("equal", adjustable="datalim")
ax.grid(True)

active_line, = ax.plot([], [], lw=1.6, label="current")
ax.legend(loc="upper left", fontsize=8)

# Sliders (bottom)
ax_gain   = fig.add_axes([0.08, 0.22, 0.50, 0.03])
ax_accrej = fig.add_axes([0.08, 0.18, 0.50, 0.03])
ax_mt     = fig.add_axes([0.08, 0.14, 0.50, 0.03])
ax_sm     = fig.add_axes([0.08, 0.10, 0.50, 0.03])
ax_rt     = fig.add_axes([0.08, 0.06, 0.50, 0.03])

s_gain   = Slider(ax_gain,   "gain",              0.2, 10.0, valinit=1.0,  valstep=0.05)
s_accrej = Slider(ax_accrej, "accel_rej",         4,   20,   valinit=12,   valstep=1)
s_mt     = Slider(ax_mt,     "motion_threshold",  0.05,3.0,  valinit=0.50, valstep=0.01)
s_sm     = Slider(ax_sm,     "smooth (×fs)",      0.05,0.60, valinit=0.15, valstep=0.01)
s_rt     = Slider(ax_rt,     "rej_timeout (×fs)", 0.5, 5.0,  valinit=1.5,  valstep=0.1)

# Radios
ax_gyro = fig.add_axes([0.82, 0.60, 0.16, 0.26])
r_gyro = RadioButtons(ax_gyro, labels=["200","500","1000","1500","2000"], active=2)
ax_gyro.set_title("gyro_range (dps)")

ax_mag = fig.add_axes([0.82, 0.54, 0.16, 0.06])
r_mag = RadioButtons(ax_mag, labels=["0","5","10"], active=0)
ax_mag.set_title("mag_rej")

# Check
ax_chk = fig.add_axes([0.82, 0.49, 0.16, 0.05])
c_overlay = CheckButtons(ax_chk, ["Overlay runs"], [False])

# Iteration speed
ax_spd = fig.add_axes([0.08, 0.02, 0.50, 0.03])
s_speed = Slider(ax_spd, "Play speed (ms/step)", 50, 1000, valinit=DEFAULT_SPEED_MS, valstep=10)

# Presets
ax_p1 = fig.add_axes([0.82, 0.43, 0.16, 0.045])
ax_p2 = fig.add_axes([0.82, 0.38, 0.16, 0.045])
ax_p3 = fig.add_axes([0.82, 0.33, 0.16, 0.045])
b_snap   = Button(ax_p1, "Preset: SNAP")
b_bal    = Button(ax_p2, "Preset: BAL")
b_noisy  = Button(ax_p3, "Preset: NOISY")

# Grid controls
ax_build = fig.add_axes([0.82, 0.27, 0.16, 0.045])
ax_prev  = fig.add_axes([0.82, 0.22, 0.16, 0.045])
ax_play  = fig.add_axes([0.82, 0.17, 0.16, 0.045])
ax_next  = fig.add_axes([0.82, 0.12, 0.16, 0.045])
b_build  = Button(ax_build, "Reset Grid")
b_prev   = Button(ax_prev,  "⟵ Back")
b_play   = Button(ax_play,  "Play / Pause")
b_next   = Button(ax_next,  "Next ⟶")

# Save/Best/Export
ax_note = fig.add_axes([0.62, 0.24, 0.15, 0.045])
t_note  = TextBox(ax_note, "Note:", initial="")

ax_save = fig.add_axes([0.62, 0.19, 0.15, 0.045])
ax_best = fig.add_axes([0.62, 0.14, 0.15, 0.045])
ax_csv  = fig.add_axes([0.62, 0.09, 0.15, 0.045])
b_save  = Button(ax_save, "Save Current")
b_best  = Button(ax_best, "Mark Best")
b_csv   = Button(ax_csv,  "Export CSV")

# Run/Reset (manual recompute)
ax_run = fig.add_axes([0.62, 0.30, 0.15, 0.045])
ax_rst = fig.add_axes([0.62, 0.04, 0.15, 0.045])
b_run  = Button(ax_run, "RUN", color="#e6f2ff")
b_rst  = Button(ax_rst, "Reset view")

# Status
status  = fig.text(0.08, 0.26, "", fontsize=10, family="monospace")
status2 = fig.text(0.08, 0.28, "", fontsize=10, family="monospace")
status3 = fig.text(0.62, 0.02, "", fontsize=9, family="monospace")

saved_rows = []

# =========================================================
# Helpers: params <-> UI
# =========================================================
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

def set_ui_from_params(p):
    # set slider/radio values without worrying about callbacks (we call do_run after)
    s_gain.set_val(p["gain"])
    # set radios by value
    try:
        gyro_labels = [int(l.get_text()) for l in r_gyro.labels]
        r_gyro.set_active(gyro_labels.index(int(p["gyro_range"])))
    except Exception:
        pass
    s_accrej.set_val(p["accel_rej"])
    try:
        mag_labels = [int(l.get_text()) for l in r_mag.labels]
        r_mag.set_active(mag_labels.index(int(p["mag_rej"])))
    except Exception:
        pass
    s_rt.set_val(max(0.5, min(5.0, p["rej_timeout"]/fs)))
    s_mt.set_val(p["motion_threshold"])
    s_sm.set_val(max(0.05, min(0.60, p["smoothing_margin"]/fs)))

def params_label(params, err, idx=None, total=None):
    prefix = f"[{idx+1}/{total}] " if (idx is not None and total is not None) else ""
    return (f"{prefix}g={params['gain']:.2f}, gr={params['gyro_range']}, ar={params['accel_rej']}, "
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

# =========================================================
# Plot update
# =========================================================
def do_run(event=None, highlight=False, idx=None, total=None):
    params = collect_params()
    pos, err = run_once(params)
    label = params_label(params, err, idx=idx, total=total)

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

# =========================================================
# Save / Best / Export
# =========================================================
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
    status3.set_text(f"Saved current: errXY={err:.2f}  (rows: {len(saved_rows)})")

def on_best(event=None):
    params, err = do_run(highlight=True, idx=current_index, total=len(GRID))
    note = t_note.text.strip()
    row = save_row(params, err, note, mark_best=True)
    with open("best_params.json", "w", encoding="utf-8") as f:
        json.dump(row, f, indent=2)
    status3.set_text(f"Marked BEST → best_params.json (errXY={err:.2f})")

def on_export_csv(event=None):
    if not saved_rows:
        status3.set_text("Nothing to export yet.")
        return
    with open("tuning_log.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(saved_rows[0].keys()))
        writer.writeheader()
        for r in saved_rows:
            writer.writerow(r)
    status3.set_text(f"Exported {len(saved_rows)} rows → tuning_log.csv")

# =========================================================
# Grid iterator
# =========================================================
def build_grid():
    combos = itertools.product(
        GRID_GAIN, GRID_GYRO_RANGE, GRID_ACCEL_REJ, GRID_MAG_REJ, GRID_RT_FACT, GRID_MT, GRID_SM_FACT
    )
    grid = []
    for g, gr, ar, mr, rtf, mt, smf in combos:
        grid.append({
            "gain": float(g),
            "gyro_range": int(gr),
            "accel_rej": int(ar),
            "mag_rej": int(mr),
            "rej_timeout": int(rtf * fs),
            "motion_threshold": float(mt),
            "smoothing_margin": int(smf * fs),
        })
        if len(grid) >= MAX_COMBOS:
            break
    return grid

GRID = build_grid()
current_index = 0
is_playing = False
timer = fig.canvas.new_timer(interval=DEFAULT_SPEED_MS)

def update_status2():
    status2.set_text(f"Combo {current_index+1} / {len(GRID)}")

def goto_index(i):
    global current_index
    current_index = max(0, min(len(GRID)-1, i))
    p = GRID[current_index]
    set_ui_from_params(p)
    do_run(idx=current_index, total=len(GRID))
    update_status2()

def step(delta):
    goto_index(current_index + delta)

def on_play(event=None):
    global is_playing
    if not GRID:
        return
    is_playing = not is_playing
    timer.stop()
    if is_playing:
        timer.interval = int(s_speed.val)  # ms
        timer.start()

def on_timer():
    if not is_playing:
        return
    step(1)
    if current_index >= len(GRID)-1:
        # stop at end
        on_play()  # toggle to paused

# Hook timer
timer.add_callback(on_timer)

def on_reset_grid(event=None):
    global GRID, current_index
    GRID = build_grid()
    current_index = 0
    update_status2()
    goto_index(0)

# =========================================================
# Wire up buttons and hotkeys
# =========================================================
b_run.on_clicked(lambda e: do_run(idx=current_index, total=len(GRID)))
b_rst.on_clicked(reset_view)
b_save.on_clicked(on_save)
b_best.on_clicked(on_best)
b_csv.on_clicked(on_export_csv)
b_snap.on_clicked(lambda e: (apply_preset("SNAP"), do_run()))
b_bal.on_clicked(lambda e: (apply_preset("BAL"), do_run()))
b_noisy.on_clicked(lambda e: (apply_preset("NOISY"), do_run()))
b_build.on_clicked(on_reset_grid)
b_prev.on_clicked(lambda e: step(-1))
b_next.on_clicked(lambda e: step(1))
b_play.on_clicked(on_play)

def on_key(event):
    if event.key in ("right", "n"):
        step(1)
    elif event.key in ("left",):
        step(-1)
    elif event.key in ("p", " "):  # p or space toggles play/pause
        on_play()
    elif event.key in ("s",):
        on_save()
    elif event.key in ("b",):
        on_best()

fig.canvas.mpl_connect("key_press_event", on_key)

# =========================================================
# Init
# =========================================================
goto_index(0)  # loads first combo & runs
plt.tight_layout()
plt.show()
