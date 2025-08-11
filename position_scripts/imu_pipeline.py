# imu_pipeline.py
# Full Python mirror of x-io "Oscillatory-Motion-Tracking-With-x-IMU/Script.m"
# (Adds: accel unit auto-detect (to g), safe 1g rescale clamp, gyro bias removal, dt spike clamp, diagnostics)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.signal import butter, filtfilt
import imufusion
import os
import csv

# =========================
# Config / Paths
# =========================
CSV_PATH = "../sensor_logs/2025-08-08 19-14-49.csv"   # change to your logger's file
HAS_HEADER = True
# Column order if no header: [time, gx, gy, gz, ax, ay, az]
TIME_COL = 0; GYR_COLS = (1,2,3); ACC_COLS = (4,5,6)

USE_FIXED_SAMPLE_PERIOD = False
SAMPLE_PERIOD = 1/256.0

# AHRS & filter params (aligned to MATLAB)
MAHONY_KP = 1.0
HPF_ORDER = 1
HPF_CUTOFF_HZ = 0.1

# Earth frame NWU: gravity ≈ +1 g on +Z after tilt-comp
GRAVITY_G_VEC = np.array([0.0, 0.0, 1.0])
G = 9.81

# Debug controls
DEBUG_PRINT = True
DEBUG_SAVE_DERIVATIVES_CSV = False

# =========================
# Helpers
# =========================
def load_csv(csv_path, has_header=True):
    if has_header:
        data = np.genfromtxt(csv_path, delimiter=",", names=True)
        names = [n.strip() for n in data.dtype.names]
        def pick(possible):
            for p in possible:
                if p in names:
                    return data[p]
            return None
        t  = pick(["time","timestamp","Time","Timestamp"])
        gx = pick(["gx","gyro_x","GyroscopeX","Gyroscope_X_deg_s","Gyroscope X (deg/s)","GyroscopeX_deg_s"])
        gy = pick(["gy","gyro_y","GyroscopeY","Gyroscope_Y_deg_s","Gyroscope Y (deg/s)","GyroscopeY_deg_s"])
        gz = pick(["gz","gyro_z","GyroscopeZ","Gyroscope_Z_deg_s","Gyroscope Z (deg/s)","GyroscopeZ_deg_s"])
        ax = pick(["ax","accel_x","AccelerometerX","Accelerometer_X_g","Accelerometer X (g)","AccelerometerX_g","Accelerometer X (m/s^2)","AccelX_ms2"])
        ay = pick(["ay","accel_y","AccelerometerY","Accelerometer_Y_g","Accelerometer Y (g)","AccelerometerY_g","Accelerometer Y (m/s^2)","AccelY_ms2"])
        az = pick(["az","accel_z","AccelerometerZ","Accelerometer_Z_g","Accelerometer Z (g)","AccelerometerZ_g","Accelerometer Z (m/s^2)","AccelZ_ms2"])
        if t is None:
            N = len(gx)
            t = np.arange(N) * SAMPLE_PERIOD
        gyr = np.vstack([gx, gy, gz]).T.astype(float)
        acc_raw = np.vstack([ax, ay, az]).T.astype(float)
        return t.astype(float), gyr, acc_raw
    else:
        data = np.genfromtxt(csv_path, delimiter=",", skip_header=0)
        t   = data[:, TIME_COL]
        gyr = data[:, GYR_COLS]
        acc_raw = data[:, ACC_COLS]
        return t.astype(float), gyr.astype(float), acc_raw.astype(float)

def compute_sample_period_from_dt(dt):
    return float(np.mean(dt[1:])) if len(dt) > 1 else float(dt[0])

# -------- NEW: unit auto-detect so accelerometer is normalized to g --------
def normalize_acc_units_to_g(acc_raw):
    """
    Returns acc_g, unit_tag
      - If median|acc| in [7,20]: assume m/s^2 -> divide by 9.81
      - If median|acc| in [700, 20000]: assume mg -> divide by 1000
      - Else: assume already in g
    """
    m = np.nanmedian(np.linalg.norm(acc_raw, axis=1))
    if 7.0 <= m <= 20.0:
        return acc_raw / G, "m/s^2→g (/9.81)"
    elif 700.0 <= m <= 20000.0:
        return acc_raw / 1000.0, "mg→g (/1000)"
    else:
        return acc_raw, "assumed g (no change)"

def rescale_accel_to_1g(acc_g, use_first_seconds=None, sample_rate=None, clamp=(0.2, 5.0)):
    """
    Scale accelerometer so median |acc| ≈ 1 g using an early segment,
    but clamp the gain to avoid explosions due to spikes/outliers.
    """
    if use_first_seconds and sample_rate:
        N = min(len(acc_g), int(use_first_seconds * sample_rate))
        seg = acc_g[:N]
    else:
        seg = acc_g
    m = np.nanmedian(np.linalg.norm(seg, axis=1))
    if m <= 0:
        scale = 1.0
    else:
        scale = 1.0 / m
    # clamp
    scale = float(np.clip(scale, clamp[0], clamp[1]))
    return acc_g * scale, scale

def estimate_gyro_bias(gyr_dps, acc_g, sample_rate, max_calib_sec=3.0):
    """Use early near-still samples for bias; fallback to simple mean."""
    N = min(len(gyr_dps), int(max_calib_sec*sample_rate))
    gnorm = np.linalg.norm(acc_g[:N], axis=1)
    gyro_norm = np.linalg.norm(gyr_dps[:N], axis=1)
    stat = (np.abs(gnorm - 1.0) < 0.15) & (gyro_norm < 5.0)
    return np.mean(gyr_dps[:N][stat], axis=0) if np.any(stat) else np.mean(gyr_dps[:N], axis=0)

def mahony_ahrs(gyr_dps, acc_g, delta_time, kp=1.0):
    """Return R[N,3,3] Sensor->Earth using imufusion; delta_time is per-sample."""
    def quat_to_rotmat(q):
        w, x, y, z = float(q.w), float(q.x), float(q.y), float(q.z)
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        return np.array([
            [1 - 2*(yy + zz),  2*(xy - wz),      2*(xz + wy)],
            [2*(xy + wz),      1 - 2*(xx + zz),  2*(yz - wx)],
            [2*(xz - wy),      2*(yz + wx),      1 - 2*(xx + yy)]
        ], dtype=float)

    ahrs = imufusion.Ahrs()
    ahrs.settings = imufusion.Settings(
        imufusion.CONVENTION_NWU, kp, 2000, 0, 0,
        int(3 * (1.0 / max(np.mean(delta_time), 1e-6)))
    )
    gyr_rad = np.deg2rad(gyr_dps.astype(float))
    R = np.zeros((len(gyr_dps), 3, 3))
    upd = ahrs.update_no_magnetometer if hasattr(ahrs, "update_no_magnetometer") else ahrs.updateNoMagnetometer
    for i in range(len(gyr_dps)):
        upd(gyr_rad[i], acc_g[i], float(delta_time[i]))
        Re2s = quat_to_rotmat(ahrs.quaternion)  # Earth->Sensor
        R[i] = Re2s.T                            # Sensor->Earth
    return R

def tilt_compensate(acc_g, R):
    return (R @ acc_g[..., None]).squeeze(-1)

def highpass(data, order, cutoff_hz, sample_rate_hz):
    b, a = butter(order, (2.0 * cutoff_hz) / sample_rate_hz, btype="high")
    return filtfilt(b, a, data, axis=0, method="gust")

def integrate(data, dt):
    out = np.zeros_like(data)
    np.cumsum(data * dt, axis=0, out=out)
    return out

# ---------- Debug utilities ----------
def _fmt3(v): return f"[{v[0]: .4f}, {v[1]: .4f}, {v[2]: .4f}]"

def debug_report(t, gyr_dps, acc_g, unit_tag, scale_acc, gyro_bias, tcAcc_g, linAcc_mps2, linVel_mps, linVelHP_mps, linPos_m, linPosHP_m, sample_period):
    N = len(t)
    dt = np.diff(t, prepend=t[0])
    dur = t[-1] - t[0]
    sr  = 1.0 / sample_period
    acc_norm = np.linalg.norm(acc_g, axis=1)
    out = []
    out.append(f"\n=== DEBUG STATS ===")
    out.append(f"samples: {N:,}   duration: {dur:.3f} s   mean fs: {sr:.3f} Hz")
    out.append(f"dt: mean={dt[1:].mean():.6f} s  std={dt[1:].std():.6f} s  min={dt[1:].min():.6f}  max={dt[1:].max():.6f}")
    out.append(f"acc units normalization: {unit_tag}")
    out.append(f"|acc_g| median AFTER norm = {np.median(acc_norm):.3f} g  (extra scale→1g: ×{scale_acc:.3f})")
    out.append(f"gyro bias removed (dps): {_fmt3(gyro_bias)}")
    out.append(f"linAcc mean (m/s^2): {_fmt3(linAcc_mps2.mean(axis=0))}")
    out.append(f"vel final (m/s): {_fmt3(linVel_mps[-1])}   velHP final: {_fmt3(linVelHP_mps[-1])}")
    out.append(f"pos span (unfiltered) (m): {_fmt3(linPos_m.max(axis=0) - linPos_m.min(axis=0))}")
    out.append(f"pos span (HPF)        (m): {_fmt3(linPosHP_m.max(axis=0) - linPosHP_m.min(axis=0))}")
    out.append(f"pos final (HPF)       (m): {_fmt3(linPosHP_m[-1])}")
    print("\n".join(out))

def plot_diagnostics(t, gyr_dps, acc_g, dt, linAcc_mps2, linVel_mps, linPos_m):
    fig, ax = plt.subplots(2, 3, figsize=(15, 7)); ax = ax.ravel()
    ax[0].plot(dt, lw=0.8); ax[0].set_title("dt (s)"); ax[0].grid(True)
    ax[1].plot(np.linalg.norm(acc_g, axis=1), lw=0.8); ax[1].axhline(1.0, ls="--"); ax[1].set_title("|acc_g| (g)"); ax[1].grid(True)
    ax[2].plot(np.linalg.norm(gyr_dps, axis=1), lw=0.8); ax[2].set_title("|gyro| (dps)"); ax[2].grid(True)
    ax[3].plot(linAcc_mps2[:,0], "r", lw=0.8); ax[3].plot(linAcc_mps2[:,1], "g", lw=0.8); ax[3].plot(linAcc_mps2[:,2], "b", lw=0.8)
    ax[3].set_title("Linear accel (m/s²)"); ax[3].grid(True)
    ax[4].plot(linVel_mps[:,0], "r", lw=0.8); ax[4].plot(linVel_mps[:,1], "g", lw=0.8); ax[4].plot(linVel_mps[:,2], "b", lw=0.8)
    ax[4].set_title("Velocity (unfiltered, m/s)"); ax[4].grid(True)
    ax[5].plot(linPos_m[:,0], "r", lw=0.8); ax[5].plot(linPos_m[:,1], "g", lw=0.8); ax[5].plot(linPos_m[:,2], "b", lw=0.8)
    ax[5].set_title("Position (unfiltered, m)"); ax[5].grid(True)
    fig.suptitle("Diagnostics", y=0.98, fontsize=12); fig.tight_layout(rect=[0,0,1,0.96])

def save_derivatives_csv(path_base, tcAcc_g, linAcc_mps2, linVel_mps, linVelHP_mps, linPos_m, linPosHP_m):
    out_path = os.path.splitext(path_base)[0] + "_derived.csv"
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "tcAcc_g_x","tcAcc_g_y","tcAcc_g_z",
            "linAcc_x","linAcc_y","linAcc_z",
            "vel_x","vel_y","vel_z",
            "velHP_x","velHP_y","velHP_z",
            "pos_x","pos_y","pos_z",
            "posHP_x","posHP_y","posHP_z"
        ])
        for i in range(len(tcAcc_g)):
            w.writerow([
                *tcAcc_g[i].tolist(),
                *linAcc_mps2[i].tolist(),
                *linVel_mps[i].tolist(),
                *linVelHP_mps[i].tolist(),
                *linPos_m[i].tolist(),
                *linPosHP_m[i].tolist(),
            ])
    print(f"[debug] wrote derived signals to {out_path}")

# =========================
# Main pipeline
# =========================
def run_pipeline():
    if not os.path.exists(CSV_PATH):
        raise FileNotFoundError(f"CSV not found: {CSV_PATH}")

    # Import data
    t, gyr_dps, acc_raw = load_csv(CSV_PATH, has_header=HAS_HEADER)

    # Build dt; clamp big spikes to median to avoid 0.2s impulses
    dt = np.diff(t, prepend=t[0])
    if len(dt) > 1:
        med = np.median(dt[1:])
        dt = dt.copy()
        dt[dt > 3.0*med] = med
    dt[0] = dt[1] if len(dt) > 1 else (SAMPLE_PERIOD if USE_FIXED_SAMPLE_PERIOD else dt[0])
    delta_time = dt.astype(float)

    # Sample period & rate from (possibly clamped) dt
    sample_period = SAMPLE_PERIOD if USE_FIXED_SAMPLE_PERIOD else compute_sample_period_from_dt(delta_time)
    sample_rate = 1.0 / sample_period

    # --- 1) Normalize accel units to g
    acc_g, unit_tag = normalize_acc_units_to_g(acc_raw)

    # --- 2) Safe rescale towards 1 g using early window (clamped)
    acc_g, scale_acc = rescale_accel_to_1g(acc_g, use_first_seconds=3.0, sample_rate=sample_rate, clamp=(0.2, 5.0))

    # --- 3) Gyro bias removal (use early near-still samples)
    gyro_bias = estimate_gyro_bias(gyr_dps, acc_g, sample_rate)
    gyr_dps = gyr_dps - gyro_bias

    # === AHRS: orientation (Sensor->Earth rotation matrices) ===
    R = mahony_ahrs(gyr_dps, acc_g, delta_time, kp=MAHONY_KP)

    # === Tilt-compensated accelerometer (Earth frame, still in g) ===
    tcAcc_g = tilt_compensate(acc_g, R)

    # === Linear acceleration (subtract gravity) and convert to m/s^2 ===
    linAcc_mps2 = (tcAcc_g - GRAVITY_G_VEC) * G

    # === Integrate to linear velocity ===
    linVel_mps = integrate(linAcc_mps2, sample_period)

    # === High-pass filter velocity ===
    linVelHP_mps = highpass(linVel_mps, HPF_ORDER, HPF_CUTOFF_HZ, sample_rate)

    # === Integrate to linear position ===
    linPos_m = integrate(linVelHP_mps, sample_period)

    # === High-pass filter position ===
    linPosHP_m = highpass(linPos_m, HPF_ORDER, HPF_CUTOFF_HZ, sample_rate)

    # ---------- Terminal debug ----------
    if DEBUG_PRINT:
        debug_report(t, gyr_dps, acc_g, unit_tag, scale_acc, gyro_bias, tcAcc_g, linAcc_mps2, linVel_mps, linVelHP_mps, linPos_m, linPosHP_m, sample_period)

    # Optional CSV dump
    if DEBUG_SAVE_DERIVATIVES_CSV:
        save_derivatives_csv(CSV_PATH, tcAcc_g, linAcc_mps2, linVel_mps, linVelHP_mps, linPos_m, linPosHP_m)

    # ---------- Diagnostics figure ----------
    plot_diagnostics(t, gyr_dps, acc_g, delta_time, linAcc_mps2, linVel_mps, linPos_m)

    # === Main 2x3 figure ===
    fig, axes = plt.subplots(2, 3, figsize=(15, 8)); axes = axes.ravel()

    axes[0].plot(gyr_dps[:,0], "r", label="X"); axes[0].plot(gyr_dps[:,1], "g", label="Y"); axes[0].plot(gyr_dps[:,2], "b", label="Z")
    axes[0].set_title("Gyroscope (bias removed)"); axes[0].set_xlabel("sample"); axes[0].set_ylabel("dps"); axes[0].legend(); axes[0].grid(True)

    axes[1].plot(acc_g[:,0], "r", label="X"); axes[1].plot(acc_g[:,1], "g", label="Y"); axes[1].plot(acc_g[:,2], "b", label="Z")
    axes[1].set_title("Accelerometer (normalized to g)"); axes[1].set_xlabel("sample"); axes[1].set_ylabel("g"); axes[1].legend(); axes[1].grid(True)

    axes[2].plot(tcAcc_g[:,0], "r", label="X"); axes[2].plot(tcAcc_g[:,1], "g", label="Y"); axes[2].plot(tcAcc_g[:,2], "b", label="Z")
    axes[2].set_title("'Tilt-compensated' accel"); axes[2].set_xlabel("sample"); axes[2].set_ylabel("g"); axes[2].legend(); axes[2].grid(True)

    axes[3].plot(linAcc_mps2[:,0], "r", label="X"); axes[3].plot(linAcc_mps2[:,1], "g", label="Y"); axes[3].plot(linAcc_mps2[:,2], "b", label="Z")
    axes[3].set_title("Linear acceleration"); axes[3].set_xlabel("sample"); axes[3].set_ylabel("m/s²"); axes[3].legend(); axes[3].grid(True)

    axes[4].plot(linVelHP_mps[:,0], "r", label="X"); axes[4].plot(linVelHP_mps[:,1], "g", label="Y"); axes[4].plot(linVelHP_mps[:,2], "b", label="Z")
    axes[4].set_title("HPF linear velocity"); axes[4].set_xlabel("sample"); axes[4].set_ylabel("m/s"); axes[4].legend(); axes[4].grid(True)

    axes[5].plot(linPosHP_m[:,0], "r", label="X"); axes[5].plot(linPosHP_m[:,1], "g", label="Y"); axes[5].plot(linPosHP_m[:,2], "b", label="Z")
    axes[5].set_title("HPF linear position"); axes[5].set_xlabel("sample"); axes[5].set_ylabel("m"); axes[5].legend(); axes[5].grid(True)

    fig.tight_layout()

    # === 3D animation (trajectory + body axes) ===
    global _ANIS
    try:
        _ANIS
    except NameError:
        _ANIS = []
    fig_anim, ani = animate_3d(linPosHP_m, R, 1.0/sample_period)
    _ANIS.append(ani)

    plt.show()
    return dict(R=R, tcAcc_g=tcAcc_g, linAcc_mps2=linAcc_mps2,
                linVelHP_mps=linVelHP_mps, linPosHP_m=linPosHP_m)

# -------------------------
# Simple 3D animation (MATLAB SixDOFanimation analogue)
# -------------------------
def animate_3d(pos, R, sample_rate, trail=False):
    from mpl_toolkits.mplot3d import Axes3D  # noqa
    N = pos.shape[0]
    fig = plt.figure("Unfiltered (3D Animation)", figsize=(10,6))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("Unfiltered")
    Pmin = pos.min(axis=0); Pmax = pos.max(axis=0)
    span = (Pmax - Pmin); center = (Pmax + Pmin)/2
    if np.all(span == 0): span = np.array([1,1,1])
    max_range = 0.5 * np.max(span) + 0.1
    ax.set_xlim(center[0]-max_range, center[0]+max_range)
    ax.set_ylim(center[1]-max_range, center[1]+max_range)
    ax.set_zlim(center[2]-max_range, center[2]+max_range)
    traj_line, = ax.plot([], [], [], lw=1)
    point = ax.scatter([], [], [], s=20)
    L = 0.1
    bx, = ax.plot([], [], [], lw=2)
    by, = ax.plot([], [], [], lw=2)
    bz, = ax.plot([], [], [], lw=2)
    xs, ys, zs = [], [], []
    def update(i):
        p = pos[i]
        xs.append(p[0]); ys.append(p[1]); zs.append(p[2])
        if not trail and len(xs) > 200:
            xs[:len(xs)-200] = []; ys[:len(ys)-200] = []; zs[:len(zs)-200] = []
        traj_line.set_data(xs, ys); traj_line.set_3d_properties(zs)
        point._offsets3d = ([p[0]],[p[1]],[p[2]])
        Re = R[i]; origin = p
        Xend = origin + Re[:,0]*L; Yend = origin + Re[:,1]*L; Zend = origin + Re[:,2]*L
        bx.set_data([origin[0], Xend[0]], [origin[1], Xend[1]]); bx.set_3d_properties([origin[2], Xend[2]])
        by.set_data([origin[0], Yend[0]], [origin[1], Yend[1]]); by.set_3d_properties([origin[2], Yend[2]])
        bz.set_data([origin[0], Zend[0]], [origin[1], Zend[1]]); bz.set_3d_properties([origin[2], Zend[2]])
        return traj_line, point, bx, by, bz
    interval_ms = 1000.0 / max(sample_rate/8.0, 1.0)
    ani = FuncAnimation(fig, update, frames=N, interval=interval_ms, blit=False)
    return fig, ani

# =========================
# Run
# =========================
if __name__ == "__main__":
    run_pipeline()
