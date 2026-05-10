from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DATA_DIR = Path("data/simulated")
RESULTS_DIR = Path("results")

OUTPUT_CSV = DATA_DIR / "simulated_sensor_data.csv"
OUTPUT_PLOT = RESULTS_DIR / "true_vs_gps_measurements.png"
OUTPUT_ACCEL_PLOT = RESULTS_DIR / "true_vs_imu_acceleration.png"

DT = 0.1
DURATION = 60.0
RANDOM_SEED = 42

GPS_NOISE_STD_M = 3.0
IMU_ACCEL_NOISE_STD = 0.15

GPS_MEASUREMENT_INTERVAL = 5  # GPS available every 5 simulation steps
GPS_DROPOUT_START_S = 25.0
GPS_DROPOUT_END_S = 35.0


def acceleration_profile(t):
    """
    Scripted 2D acceleration profile for an autonomous platform.

    The profile creates a smooth trajectory with acceleration, turning,
    and deceleration phases. This keeps the project physically interpretable
    while still producing a non-trivial path for estimation.
    """
    ax = 0.0
    ay = 0.0

    if 0.0 <= t < 8.0:
        ax = 0.35
        ay = 0.05
    elif 8.0 <= t < 18.0:
        ax = 0.05
        ay = 0.25
    elif 18.0 <= t < 30.0:
        ax = -0.10
        ay = 0.15
    elif 30.0 <= t < 42.0:
        ax = -0.20
        ay = -0.10
    elif 42.0 <= t < 52.0:
        ax = 0.10
        ay = -0.25
    else:
        ax = 0.0
        ay = 0.0

    return ax, ay


def simulate_true_motion():
    times = np.arange(0.0, DURATION + DT, DT)
    n_steps = len(times)

    true_x = np.zeros(n_steps)
    true_y = np.zeros(n_steps)
    true_vx = np.zeros(n_steps)
    true_vy = np.zeros(n_steps)
    true_ax = np.zeros(n_steps)
    true_ay = np.zeros(n_steps)

    # Initial velocity gives the platform forward motion from the start.
    true_vx[0] = 1.5
    true_vy[0] = 0.3

    for k in range(1, n_steps):
        t = times[k - 1]
        ax, ay = acceleration_profile(t)

        true_ax[k - 1] = ax
        true_ay[k - 1] = ay

        true_vx[k] = true_vx[k - 1] + ax * DT
        true_vy[k] = true_vy[k - 1] + ay * DT

        true_x[k] = true_x[k - 1] + true_vx[k - 1] * DT + 0.5 * ax * DT**2
        true_y[k] = true_y[k - 1] + true_vy[k - 1] * DT + 0.5 * ay * DT**2

    true_ax[-1], true_ay[-1] = acceleration_profile(times[-1])

    return times, true_x, true_y, true_vx, true_vy, true_ax, true_ay


def generate_sensor_measurements(times, true_x, true_y, true_ax, true_ay):
    rng = np.random.default_rng(RANDOM_SEED)

    n_steps = len(times)

    imu_ax = true_ax + rng.normal(0.0, IMU_ACCEL_NOISE_STD, size=n_steps)
    imu_ay = true_ay + rng.normal(0.0, IMU_ACCEL_NOISE_STD, size=n_steps)

    gps_x = np.full(n_steps, np.nan)
    gps_y = np.full(n_steps, np.nan)
    gps_available = np.zeros(n_steps, dtype=int)

    for k, t in enumerate(times):
        is_gps_time = k % GPS_MEASUREMENT_INTERVAL == 0
        is_dropout = GPS_DROPOUT_START_S <= t <= GPS_DROPOUT_END_S

        if is_gps_time and not is_dropout:
            gps_x[k] = true_x[k] + rng.normal(0.0, GPS_NOISE_STD_M)
            gps_y[k] = true_y[k] + rng.normal(0.0, GPS_NOISE_STD_M)
            gps_available[k] = 1

    return imu_ax, imu_ay, gps_x, gps_y, gps_available


def build_dataset():
    (
        times,
        true_x,
        true_y,
        true_vx,
        true_vy,
        true_ax,
        true_ay,
    ) = simulate_true_motion()

    imu_ax, imu_ay, gps_x, gps_y, gps_available = generate_sensor_measurements(
        times=times,
        true_x=true_x,
        true_y=true_y,
        true_ax=true_ax,
        true_ay=true_ay,
    )

    df = pd.DataFrame(
        {
            "time_s": times,
            "true_x_m": true_x,
            "true_y_m": true_y,
            "true_vx_mps": true_vx,
            "true_vy_mps": true_vy,
            "true_ax_mps2": true_ax,
            "true_ay_mps2": true_ay,
            "imu_ax_mps2": imu_ax,
            "imu_ay_mps2": imu_ay,
            "gps_x_m": gps_x,
            "gps_y_m": gps_y,
            "gps_available": gps_available,
        }
    )

    return df


def save_trajectory_plot(df):
    fig, ax = plt.subplots(figsize=(8, 6))

    ax.plot(
        df["true_x_m"],
        df["true_y_m"],
        label="True trajectory",
        linewidth=2,
    )

    gps_df = df[df["gps_available"] == 1]

    ax.scatter(
        gps_df["gps_x_m"],
        gps_df["gps_y_m"],
        s=16,
        alpha=0.65,
        label="Noisy GPS-like measurements",
    )

    dropout_df = df[
        (df["time_s"] >= GPS_DROPOUT_START_S)
        & (df["time_s"] <= GPS_DROPOUT_END_S)
    ]

    ax.plot(
        dropout_df["true_x_m"],
        dropout_df["true_y_m"],
        linewidth=4,
        alpha=0.35,
        label="GPS dropout interval",
    )

    ax.set_xlabel("x position (m)")
    ax.set_ylabel("y position (m)")
    ax.set_title("True Trajectory and Noisy GPS-like Measurements")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()
    plt.savefig(OUTPUT_PLOT, dpi=150)
    plt.close()

    print(f"Saved trajectory plot to: {OUTPUT_PLOT}")


def save_acceleration_plot(df):
    fig, ax = plt.subplots(figsize=(9, 5))

    ax.plot(
        df["time_s"],
        df["true_ax_mps2"],
        label="True ax",
        linewidth=2,
    )
    ax.plot(
        df["time_s"],
        df["imu_ax_mps2"],
        label="Noisy IMU ax",
        alpha=0.6,
    )
    ax.plot(
        df["time_s"],
        df["true_ay_mps2"],
        label="True ay",
        linewidth=2,
    )
    ax.plot(
        df["time_s"],
        df["imu_ay_mps2"],
        label="Noisy IMU ay",
        alpha=0.6,
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (m/s²)")
    ax.set_title("True vs IMU-like Acceleration Measurements")
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()
    plt.savefig(OUTPUT_ACCEL_PLOT, dpi=150)
    plt.close()

    print(f"Saved acceleration plot to: {OUTPUT_ACCEL_PLOT}")


def main():
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    df = build_dataset()
    df.to_csv(OUTPUT_CSV, index=False)

    print(f"Saved simulated sensor dataset to: {OUTPUT_CSV}")
    print(f"Dataset shape: {df.shape}")
    print()
    print("Columns:")
    for col in df.columns:
        print(f"- {col}")

    print()
    print("GPS availability:")
    print(df["gps_available"].value_counts().sort_index())

    save_trajectory_plot(df)
    save_acceleration_plot(df)


if __name__ == "__main__":
    main()
