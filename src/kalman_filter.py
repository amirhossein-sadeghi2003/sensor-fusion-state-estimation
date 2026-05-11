from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DATA_PATH = Path("data/simulated/simulated_sensor_data.csv")
RESULTS_DIR = Path("results")

ESTIMATED_CSV = Path("data/simulated/kalman_estimates.csv")
RMSE_CSV = RESULTS_DIR / "rmse_comparison.csv"

TRAJECTORY_PLOT = RESULTS_DIR / "kalman_estimated_trajectory.png"
ERROR_PLOT = RESULTS_DIR / "position_error_over_time.png"
DROPOUT_PLOT = RESULTS_DIR / "gps_dropout_estimation.png"

DT = 0.1

# Measurement noise: matches the GPS-like noise level used in simulation.
GPS_NOISE_STD_M = 3.0

# Process noise controls how much uncertainty we allow in the motion model.
# This is tuned for the simulated IMU-like acceleration input.
PROCESS_NOISE_STD = 0.35


def load_sensor_data():
    return pd.read_csv(DATA_PATH)


def initialize_filter(first_row):
    """
    State vector:
        x = [position_x, position_y, velocity_x, velocity_y]^T

    The first available GPS measurement initializes position.
    Velocity is initialized to zero because only position measurements are
    directly available from the GPS-like sensor.
    """
    initial_x = first_row["gps_x_m"]
    initial_y = first_row["gps_y_m"]

    state = np.array(
        [
            initial_x,
            initial_y,
            0.0,
            0.0,
        ],
        dtype=float,
    )

    covariance = np.diag([GPS_NOISE_STD_M**2, GPS_NOISE_STD_M**2, 10.0, 10.0])

    return state, covariance


def build_matrices(dt):
    """
    Constant-velocity state transition model with acceleration as control input.

    State:
        [x, y, vx, vy]

    Control input:
        [ax, ay]

    Prediction:
        x_k = F x_{k-1} + B u_k
    """
    F = np.array(
        [
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    B = np.array(
        [
            [0.5 * dt**2, 0.0],
            [0.0, 0.5 * dt**2],
            [dt, 0.0],
            [0.0, dt],
        ]
    )

    H = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ]
    )

    q = PROCESS_NOISE_STD**2

    Q = q * np.array(
        [
            [0.25 * dt**4, 0.0, 0.5 * dt**3, 0.0],
            [0.0, 0.25 * dt**4, 0.0, 0.5 * dt**3],
            [0.5 * dt**3, 0.0, dt**2, 0.0],
            [0.0, 0.5 * dt**3, 0.0, dt**2],
        ]
    )

    R = np.diag([GPS_NOISE_STD_M**2, GPS_NOISE_STD_M**2])

    return F, B, H, Q, R


def run_kalman_filter(df):
    gps_rows = df[df["gps_available"] == 1]

    if gps_rows.empty:
        raise ValueError("No GPS measurements available for initialization.")

    first_gps_index = gps_rows.index[0]
    first_gps_row = df.loc[first_gps_index]

    state, covariance = initialize_filter(first_gps_row)
    F, B, H, Q, R = build_matrices(DT)

    estimates = []

    identity = np.eye(4)

    for index, row in df.iterrows():
        ax = row["imu_ax_mps2"]
        ay = row["imu_ay_mps2"]
        control = np.array([ax, ay], dtype=float)

        # Prediction step
        state = F @ state + B @ control
        covariance = F @ covariance @ F.T + Q

        # Correction step, only when GPS-like measurement is available
        if row["gps_available"] == 1:
            measurement = np.array([row["gps_x_m"], row["gps_y_m"]], dtype=float)

            innovation = measurement - H @ state
            innovation_covariance = H @ covariance @ H.T + R
            kalman_gain = covariance @ H.T @ np.linalg.inv(innovation_covariance)

            state = state + kalman_gain @ innovation
            covariance = (identity - kalman_gain @ H) @ covariance

        estimates.append(
            {
                "time_s": row["time_s"],
                "estimated_x_m": state[0],
                "estimated_y_m": state[1],
                "estimated_vx_mps": state[2],
                "estimated_vy_mps": state[3],
                "gps_available": int(row["gps_available"]),
            }
        )

    estimates_df = pd.DataFrame(estimates)

    merged = pd.concat(
        [
            df.reset_index(drop=True),
            estimates_df[
                [
                    "estimated_x_m",
                    "estimated_y_m",
                    "estimated_vx_mps",
                    "estimated_vy_mps",
                ]
            ],
        ],
        axis=1,
    )

    return merged


def compute_position_errors(df):
    df = df.copy()

    df["gps_position_error_m"] = np.sqrt(
        (df["gps_x_m"] - df["true_x_m"]) ** 2
        + (df["gps_y_m"] - df["true_y_m"]) ** 2
    )

    df["kalman_position_error_m"] = np.sqrt(
        (df["estimated_x_m"] - df["true_x_m"]) ** 2
        + (df["estimated_y_m"] - df["true_y_m"]) ** 2
    )

    gps_rmse = np.sqrt(
        np.nanmean(
            (df["gps_x_m"] - df["true_x_m"]) ** 2
            + (df["gps_y_m"] - df["true_y_m"]) ** 2
        )
    )

    kalman_rmse = np.sqrt(
        np.mean(
            (df["estimated_x_m"] - df["true_x_m"]) ** 2
            + (df["estimated_y_m"] - df["true_y_m"]) ** 2
        )
    )

    gps_available_df = df[df["gps_available"] == 1]

    kalman_rmse_at_gps_times = np.sqrt(
        np.mean(
            (gps_available_df["estimated_x_m"] - gps_available_df["true_x_m"]) ** 2
            + (gps_available_df["estimated_y_m"] - gps_available_df["true_y_m"]) ** 2
        )
    )

    rmse_df = pd.DataFrame(
        [
            {
                "method": "Raw GPS-like measurements",
                "position_rmse_m": gps_rmse,
                "notes": "Computed only when GPS-like measurements are available",
            },
            {
                "method": "Kalman filter estimate",
                "position_rmse_m": kalman_rmse,
                "notes": "Computed over the full trajectory",
            },
            {
                "method": "Kalman estimate at GPS timestamps",
                "position_rmse_m": kalman_rmse_at_gps_times,
                "notes": "Computed only at GPS measurement timestamps",
            },
        ]
    )

    return df, rmse_df


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
        alpha=0.55,
        label="Noisy GPS-like measurements",
    )

    ax.plot(
        df["estimated_x_m"],
        df["estimated_y_m"],
        label="Kalman estimate",
        linewidth=2,
    )

    ax.set_xlabel("x position (m)")
    ax.set_ylabel("y position (m)")
    ax.set_title("Kalman Filter Trajectory Estimation")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()
    plt.savefig(TRAJECTORY_PLOT, dpi=150)
    plt.close()

    print(f"Saved trajectory estimation plot to: {TRAJECTORY_PLOT}")


def save_error_plot(df):
    fig, ax = plt.subplots(figsize=(9, 5))

    ax.plot(
        df["time_s"],
        df["kalman_position_error_m"],
        label="Kalman position error",
        linewidth=2,
    )

    gps_df = df[df["gps_available"] == 1]

    ax.scatter(
        gps_df["time_s"],
        gps_df["gps_position_error_m"],
        s=18,
        alpha=0.65,
        label="Raw GPS-like error",
    )

    ax.axvspan(
        25.0,
        35.0,
        alpha=0.15,
        label="GPS dropout interval",
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position error (m)")
    ax.set_title("Position Error Over Time")
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()
    plt.savefig(ERROR_PLOT, dpi=150)
    plt.close()

    print(f"Saved position error plot to: {ERROR_PLOT}")


def save_dropout_plot(df):
    dropout_df = df[(df["time_s"] >= 20.0) & (df["time_s"] <= 40.0)]

    fig, ax = plt.subplots(figsize=(8, 6))

    ax.plot(
        dropout_df["true_x_m"],
        dropout_df["true_y_m"],
        label="True trajectory",
        linewidth=2,
    )

    gps_df = dropout_df[dropout_df["gps_available"] == 1]

    ax.scatter(
        gps_df["gps_x_m"],
        gps_df["gps_y_m"],
        s=18,
        alpha=0.65,
        label="GPS-like measurements",
    )

    ax.plot(
        dropout_df["estimated_x_m"],
        dropout_df["estimated_y_m"],
        label="Kalman estimate",
        linewidth=2,
    )

    during_dropout = dropout_df[
        (dropout_df["time_s"] >= 25.0) & (dropout_df["time_s"] <= 35.0)
    ]

    ax.plot(
        during_dropout["true_x_m"],
        during_dropout["true_y_m"],
        linewidth=5,
        alpha=0.25,
        label="GPS dropout interval",
    )

    ax.set_xlabel("x position (m)")
    ax.set_ylabel("y position (m)")
    ax.set_title("Kalman Estimation During GPS Dropout")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()
    plt.savefig(DROPOUT_PLOT, dpi=150)
    plt.close()

    print(f"Saved GPS dropout estimation plot to: {DROPOUT_PLOT}")


def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    ESTIMATED_CSV.parent.mkdir(parents=True, exist_ok=True)

    df = load_sensor_data()

    estimated_df = run_kalman_filter(df)
    estimated_df, rmse_df = compute_position_errors(estimated_df)

    estimated_df.to_csv(ESTIMATED_CSV, index=False)
    rmse_df.to_csv(RMSE_CSV, index=False)

    print(f"Saved Kalman estimates to: {ESTIMATED_CSV}")
    print(f"Saved RMSE comparison to: {RMSE_CSV}")
    print()
    print("RMSE comparison:")
    print(rmse_df.to_string(index=False))

    save_trajectory_plot(estimated_df)
    save_error_plot(estimated_df)
    save_dropout_plot(estimated_df)


if __name__ == "__main__":
    main()
