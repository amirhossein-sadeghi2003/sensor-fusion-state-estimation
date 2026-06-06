from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from kalman_filter import compute_position_errors, run_kalman_filter
from simulate_motion import (
    DT,
    GPS_DROPOUT_END_S,
    GPS_DROPOUT_START_S,
    GPS_MEASUREMENT_INTERVAL,
    GPS_NOISE_STD_M,
    IMU_ACCEL_NOISE_STD,
    RANDOM_SEED,
    simulate_true_motion,
)


RESULTS_DIR = Path("results")
OUTPUT_CSV = RESULTS_DIR / "imu_bias_sensitivity.csv"
OUTPUT_PLOT = RESULTS_DIR / "imu_bias_rmse_comparison.png"
OUTPUT_TRAJECTORY_PLOT = RESULTS_DIR / "imu_bias_trajectory_comparison.png"

IMU_BIAS_LEVELS_MPS2 = [0.0, 0.02, 0.05, 0.10, 0.20, 0.35]
N_REPEATS = 5


def generate_measurements_with_imu_bias(
    times,
    true_x,
    true_y,
    true_ax,
    true_ay,
    imu_bias_ax,
    imu_bias_ay,
    random_seed,
):
    rng = np.random.default_rng(random_seed)

    n_steps = len(times)

    imu_ax = true_ax + imu_bias_ax + rng.normal(
        0.0,
        IMU_ACCEL_NOISE_STD,
        size=n_steps,
    )
    imu_ay = true_ay + imu_bias_ay + rng.normal(
        0.0,
        IMU_ACCEL_NOISE_STD,
        size=n_steps,
    )

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


def build_dataset_for_bias(imu_bias_level, random_seed):
    (
        times,
        true_x,
        true_y,
        true_vx,
        true_vy,
        true_ax,
        true_ay,
    ) = simulate_true_motion()

    imu_bias_ax = imu_bias_level
    imu_bias_ay = -0.5 * imu_bias_level

    imu_ax, imu_ay, gps_x, gps_y, gps_available = generate_measurements_with_imu_bias(
        times=times,
        true_x=true_x,
        true_y=true_y,
        true_ax=true_ax,
        true_ay=true_ay,
        imu_bias_ax=imu_bias_ax,
        imu_bias_ay=imu_bias_ay,
        random_seed=random_seed,
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
            "imu_bias_ax_mps2": imu_bias_ax,
            "imu_bias_ay_mps2": imu_bias_ay,
        }
    )

    return df


def extract_error_metrics(estimated_df):
    estimated_df, rmse_df = compute_position_errors(estimated_df)

    full_rmse = rmse_df.loc[
        rmse_df["method"] == "Kalman filter estimate",
        "position_rmse_m",
    ].iloc[0]

    dropout_df = estimated_df[
        (estimated_df["time_s"] >= GPS_DROPOUT_START_S)
        & (estimated_df["time_s"] <= GPS_DROPOUT_END_S)
    ]

    dropout_rmse = np.sqrt(
        np.mean(
            (dropout_df["estimated_x_m"] - dropout_df["true_x_m"]) ** 2
            + (dropout_df["estimated_y_m"] - dropout_df["true_y_m"]) ** 2
        )
    )

    estimated_df["kalman_position_error_m"] = np.sqrt(
        (estimated_df["estimated_x_m"] - estimated_df["true_x_m"]) ** 2
        + (estimated_df["estimated_y_m"] - estimated_df["true_y_m"]) ** 2
    )

    final_error = estimated_df["kalman_position_error_m"].iloc[-1]
    max_error = estimated_df["kalman_position_error_m"].max()

    return {
        "kalman_full_rmse_m": float(full_rmse),
        "kalman_dropout_rmse_m": float(dropout_rmse),
        "kalman_final_error_m": float(final_error),
        "kalman_max_error_m": float(max_error),
    }


def run_imu_bias_experiment():
    rows = []

    for imu_bias_level in IMU_BIAS_LEVELS_MPS2:
        print("=" * 70)
        print(f"IMU bias level: {imu_bias_level:.3f} m/s^2")

        repeated_metrics = []

        for repeat_idx in range(N_REPEATS):
            random_seed = RANDOM_SEED + repeat_idx

            df = build_dataset_for_bias(
                imu_bias_level=imu_bias_level,
                random_seed=random_seed,
            )

            estimated_df = run_kalman_filter(df)
            metrics = extract_error_metrics(estimated_df)
            repeated_metrics.append(metrics)

        row = {
            "imu_bias_level_mps2": imu_bias_level,
            "imu_bias_ax_mps2": imu_bias_level,
            "imu_bias_ay_mps2": -0.5 * imu_bias_level,
            "n_repeats": N_REPEATS,
        }

        for metric_name in repeated_metrics[0]:
            values = [m[metric_name] for m in repeated_metrics]
            row[f"{metric_name}_mean"] = float(np.mean(values))
            row[f"{metric_name}_std"] = float(np.std(values))

        rows.append(row)

        print(
            f"Full RMSE: {row['kalman_full_rmse_m_mean']:.3f} ± "
            f"{row['kalman_full_rmse_m_std']:.3f} m"
        )
        print(
            f"Dropout RMSE: {row['kalman_dropout_rmse_m_mean']:.3f} ± "
            f"{row['kalman_dropout_rmse_m_std']:.3f} m"
        )
        print(
            f"Final error: {row['kalman_final_error_m_mean']:.3f} ± "
            f"{row['kalman_final_error_m_std']:.3f} m"
        )

    return pd.DataFrame(rows)


def save_bias_rmse_plot(results_df):
    fig, ax = plt.subplots(figsize=(9, 5))

    ax.errorbar(
        results_df["imu_bias_level_mps2"],
        results_df["kalman_full_rmse_m_mean"],
        yerr=results_df["kalman_full_rmse_m_std"],
        marker="o",
        linewidth=2,
        capsize=3,
        label="Full trajectory RMSE",
    )

    ax.errorbar(
        results_df["imu_bias_level_mps2"],
        results_df["kalman_dropout_rmse_m_mean"],
        yerr=results_df["kalman_dropout_rmse_m_std"],
        marker="s",
        linewidth=2,
        capsize=3,
        label="GPS dropout interval RMSE",
    )

    ax.set_xlabel("Constant IMU acceleration bias magnitude (m/s²)")
    ax.set_ylabel("Position RMSE (m)")
    ax.set_title("Kalman Position Error Under IMU Acceleration Bias")
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()
    plt.savefig(OUTPUT_PLOT, dpi=150)
    plt.close()

    print(f"Saved IMU bias RMSE plot to: {OUTPUT_PLOT}")


def save_trajectory_comparison_plot():
    no_bias_df = build_dataset_for_bias(
        imu_bias_level=0.0,
        random_seed=RANDOM_SEED,
    )
    high_bias_df = build_dataset_for_bias(
        imu_bias_level=IMU_BIAS_LEVELS_MPS2[-1],
        random_seed=RANDOM_SEED,
    )

    no_bias_estimated = run_kalman_filter(no_bias_df)
    high_bias_estimated = run_kalman_filter(high_bias_df)

    fig, ax = plt.subplots(figsize=(8, 6))

    ax.plot(
        no_bias_df["true_x_m"],
        no_bias_df["true_y_m"],
        label="True trajectory",
        linewidth=2,
    )

    gps_df = no_bias_df[no_bias_df["gps_available"] == 1]
    ax.scatter(
        gps_df["gps_x_m"],
        gps_df["gps_y_m"],
        s=14,
        alpha=0.45,
        label="GPS-like measurements",
    )

    ax.plot(
        no_bias_estimated["estimated_x_m"],
        no_bias_estimated["estimated_y_m"],
        linewidth=2,
        label="Kalman estimate, no IMU bias",
    )

    ax.plot(
        high_bias_estimated["estimated_x_m"],
        high_bias_estimated["estimated_y_m"],
        linewidth=2,
        linestyle="--",
        label=f"Kalman estimate, {IMU_BIAS_LEVELS_MPS2[-1]:.2f} m/s² IMU bias",
    )

    dropout_df = no_bias_df[
        (no_bias_df["time_s"] >= GPS_DROPOUT_START_S)
        & (no_bias_df["time_s"] <= GPS_DROPOUT_END_S)
    ]

    ax.plot(
        dropout_df["true_x_m"],
        dropout_df["true_y_m"],
        linewidth=4,
        alpha=0.25,
        label="GPS dropout interval",
    )

    ax.set_xlabel("x position (m)")
    ax.set_ylabel("y position (m)")
    ax.set_title("Trajectory Estimate With and Without IMU Bias")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()
    plt.savefig(OUTPUT_TRAJECTORY_PLOT, dpi=150)
    plt.close()

    print(f"Saved trajectory comparison plot to: {OUTPUT_TRAJECTORY_PLOT}")


def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    results_df = run_imu_bias_experiment()
    results_df.to_csv(OUTPUT_CSV, index=False)
    print(f"Saved IMU bias results to: {OUTPUT_CSV}")

    save_bias_rmse_plot(results_df)
    save_trajectory_comparison_plot()


if __name__ == "__main__":
    main()
