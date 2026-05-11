from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from kalman_filter import (
    compute_position_errors,
    run_kalman_filter,
)
from simulate_motion import (
    DT,
    DURATION,
    GPS_DROPOUT_END_S,
    GPS_DROPOUT_START_S,
    GPS_MEASUREMENT_INTERVAL,
    IMU_ACCEL_NOISE_STD,
    RANDOM_SEED,
    simulate_true_motion,
)


RESULTS_DIR = Path("results")
OUTPUT_CSV = RESULTS_DIR / "noise_level_comparison.csv"
OUTPUT_PLOT = RESULTS_DIR / "noise_level_rmse_comparison.png"

GPS_NOISE_LEVELS_M = [1.0, 2.0, 3.0, 5.0, 8.0, 12.0]
N_REPEATS = 5


def generate_sensor_measurements_with_gps_noise(
    times,
    true_x,
    true_y,
    true_ax,
    true_ay,
    gps_noise_std_m,
    random_seed,
):
    rng = np.random.default_rng(random_seed)

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
            gps_x[k] = true_x[k] + rng.normal(0.0, gps_noise_std_m)
            gps_y[k] = true_y[k] + rng.normal(0.0, gps_noise_std_m)
            gps_available[k] = 1

    return imu_ax, imu_ay, gps_x, gps_y, gps_available


def build_dataset_for_noise_level(gps_noise_std_m, random_seed):
    (
        times,
        true_x,
        true_y,
        true_vx,
        true_vy,
        true_ax,
        true_ay,
    ) = simulate_true_motion()

    imu_ax, imu_ay, gps_x, gps_y, gps_available = (
        generate_sensor_measurements_with_gps_noise(
            times=times,
            true_x=true_x,
            true_y=true_y,
            true_ax=true_ax,
            true_ay=true_ay,
            gps_noise_std_m=gps_noise_std_m,
            random_seed=random_seed,
        )
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


def extract_rmse_values(rmse_df):
    gps_rmse = rmse_df.loc[
        rmse_df["method"] == "Raw GPS-like measurements",
        "position_rmse_m",
    ].iloc[0]

    kalman_full_rmse = rmse_df.loc[
        rmse_df["method"] == "Kalman filter estimate",
        "position_rmse_m",
    ].iloc[0]

    kalman_gps_time_rmse = rmse_df.loc[
        rmse_df["method"] == "Kalman estimate at GPS timestamps",
        "position_rmse_m",
    ].iloc[0]

    return gps_rmse, kalman_full_rmse, kalman_gps_time_rmse


def run_noise_level_experiment():
    rows = []

    for gps_noise_std_m in GPS_NOISE_LEVELS_M:
        print("=" * 70)
        print(f"GPS noise std: {gps_noise_std_m:.1f} m")

        repeat_gps_rmse = []
        repeat_kalman_full_rmse = []
        repeat_kalman_gps_time_rmse = []

        for repeat_idx in range(N_REPEATS):
            random_seed = RANDOM_SEED + repeat_idx

            df = build_dataset_for_noise_level(
                gps_noise_std_m=gps_noise_std_m,
                random_seed=random_seed,
            )

            estimated_df = run_kalman_filter(df)
            estimated_df, rmse_df = compute_position_errors(estimated_df)

            gps_rmse, kalman_full_rmse, kalman_gps_time_rmse = extract_rmse_values(
                rmse_df
            )

            repeat_gps_rmse.append(gps_rmse)
            repeat_kalman_full_rmse.append(kalman_full_rmse)
            repeat_kalman_gps_time_rmse.append(kalman_gps_time_rmse)

        row = {
            "gps_noise_std_m": gps_noise_std_m,
            "raw_gps_rmse_mean_m": float(np.mean(repeat_gps_rmse)),
            "raw_gps_rmse_std_m": float(np.std(repeat_gps_rmse)),
            "kalman_full_rmse_mean_m": float(np.mean(repeat_kalman_full_rmse)),
            "kalman_full_rmse_std_m": float(np.std(repeat_kalman_full_rmse)),
            "kalman_gps_time_rmse_mean_m": float(
                np.mean(repeat_kalman_gps_time_rmse)
            ),
            "kalman_gps_time_rmse_std_m": float(np.std(repeat_kalman_gps_time_rmse)),
            "n_repeats": N_REPEATS,
        }

        rows.append(row)

        print(
            f"Raw GPS RMSE: {row['raw_gps_rmse_mean_m']:.3f} ± "
            f"{row['raw_gps_rmse_std_m']:.3f} m"
        )
        print(
            f"Kalman full RMSE: {row['kalman_full_rmse_mean_m']:.3f} ± "
            f"{row['kalman_full_rmse_std_m']:.3f} m"
        )
        print(
            f"Kalman at GPS timestamps RMSE: "
            f"{row['kalman_gps_time_rmse_mean_m']:.3f} ± "
            f"{row['kalman_gps_time_rmse_std_m']:.3f} m"
        )

    results_df = pd.DataFrame(rows)
    return results_df


def save_noise_level_plot(results_df):
    fig, ax = plt.subplots(figsize=(9, 5))

    ax.errorbar(
        results_df["gps_noise_std_m"],
        results_df["raw_gps_rmse_mean_m"],
        yerr=results_df["raw_gps_rmse_std_m"],
        marker="o",
        linewidth=2,
        capsize=3,
        label="Raw GPS-like measurements",
    )

    ax.errorbar(
        results_df["gps_noise_std_m"],
        results_df["kalman_full_rmse_mean_m"],
        yerr=results_df["kalman_full_rmse_std_m"],
        marker="s",
        linewidth=2,
        capsize=3,
        label="Kalman estimate, full trajectory",
    )

    ax.errorbar(
        results_df["gps_noise_std_m"],
        results_df["kalman_gps_time_rmse_mean_m"],
        yerr=results_df["kalman_gps_time_rmse_std_m"],
        marker="^",
        linewidth=2,
        capsize=3,
        label="Kalman estimate, GPS timestamps",
    )

    ax.set_xlabel("GPS-like measurement noise std (m)")
    ax.set_ylabel("Position RMSE (m)")
    ax.set_title("Effect of GPS Noise on Position Estimation")
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()
    plt.savefig(OUTPUT_PLOT, dpi=150)
    plt.close()

    print(f"Saved noise-level comparison plot to: {OUTPUT_PLOT}")


def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    results_df = run_noise_level_experiment()
    results_df.to_csv(OUTPUT_CSV, index=False)

    print()
    print(f"Saved noise-level comparison results to: {OUTPUT_CSV}")
    print()
    print(results_df.to_string(index=False))

    save_noise_level_plot(results_df)


if __name__ == "__main__":
    main()
