from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

DATA_DIR = ROOT / "data" / "simulated"
RESULTS_DIR = ROOT / "results"

ESTIMATED_CSV = DATA_DIR / "kalman_estimates.csv"
SIMULATED_CSV = DATA_DIR / "simulated_sensor_data.csv"
OUTPUT_GIF = RESULTS_DIR / "trajectory_tracking_animation.gif"


def load_or_build_estimates():
    if ESTIMATED_CSV.exists():
        return pd.read_csv(ESTIMATED_CSV)

    from src.simulate_motion import build_dataset
    from src.kalman_filter import run_kalman_filter, compute_position_errors

    DATA_DIR.mkdir(parents=True, exist_ok=True)

    if SIMULATED_CSV.exists():
        sensor_df = pd.read_csv(SIMULATED_CSV)
    else:
        sensor_df = build_dataset()
        sensor_df.to_csv(SIMULATED_CSV, index=False)

    estimated_df = run_kalman_filter(sensor_df)
    estimated_df, _ = compute_position_errors(estimated_df)
    estimated_df.to_csv(ESTIMATED_CSV, index=False)

    return estimated_df


def make_animation(df):
    RESULTS_DIR.mkdir(exist_ok=True)

    frame_step = 4
    frame_indexes = list(range(0, len(df), frame_step))

    x_min = min(df["true_x_m"].min(), df["estimated_x_m"].min(), df["gps_x_m"].min(skipna=True)) - 8
    x_max = max(df["true_x_m"].max(), df["estimated_x_m"].max(), df["gps_x_m"].max(skipna=True)) + 8
    y_min = min(df["true_y_m"].min(), df["estimated_y_m"].min(), df["gps_y_m"].min(skipna=True)) - 8
    y_max = max(df["true_y_m"].max(), df["estimated_y_m"].max(), df["gps_y_m"].max(skipna=True)) + 8

    fig, ax = plt.subplots(figsize=(8, 7))
    fig.suptitle("Kalman Filter Sensor Fusion - Live Trajectory Tracking", fontsize=14)

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect("equal")
    ax.set_xlabel("x position (m)")
    ax.set_ylabel("y position (m)")
    ax.grid(True, alpha=0.3)

    true_line, = ax.plot([], [], linewidth=2, label="true trajectory")
    estimate_line, = ax.plot([], [], linewidth=2, linestyle="--", label="Kalman estimate")
    gps_points = ax.scatter([], [], s=18, alpha=0.55, label="GPS measurements")

    true_dot, = ax.plot([], [], marker="o", markersize=8)
    estimate_dot, = ax.plot([], [], marker="o", markersize=8)
    gps_current = ax.scatter([], [], s=70, marker="x", label="current GPS")

    dropout_mask = (df["time_s"] >= 25.0) & (df["time_s"] <= 35.0)
    ax.plot(
        df.loc[dropout_mask, "true_x_m"],
        df.loc[dropout_mask, "true_y_m"],
        linewidth=5,
        alpha=0.20,
        label="GPS dropout interval",
    )

    info_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        fontsize=9,
        family="monospace",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85),
    )

    ax.legend(loc="lower right")

    def init():
        true_line.set_data([], [])
        estimate_line.set_data([], [])
        gps_points.set_offsets(np.empty((0, 2)))
        true_dot.set_data([], [])
        estimate_dot.set_data([], [])
        gps_current.set_offsets(np.empty((0, 2)))
        info_text.set_text("")
        return true_line, estimate_line, gps_points, true_dot, estimate_dot, gps_current, info_text

    def update(frame_index):
        current = df.iloc[: frame_index + 1]
        row = df.iloc[frame_index]

        gps_seen = current[current["gps_available"] == 1]

        true_line.set_data(current["true_x_m"], current["true_y_m"])
        estimate_line.set_data(current["estimated_x_m"], current["estimated_y_m"])

        if len(gps_seen) > 0:
            gps_points.set_offsets(gps_seen[["gps_x_m", "gps_y_m"]].to_numpy())
        else:
            gps_points.set_offsets(np.empty((0, 2)))

        true_dot.set_data([row["true_x_m"]], [row["true_y_m"]])
        estimate_dot.set_data([row["estimated_x_m"]], [row["estimated_y_m"]])

        if row["gps_available"] == 1:
            gps_current.set_offsets([[row["gps_x_m"], row["gps_y_m"]]])
            gps_status = "GPS update"
        else:
            gps_current.set_offsets(np.empty((0, 2)))
            if 25.0 <= row["time_s"] <= 35.0:
                gps_status = "GPS dropout"
            else:
                gps_status = "prediction only"

        error = np.sqrt(
            (row["estimated_x_m"] - row["true_x_m"]) ** 2
            + (row["estimated_y_m"] - row["true_y_m"]) ** 2
        )

        info_text.set_text(
            f"time: {row['time_s']:.1f} s\n"
            f"mode: {gps_status}\n"
            f"tracking error: {error:.2f} m\n"
            f"true x,y: {row['true_x_m']:.1f}, {row['true_y_m']:.1f}\n"
            f"est. x,y: {row['estimated_x_m']:.1f}, {row['estimated_y_m']:.1f}"
        )

        return true_line, estimate_line, gps_points, true_dot, estimate_dot, gps_current, info_text

    animation = FuncAnimation(
        fig,
        update,
        frames=frame_indexes,
        init_func=init,
        interval=45,
        blit=True,
    )

    fig.tight_layout()
    animation.save(OUTPUT_GIF, writer=PillowWriter(fps=22))
    plt.close(fig)


def main():
    df = load_or_build_estimates()
    make_animation(df)

    print("Saved:")
    print(OUTPUT_GIF)


if __name__ == "__main__":
    main()
