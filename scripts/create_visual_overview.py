from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"

items = [
    (
        "True path vs noisy GPS",
        RESULTS / "true_vs_gps_measurements.png",
    ),
    (
        "Kalman estimated trajectory",
        RESULTS / "kalman_estimated_trajectory.png",
    ),
    (
        "Position error over time",
        RESULTS / "position_error_over_time.png",
    ),
    (
        "GPS dropout estimation",
        RESULTS / "gps_dropout_estimation.png",
    ),
]

fig, axes = plt.subplots(2, 2, figsize=(13, 9))
fig.suptitle("Sensor Fusion State Estimation - Visual Overview", fontsize=16)

for ax, (title, path) in zip(axes.flat, items):
    image = mpimg.imread(path)
    ax.imshow(image)
    ax.set_title(title, fontsize=11)
    ax.axis("off")

fig.tight_layout()
fig.savefig(RESULTS / "sensor_fusion_visual_overview.png", dpi=160)
plt.close(fig)

print("Saved results/sensor_fusion_visual_overview.png")
