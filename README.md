# Sensor Fusion State Estimation

Aerospace- and robotics-inspired state estimation project for autonomous motion tracking using simulated sensor data and Kalman filtering.

This project focuses on:

- 2D dynamic motion simulation
- GPS-like noisy position measurements
- IMU-like acceleration measurements
- GPS dropout simulation
- sensor fusion and state estimation
- trajectory and error visualization

---

## Current Stage

The current version generates a simulated 2D trajectory and synthetic sensor measurements.

Current workflow:

```text
simulated 2D trajectory
→ noisy GPS-like measurements
→ noisy IMU-like acceleration
→ GPS dropout interval
→ generated dataset and initial plots
```

---

## Current Outputs

### True Trajectory and GPS-like Measurements

![True vs GPS Measurements](results/true_vs_gps_measurements.png)

### True vs IMU-like Acceleration

![True vs IMU Acceleration](results/true_vs_imu_acceleration.png)

---

## Repository Structure

```text
sensor-fusion-state-estimation/
├── data/
│   └── simulated/
│       └── simulated_sensor_data.csv
├── docs/
├── results/
│   ├── true_vs_gps_measurements.png
│   └── true_vs_imu_acceleration.png
├── src/
│   └── simulate_motion.py
├── .gitignore
├── README.md
└── requirements.txt
```

---

## How to Run

Create and activate a virtual environment:

```bash
python3 -m venv venv
source venv/bin/activate
```

Install dependencies:

```bash
pip install -r requirements.txt
```

If internet access is limited, dependencies can also be installed from a local wheelhouse:

```bash
pip install --no-index --find-links=/home/amir/python-wheels -r requirements.txt
```

Generate simulated trajectory and sensor data:

```bash
python src/simulate_motion.py
```

---

## Generated Data

The simulation script creates:

```text
data/simulated/simulated_sensor_data.csv
```

The dataset includes:

- true 2D position
- true 2D velocity
- true 2D acceleration
- noisy IMU-like acceleration
- noisy GPS-like position measurements
- GPS availability flag

Main columns:

```text
time_s
true_x_m
true_y_m
true_vx_mps
true_vy_mps
true_ax_mps2
true_ay_mps2
imu_ax_mps2
imu_ay_mps2
gps_x_m
gps_y_m
gps_available
```

---

## Project Motivation

State estimation is a core part of many autonomous and intelligent physical systems.

In real-world systems, sensor measurements are often noisy, incomplete, or temporarily unavailable. A state estimator can combine a motion model with sensor measurements to produce a more reliable estimate of the system state.

This project is designed as a portfolio project connecting:

- dynamic systems
- sensor data
- robotics-inspired motion tracking
- aerospace-inspired trajectory estimation
- Kalman filtering
- intelligent physical systems

---

## Next Steps

Planned next steps:

- implement a Kalman Filter for 2D state estimation
- compare raw GPS-like measurements with filtered estimates
- compute RMSE for position estimation
- evaluate performance during GPS dropout
- add trajectory estimation and error plots
- document the mathematical model used for prediction and correction

---

## Portfolio Context

This project supports a broader portfolio direction focused on:

```text
AI / ML for Intelligent Physical Systems
```

It connects computer engineering skills with physical-system modeling, sensing, state estimation, and autonomous-system concepts.

The long-term goal is to build a clean and practical project that can be discussed in the context of:

- control systems
- robotics
- autonomous systems
- cyber-physical systems
- embedded sensing
- aerospace-inspired monitoring and navigation
