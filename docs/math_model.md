# Mathematical Model

## Overview

This document explains the motion and sensor model used in the 2D Kalman Filter state estimation project.

The goal is to estimate the position and velocity of a moving platform using:

- a motion model
- noisy GPS-like position measurements
- noisy IMU-like acceleration measurements

The system is simulation-based, which means the true trajectory is known and can be used for RMSE evaluation.

---

## State Vector

The Kalman Filter estimates a 2D position and velocity state:

```text
x = [px, py, vx, vy]^T
```

where:

| Symbol | Meaning |
|---|---|
| `px` | x-position in meters |
| `py` | y-position in meters |
| `vx` | x-velocity in meters per second |
| `vy` | y-velocity in meters per second |

---

## Control Input

The IMU-like acceleration measurements are used as control inputs:

```text
u = [ax, ay]^T
```

where:

| Symbol | Meaning |
|---|---|
| `ax` | x-axis acceleration in m/s² |
| `ay` | y-axis acceleration in m/s² |

In the simulation, the IMU-like acceleration is noisy:

```text
imu_ax = true_ax + noise
imu_ay = true_ay + noise
```

---

## Motion Model

The project uses a constant-velocity model with acceleration input.

For a timestep `dt`, the motion equations are:

```text
px_k = px_{k-1} + vx_{k-1} dt + 0.5 ax dt²
py_k = py_{k-1} + vy_{k-1} dt + 0.5 ay dt²
vx_k = vx_{k-1} + ax dt
vy_k = vy_{k-1} + ay dt
```

This can be written in matrix form as:

```text
x_k = F x_{k-1} + B u_k
```

where:

```text
F =
[1  0  dt  0 ]
[0  1  0   dt]
[0  0  1   0 ]
[0  0  0   1 ]
```

and:

```text
B =
[0.5 dt²   0      ]
[0         0.5 dt²]
[dt        0      ]
[0         dt     ]
```

---

## Measurement Model

The GPS-like sensor measures only position:

```text
z = [gps_x, gps_y]^T
```

The measurement model is:

```text
z_k = H x_k
```

where:

```text
H =
[1  0  0  0]
[0  1  0  0]
```

This means the GPS-like sensor observes position, but not velocity.

---

## Prediction Step

At every timestep, the filter predicts the next state using the motion model and IMU-like acceleration input:

```text
x_pred = F x + B u
P_pred = F P F^T + Q
```

where:

| Symbol | Meaning |
|---|---|
| `x_pred` | predicted state |
| `P_pred` | predicted covariance |
| `Q` | process noise covariance |

The prediction step is always performed, even when GPS-like measurements are unavailable.

---

## Correction Step

When a GPS-like measurement is available, the filter corrects the predicted state.

Innovation:

```text
y = z - H x_pred
```

Innovation covariance:

```text
S = H P_pred H^T + R
```

Kalman gain:

```text
K = P_pred H^T S^{-1}
```

Updated state:

```text
x = x_pred + K y
```

Updated covariance:

```text
P = (I - K H) P_pred
```

where:

| Symbol | Meaning |
|---|---|
| `R` | GPS-like measurement noise covariance |
| `K` | Kalman gain |
| `I` | identity matrix |

---

## GPS Dropout Behavior

During the GPS dropout interval, no GPS-like position measurements are available.

In that case, the filter skips the correction step and only performs prediction:

```text
prediction only → no GPS correction
```

This causes estimation error to increase during dropout because the filter relies only on:

- the motion model
- noisy IMU-like acceleration input

When GPS-like measurements return, the correction step reduces the accumulated error.

This behavior is visible in:

```text
results/position_error_over_time.png
results/gps_dropout_estimation.png
```

---

## Noise-Level Experiment

The project also evaluates several GPS-like measurement noise levels:

```text
1 m, 2 m, 3 m, 5 m, 8 m, 12 m
```

For each noise level, the simulation and Kalman Filter are repeated multiple times.

The results show that the Kalman Filter consistently reduces position RMSE compared with raw GPS-like measurements.

Main output:

```text
results/noise_level_rmse_comparison.png
```

---

## Interpretation

The Kalman Filter improves estimation because it combines two sources of information:

1. The motion model predicts how the platform should move.
2. GPS-like measurements correct the predicted position when available.

This is useful for autonomous and intelligent physical systems because real sensors are often noisy, delayed, or temporarily unavailable.

---

## Limitations

The current model is intentionally simple.

Limitations include:

- the trajectory is simulated
- GPS and IMU measurements are synthetic
- IMU bias and drift are not yet modeled
- orientation is not estimated
- the filter is linear
- real hardware data is not used in this project

These limitations make the project easier to understand, while leaving room for future extensions such as IMU bias modeling, nonlinear motion, Extended Kalman Filtering, and real embedded IMU experiments.

---

## Summary

This project uses a linear Kalman Filter to estimate 2D position and velocity from noisy GPS-like and IMU-like measurements.

The mathematical model connects dynamic systems, sensor fusion, state estimation, and autonomous-system concepts in a clean simulation-based setting.
