# Sensor Fusion Results Interpretation

This document explains the main results of the sensor fusion experiment and how they relate to robotics, navigation, and intelligent physical systems.

## Purpose of the Experiment

The goal of this project is to estimate the 2D position and velocity of a moving platform by combining:

- a motion model
- noisy GPS-like position measurements
- noisy IMU-like acceleration measurements
- a Kalman Filter prediction and correction structure

The project is simulation-based, which makes it possible to compare the estimated trajectory against the known true trajectory.

## Why Raw GPS-like Measurements Are Not Enough

The GPS-like sensor provides direct position measurements, but these measurements are noisy and are not available at every simulation step.

Raw GPS-like measurements can jump around the true trajectory because each measurement contains random error. This makes the raw position data less smooth and less reliable for control, navigation, or tracking.

In a real robotic or autonomous system, relying only on noisy position measurements can lead to unstable or inaccurate decisions.

## Why the Kalman Filter Improves the Estimate

The Kalman Filter improves the trajectory estimate because it does not rely only on the latest GPS-like measurement.

Instead, it combines two sources of information:

1. Prediction from the motion model and IMU-like acceleration input
2. Correction from GPS-like position measurements when they are available

This allows the filter to produce a smoother and more accurate position estimate than raw GPS-like measurements.

In the current experiment, the Kalman Filter reduces the position RMSE compared with raw GPS-like measurements.

## Interpretation of RMSE Results

The current RMSE results are:

- Raw GPS-like measurements: 4.51 m
- Kalman filter estimate over full trajectory: 2.61 m
- Kalman estimate at GPS timestamps: 1.82 m

The raw GPS-like RMSE is computed only when GPS-like measurements are available.

The full Kalman trajectory RMSE is computed over the entire trajectory, including the GPS dropout interval. This makes the full-trajectory Kalman RMSE harder to optimize, because the filter must keep estimating even when GPS-like measurements are missing.

The Kalman estimate at GPS timestamps gives a more direct comparison with raw GPS-like measurements. In this case, the Kalman Filter gives a lower RMSE, showing that the fusion approach improves position estimation when GPS updates are available.

## GPS Dropout Behavior

During the GPS dropout interval, the filter cannot perform the measurement correction step.

In this period, the Kalman Filter relies on:

- the motion model
- IMU-like acceleration input
- previous state estimate

Because there is no GPS-like correction during dropout, the estimation error can increase over time.

After GPS-like measurements become available again, the filter corrects the trajectory estimate and reduces the accumulated error.

This behavior is important in robotics and navigation because real systems can temporarily lose reliable position measurements.

## Noise-Level Comparison

The noise-level comparison experiment tests how the system behaves as GPS-like measurement noise increases.

The results show that the Kalman Filter consistently reduces position RMSE compared with raw GPS-like measurements across different GPS noise levels.

At higher GPS noise levels, the raw measurements become much less reliable. The Kalman Filter is still affected by the noise, but it benefits from the motion model and IMU-like acceleration input.

This explains why the performance gap between raw GPS-like measurements and Kalman estimates becomes larger when GPS noise increases.

## Relevance to Robotics and Navigation

This project represents a simplified version of a common problem in robotics, autonomous systems, and aerospace-inspired navigation.

A real autonomous system often needs to estimate its state using imperfect sensors. For example:

- GPS may be noisy or unavailable
- IMU measurements may contain noise and bias
- sensor update rates may be different
- the system still needs a continuous estimate of position and velocity

The Kalman Filter provides a structured way to combine prediction and measurement correction.

This project therefore acts as a foundation for later work in:

- robot localization
- autonomous navigation
- sensor fusion
- state estimation
- control of dynamic systems

## Limitations

This project is intentionally simplified.

Current limitations include:

- the trajectory is simulated
- the sensor noise is modeled with simple random noise
- IMU bias and drift are not fully modeled
- the motion model is simpler than a real vehicle model
- GPS dropout is scripted instead of environment-dependent
- the filter is tested in 2D rather than full 3D motion

These limitations are acceptable for this project because the goal is to demonstrate the core idea of sensor fusion and Kalman filtering in a controlled setting.

## Possible Future Improvements

Possible future improvements include:

- adding IMU bias and drift
- testing longer GPS dropout intervals
- comparing different process noise settings
- adding a constant-acceleration model
- extending the simulation to 3D motion
- testing the filter on a real logged sensor dataset

However, the current version already demonstrates the main engineering idea: combining noisy measurements with a motion model to estimate the state of a moving platform.
