# PID Tuning

## What is PID Tuning?

PID (Proportional-Integral-Derivative) is a control loop mechanism widely used in industrial control systems and robotics. It is a fundamental concept for controlling motor systems, ensuring they reach and maintain a desired setpoint.

*   **P (Proportional)**: Responds to the current error (difference between setpoint and actual value). A larger proportional gain (`kP`) means a stronger response to error.
*   **I (Integral)**: Responds to the accumulation of past errors. It helps eliminate steady-state errors (errors that persist over time). A larger integral gain (`kI`) helps correct for small, persistent errors.
*   **D (Derivative)**: Responds to the rate of change of the error. It helps dampen oscillations and reduce overshoot. A larger derivative gain (`kD`) makes the system react more strongly to rapid changes in error.

## Resources

*   **CTRE Documentation**: Provides specific guidance for tuning PID loops on CTRE motor controllers (e.g., Talon FX, Spark SRX).
*   **WPILib Documentation**: Offers general information and best practices for implementing and tuning PID controllers in FRC.

## Closed-Loop System

PID controllers are part of a closed-loop system that drives most of our motor systems.

*   **Setpoint**: The desired target value (e.g., a specific motor position, velocity, or a target angle).
*   **Process Variable**: The actual measured value from the sensor (e.g., encoder reading, gyroscope angle).
*   **Error**: The difference between the setpoint and the process variable.
*   **Output**: The PID controller calculates an output (e.g., motor power, voltage) based on the current, past, and predicted future error.

## Tuning Process

The goal of PID tuning is to find the optimal `kP`, `kI`, and `kD` gains that allow the system to:

*   **Reach the Setpoint Quickly**: Minimize rise time.
*   **Avoid Overshoot**: Prevent the system from going past the setpoint.
*   **Minimize Oscillation**: Prevent the system from wobbling around the setpoint.
*   **Reduce Steady-State Error**: Ensure the system settles precisely at the setpoint.

Tuning often involves an iterative process, starting with `kP`, then `kD`, and finally `kI` to refine performance.
