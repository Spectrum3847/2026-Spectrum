# Phoenix Tuner X

Phoenix Tuner X is a powerful diagnostic and configuration tool provided by CTRE for their motor controllers and other devices.

## Motor Controls

When configuring motor controllers (e.g., Talon FX) using Phoenix Tuner X, consider the following:

*   **License**: Ensure the license is set to the current season under the team's CTRE account.
*   **Motor ID**: Each motor controller needs a unique CAN ID. Avoid duplicate IDs unless shared between a CANcoder and a motor ID (which requires careful management).
*   **Name**: Assign a name that matches the corresponding subsystem in the robot code for easy identification.
*   **Absolute Position/Rotation**: Monitor and reset these values as needed.
*   **Data**: View real-time data from the motor controller.

## Resetting Swerve Offsets

Correcting swerve wheel alignment is critical, especially during Systems Check.

### Swerve Aligners

*   Use physical swerve aligners on the robot with the bevel pointed toward the inside of the robot.

### CANcoders

*   Use the "Absolute Position No Offset" feature in Phoenix Tuner X to change the swerve values.
*   Check the swerve configuration value in the robot code and flip the sign of the value read from the CANcoders if necessary.
*   Deploy the updated code and recheck during Systems Check that the absolute position is approximately `0` (e.g., `✔ 0.0000024` is good, `✖ 0.3445553222` indicates an issue).

## Using Encoders

*   Ensure that the encoders are properly licensed for the current year using Phoenix Tuner X. This applies to devices like CANcoders or built-in encoders on motor controllers.

## Flashing SD Card

Phoenix Tuner X can also be used for flashing the SD card of devices like the `CANivore` or `Pigeon 2.0` with the latest firmware. This is essential for compatibility and optimal performance.
