package frc.robot.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Climb extends SubsystemBase {
    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;

    public Climb() {
        /*
         * Initialize the SPARK MAX and get its encoder and closed loop controller
         * objects for later use.
         */
        motor = new SparkMax(45, MotorType.kBrushless);
        closedLoopController = motor.getClosedLoopController();
        encoder = motor.getEncoder();

        /*
         * Create a new SPARK MAX configuration object. This will store the
         * configuration parameters for the SPARK MAX that we will set below.
         */
        motorConfig = new SparkMaxConfig();

        /*
         * Configure the encoder. For this specific example, we are using the
         * integrated encoder of the NEO, and we don't need to configure it. If
         * needed, we can adjust values like the position or velocity conversion
         * factors.
         */
        motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

        /*
         * Configure the closed loop controller. We want to make sure we set the
         * feedback sensor as the primary encoder.
         */
        motorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control. We don't need to pass a closed loop
                // slot, as it will default to slot 0.
                .p(0.1)
                .i(0)
                .d(0)
                .outputRange(-1, 1)
                // Set PID values for velocity control in slot 1
                .p(5, ClosedLoopSlot.kSlot1)
                .i(0, ClosedLoopSlot.kSlot1)
                .d(0, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
                .feedForward
                // kV is now in Volts, so we multiply by the nominal voltage (12V)
                .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);

        /*
         * Apply the configuration to the SPARK MAX.
         *
         * kResetSafeParameters is used to get the SPARK MAX to a known state. This
         * is useful in case the SPARK MAX is replaced.
         *
         * kPersistParameters is used to ensure the configuration is not lost when
         * the SPARK MAX loses power. This is useful for power cycles that may occur
         * mid-operation.
         */
        motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command runDutyCycleOut(DoubleSupplier dutyCycleSupplier) {
        return run(() -> {
                    double targetDutyCycle = dutyCycleSupplier.getAsDouble();
                    closedLoopController.setSetpoint(
                            targetDutyCycle, ControlType.kDutyCycle, ClosedLoopSlot.kSlot1);
                })
                .withName("FuelIntake.runDutyCycleOut");
    }

    public Command stopMotor() {
        return run(() -> motor.stopMotor());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb/Applied Output", motor.getAppliedOutput());
        SmartDashboard.putNumber("Climb/Output Current", motor.getOutputCurrent());
        SmartDashboard.putNumber("Climb/Bus Voltage", motor.getBusVoltage());
        SmartDashboard.putNumber("Climb/Temperature", motor.getMotorTemperature());
        SmartDashboard.putString(
                "Climb/Controller Output Type", closedLoopController.getControlType().toString());
        SmartDashboard.putNumber("Climb/Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Climb/Position", encoder.getPosition());
    }
}
