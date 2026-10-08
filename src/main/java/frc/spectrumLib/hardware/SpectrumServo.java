package frc.spectrumLib.hardware;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A WPILib {@link Servo} that registers itself with the {@link CommandScheduler}, so it takes part
 * in command requirement checking.
 */
public class SpectrumServo extends Servo implements Subsystem {

    /**
     * @param port PWM channel [0, 9] the servo signal wire is plugged into
     */
    public SpectrumServo(int port) {
        super(port);
        CommandScheduler.getInstance().registerSubsystem(this);
    }
}
