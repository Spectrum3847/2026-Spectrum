package frc.spectrumLib.hardware;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** A {@link Servo} that is also a {@link Subsystem}, so commands can require it. */
public class SpectrumServo extends Servo implements Subsystem {

    /**
     * Connects to a PWM port on the RoboRIO.
     *
     * @param port PWM channel, 0 to 9
     */
    public SpectrumServo(int port) {
        super(port);
        CommandScheduler.getInstance().registerSubsystem(this);
    }
}
