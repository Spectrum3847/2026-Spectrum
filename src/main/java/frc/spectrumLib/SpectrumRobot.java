package frc.spectrumLib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.ArrayList;

public class SpectrumRobot extends TimedRobot {

    /** Create a single static instance of all of your subsystems */
    private static final ArrayList<SpectrumSubsystem> subsystems = new ArrayList<>();

    public static void add(SpectrumSubsystem subsystem) {
        subsystems.add(subsystem);
    }

    public SpectrumRobot() {
        super();
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    protected void setupDefaultCommands() {
        // Setup Default Commands for all subsystems
        subsystems.forEach(SpectrumSubsystem::setupDefaultCommand);
    }

    protected void setupStates() {
        // Bind Triggers for all subsystems
        subsystems.forEach(SpectrumSubsystem::setupStates);
    }
}
