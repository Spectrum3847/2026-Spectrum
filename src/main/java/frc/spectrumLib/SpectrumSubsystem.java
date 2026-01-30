package frc.spectrumLib;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The base interface for all Spectrum subsystems.
 * Extends WPILib's Subsystem and adds common setup methods.
 */
public interface SpectrumSubsystem extends Subsystem {

    /**
     * Set up the states and triggers for this subsystem.
     * This is typically used to bind commands to SpectrumState triggers.
     */
    void setupStates();

    /**
     * Set up the default command for this subsystem.
     * This command will run when no other command is using this subsystem.
     */
    void setupDefaultCommand();
}
