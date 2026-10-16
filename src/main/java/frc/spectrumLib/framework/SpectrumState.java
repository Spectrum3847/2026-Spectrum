package frc.spectrumLib.framework;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * A named boolean state usable as a WPILib {@link Trigger}. Every instance built with the same name
 * reads and writes one shared value, and each raises an {@link Alert} in the States group.
 */
public class SpectrumState extends Trigger {

    /** Keyed by state name, so states that share a name share a value. */
    private static final HashMap<String, Boolean> stateConditions = new HashMap<>();

    private String name;
    private boolean value = false;
    private Alert alert;

    /**
     * Polled by the command scheduler's default event loop.
     *
     * @param name the state's name, used as its key and in its alert
     */
    public SpectrumState(String name) {
        super(pollCondition(name));
        this.name = name;
        alert = new Alert("States", name, AlertType.kInfo);
    }

    /** Polled by the given event loop instead of the scheduler's default one. */
    public SpectrumState(EventLoop eventLoop, String name) {
        super(eventLoop, pollCondition(name));
        this.name = name;
        alert = new Alert("States", name, AlertType.kInfo);
    }

    public void setState(boolean value) {
        this.value = value;
        alert.set(value);
        setCondition(name, value);
    }

    public Command setTrueWhileRunning() {
        return Commands.startEnd(() -> setState(true), () -> setState(false))
                .ignoringDisable(true)
                .withName(name + " state: TrueWhileRunning");
    }

    /**
     * @param time how long to hold the state true, in seconds
     */
    public Command setTrueForTime(DoubleSupplier time) {
        return Commands.runOnce(() -> setState(true))
                .alongWith(new WaitCommand(time.getAsDouble()))
                .andThen(() -> setState(false))
                .ignoringDisable(true)
                .withName(name + " state: SetTrueForTime->" + time.getAsDouble());
    }

    /**
     * @param time how long to hold the state false, in seconds
     */
    public Command setFalseForTime(DoubleSupplier time) {
        return Commands.runOnce(() -> setState(false))
                .alongWith(new WaitCommand(time.getAsDouble()))
                .andThen(() -> setState(true))
                .ignoringDisable(false)
                .withName(name + " state: SetFalseForTime->" + time.getAsDouble());
    }

    /**
     * @param time the longest the state stays true, in seconds
     * @param cancelCondition ends the wait early and drops the state to false
     */
    public Command setTrueForTimeWithCancel(DoubleSupplier time, Trigger cancelCondition) {
        return Commands.runOnce(() -> setState(true))
                .alongWith(new WaitCommand(time.getAsDouble()).onlyWhile(cancelCondition.negate()))
                .andThen(
                        () -> {
                            setState(false);
                        })
                .ignoringDisable(true)
                .withName(name + " state: SetTrueForTimeWithCancel->" + time.getAsDouble());
    }

    /**
     * Sets the state false, then true after a 5 ms wait, so a binding polled between them still
     * sees the false state.
     */
    public Command toggleToTrue() {
        return setFalse()
                .andThen(new WaitCommand(0.005), setTrue())
                .finallyDo(() -> setState(true))
                .ignoringDisable(true)
                .withName(name + " state: ToggleToTrue");
    }

    /**
     * Sets the state true, then false after a 5 ms wait, so a binding polled between them still
     * sees the true state.
     */
    public Command toggleToFalse() {
        return setTrue()
                .andThen(new WaitCommand(0.005), setFalse())
                .finallyDo(() -> setState(false))
                .ignoringDisable(true)
                .withName(name + " state: ToggleToFalse");
    }

    public Command set(boolean value) {
        return Commands.runOnce(() -> setState(value)).ignoringDisable(true);
    }

    public Command setTrue() {
        return set(true).withName(name + " state: SetTrue");
    }

    public Command setFalse() {
        return set(false).withName(name + " state: SetFalse");
    }

    public Command toggle() {
        return Commands.runOnce(() -> setState(!value))
                .ignoringDisable(true)
                .withName(name + " state: Toggle");
    }

    /** A supplier reading the shared value for name, registering the name as false if it is new. */
    private static BooleanSupplier pollCondition(String name) {
        stateConditions.putIfAbsent(name, false);

        return () -> stateConditions.get(name);
    }

    protected static void setCondition(String name, boolean value) {
        stateConditions.put(name, value);
    }
}
