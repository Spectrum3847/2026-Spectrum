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
 * A named boolean state exposed as a {@link Trigger}. Instances built with the name-only
 * constructor that share a name share one value, so any of them can set or read it. The {@link
 * SpectrumState(EventLoop, String)} constructor leaves the name and the alert unassigned, so a call
 * to setState on such an instance throws a NullPointerException before it reaches the shared value.
 */
public class SpectrumState extends Trigger {

    private static final HashMap<String, Boolean> stateConditions = new HashMap<>();

    private String name;
    private boolean value = false;
    private Alert alert;

    /** Creates a state polled on the default event loop, once per robot loop. */
    public SpectrumState(String name) {
        super(pollCondition(name));
        this.name = name;
        alert = new Alert("States", name, AlertType.kInfo);
    }

    /** Creates a state polled on the given event loop. */
    public SpectrumState(EventLoop eventLoop, String name) {
        super(eventLoop, pollCondition(name));
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
     * @param time seconds to hold the state true
     */
    public Command setTrueForTime(DoubleSupplier time) {
        return Commands.runOnce(() -> setState(true))
                .alongWith(new WaitCommand(time.getAsDouble()))
                .andThen(() -> setState(false))
                .ignoringDisable(true)
                .withName(name + " state: SetTrueForTime->" + time.getAsDouble());
    }

    /**
     * @param time seconds to hold the state false
     */
    public Command setFalseForTime(DoubleSupplier time) {
        return Commands.runOnce(() -> setState(false))
                .alongWith(new WaitCommand(time.getAsDouble()))
                .andThen(() -> setState(true))
                .ignoringDisable(false)
                .withName(name + " state: SetFalseForTime->" + time.getAsDouble());
    }

    /**
     * @param time seconds before the state returns to false on its own
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

    /** Toggles through {@code false} for 5 ms so triggers bound to that transition fire. */
    public Command toggleToTrue() {
        return setFalse()
                .andThen(new WaitCommand(0.005), setTrue())
                .finallyDo(() -> setState(true))
                .ignoringDisable(true)
                .withName(name + " state: ToggleToTrue");
    }

    /** Toggles through {@code true} for 5 ms so triggers bound to that transition fire. */
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
        return Commands.runOnce(
                        () -> {
                            value = !value;
                            alert.set(value);
                            setCondition(name, value);
                        })
                .ignoringDisable(true)
                .withName(name + " state: Toggle");
    }

    private static BooleanSupplier pollCondition(String name) {
        if (!stateConditions.containsKey(name)) {
            stateConditions.put(name, false);
        }

        return () -> stateConditions.get(name);
    }

    protected static void setCondition(String name, boolean value) {
        stateConditions.put(name, value);
    }
}
