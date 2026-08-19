package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.spectrumLib.gamepads.Gamepad;
import frc.spectrumLib.telemetry.Telemetry;

/*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */
public class Operator extends Gamepad {
    public final Trigger LB = leftBumper;
    public final Trigger RB = rightBumper;
    public final Trigger LT = leftTrigger;
    public final Trigger RT = rightTrigger;

    public final Trigger AButton = A;
    public final Trigger BButton = B;
    public final Trigger XButton = X;
    public final Trigger YButton = Y;

    public final Trigger startButton = start;
    public final Trigger selectButton = select;

    /* Coast/brake are pit controls, only live while disabled */
    public final Trigger coastA = AButton.and(disabled);

    public final Trigger leftStickPress = leftStickClick;
    public final Trigger rightStickPress = rightStickClick;

    public final Trigger dPadUp = upDpad;
    public final Trigger dPadDown = downDpad;
    public final Trigger dPadLeft = leftDpad;
    public final Trigger dPadRight = rightDpad;

    public static class OperatorConfig extends Config {
        /** Creates a new OperatorConfig instance. */
        public OperatorConfig() {
            super("Operator", 1);
            setTriggersDeadzone(0.0);
        }
    }

    @SuppressWarnings("unused")
    private OperatorConfig config;
    /**
     * Creates a new Operator instance.
     *
     * @param config the config
     */
    public Operator(OperatorConfig config) {
        super(config);
        this.config = config;

        Telemetry.print("Operator Subsystem Initialized: ");
    }
}
