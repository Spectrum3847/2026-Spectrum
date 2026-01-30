package frc.spectrumLib.leds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.spectrumLib.SpectrumRobot;
import frc.spectrumLib.SpectrumSubsystem;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

public class SpectrumLEDs2 extends SubsystemBase implements SpectrumSubsystem {

    public static class Config {
        @Getter
        private String name;
        @Getter
        @Setter
        private boolean attached = true;
        @Getter
        @Setter
        private int length;
        @Getter
        @Setter
        private int port = 0;
        @Getter
        @Setter
        private int startingIndex = 0;
        @Getter
        @Setter
        private double ledSpacing = 1.0 / 60.0;
        @Getter
        @Setter
        private CANdle leds = null;

        public Config(String name, int length, int port) {
            this.name = name;
            this.length = length;
            this.port = port;
        }

        public Config(String name, int length, CANdle leds, int startingIndex) {
            this.name = name;
            this.length = length;
            this.leds = leds;
            this.startingIndex = startingIndex;
        }
    }

    @Getter
    protected CANdle leds;
    protected LEDConfigs configs;
    protected boolean mainView = false;
    protected int ledsID = 0;

    @Getter
    private Config config;
    @Getter
    private int length;

    // Non-final to allow initialization
    protected StrobeAnimation defaultPattern;

    @Getter
    protected Command defaultCommand;

    public final Trigger defaultTrigger;

    @Getter
    @Setter
    private int commandPriority = 0;

    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
        for (int i = 0; i < maxAttempts; i++) {
            var error = command.get();
            if (error.isOK())
                break;
        }
    }

    public SpectrumLEDs2(Config config) {
        this.config = config;
        this.length = config.length;

        if (config.leds == null) {
            this.ledsID = config.port;
            leds = new CANdle(ledsID);
            configs = new LEDConfigs();
            configs.StripType = StripTypeValue.RGB;
            tryUntilOk(5, () -> leds.getConfigurator().apply(configs));
            mainView = true;
        } else {
            leds = config.leds;
        }

        // Initialize default pattern
        defaultPattern = new StrobeAnimation(0, config.length)
                .withColor(new RGBWColor(Color.kGreen));

        defaultCommand = setPattern(defaultPattern, -1).withName("LEDs.defaultCommand");
        defaultTrigger = new Trigger(() -> defaultCommand.isScheduled());

        SpectrumRobot.add(this);
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void setLEDs(ControlRequest request) {
        if (config.attached) {
            leds.setControl(request);
        }
    }

    public Trigger checkPriority(int priority) {
        return new Trigger(() -> commandPriority <= priority);
    }

    public Command setPattern(ControlRequest pattern, int priority) {
        return run(() -> {
            commandPriority = priority;
            setLEDs(pattern);
        })
                .ignoringDisable(true)
                .withName("LEDs.setPattern");
    }

    // --- Animation Helpers ---

    public StrobeAnimation strobe(Color c, double speed) {
        return new StrobeAnimation(0, config.length)
                .withColor(new RGBWColor(c));
    }

    public SingleFadeAnimation singleFade(Color c, double speed) {
        return new SingleFadeAnimation(0, config.length)
                .withColor(new RGBWColor(c));
    }

    public RgbFadeAnimation rgbFade(double speed) {
        return new RgbFadeAnimation(0, config.length);
    }

    public RainbowAnimation rainbow(double speed) {
        return new RainbowAnimation(0, config.length);
    }

    public StrobeAnimation solid(Color c) {
        return new StrobeAnimation(0, config.length)
                .withColor(new RGBWColor(c));
    }

    @Override
    public void setupStates() {
    }

    @Override
    public void setupDefaultCommand() {
        setDefaultCommand(defaultCommand);
    }
}