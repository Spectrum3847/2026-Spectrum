package frc.spectrumLib.leds;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;

public class SpectrumLEDs2 extends SubsystemBase{
    CANdle leds;
    LEDConfigs configs;

    public static class Config {
        @Getter private String name;
        @Getter @Setter private boolean attached = true;
        @Getter @Setter private int length;

        public Config(String name, int length) {
            this.name = name;
            this.length = length;
        }
    }

    @Getter private Config config;
    public static final int LedEndIndex = 120;

    protected final ControlRequest defaultPattern = new StrobeAnimation(0, LedEndIndex).withColor(new RGBWColor(Color.kGreen)).withFrameRate(500);

    @Getter
    protected Command defaultCommand =
            setPattern(defaultPattern, -1).withName("LEDs.defaultCommand");
    
    public final Trigger defaultTrigger = new Trigger(() -> defaultCommand.isScheduled());

    @Getter @Setter private int commandPriority = 0;

    public static final int ledsID = 61; //TODO: idk what this actually does so change that number to be significant
    
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
}

    public SpectrumLEDs2() {
        leds = new CANdle(ledsID);
        configs = new LEDConfigs();
        configs.StripType = StripTypeValue.RGB;
        tryUntilOk(5, () -> leds.getConfigurator().apply(configs));
    }

    public void setLEDs(ControlRequest request) {
            leds.setControl(request);
    }
    
    public Command setPattern(ControlRequest pattern, int priority) {
        return run(() -> {
                    commandPriority = priority;
                    setLEDs(pattern);})
                .ignoringDisable(true)
                .withName("LEDs.setPattern");
    }
}