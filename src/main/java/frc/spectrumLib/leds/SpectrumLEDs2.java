package frc.spectrumLib.leds;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpectrumLEDs2 extends SubsystemBase{
    CANdle leds;
    LEDConfigs configs;

    public static final int ledsID = 61;
    
    public static final int LedEndIndex = 120;
    
    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
}

    public SpectrumLEDs2() {
        leds = new CANdle(ledsID);
        configs = new LEDConfigs();
        configs.StripType = StripTypeValue.GRB;
        tryUntilOk(5, () -> leds.getConfigurator().apply(configs));
    }

    public void setLEDS(ControlRequest request) {
            leds.setControl(request);
    }
    
    public Command setLedCommand(ControlRequest request){
        return Commands.runOnce(()-> setLEDS(request));
    }
}