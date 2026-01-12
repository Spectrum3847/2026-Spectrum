package frc.spectrumLib.sim;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;

public class RollerConfig {
    @Getter private double rollerDiameterInches = 2;
    @Getter private int backgroundLines = 36;
    @Getter private double gearRatio = 5;
    @Getter private double simMOI = 0.01;
    @Getter private Color8Bit offColor = new Color8Bit(Color.kBlack);
    @Getter private Color8Bit fwdColor = new Color8Bit(Color.kGreen);
    @Getter private Color8Bit revColor = new Color8Bit(Color.kRed);
    @Getter private double initialX = 0;
    @Getter private double initialY = 0;
    @Getter private boolean mounted = false;
    @Getter private Mount mount;
    @Getter private double initMountX;
    @Getter private double initMountY;
    @Getter private double initMountAngle;

    public RollerConfig(double diameterInches) {
        rollerDiameterInches = diameterInches;
    }

    public RollerConfig setGearRatio(double ratio) {
        gearRatio = ratio;
        return this;
    }

    public RollerConfig setSimMOI(double moi) {
        simMOI = moi;
        return this;
    }

    public RollerConfig setPosition(double x, double y) {
        initialX = x;
        initialY = y;
        return this;
    }

    public RollerConfig setMount(LinearSim sim) {
        if (sim != null) {
            mounted = true;
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = Math.toRadians(sim.getConfig().getAngle());
        }

        return this;
    }

    public RollerConfig setMount(ArmSim sim) {
        if (sim != null) {
            mounted = true;
            mount = sim;
            initMountX = sim.getConfig().getInitialX();
            initMountY = sim.getConfig().getInitialY();
            initMountAngle = sim.getConfig().getStartingAngle();
        }

        return this;
    }
}
