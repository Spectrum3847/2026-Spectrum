package frc.spectrumLib.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lombok.Getter;
import lombok.Setter;

/**
 * Draws a circle in a {@link Mechanism2d} canvas as a ring of evenly spaced radial ligaments.
 * {@link RollerSim} uses it to show the roller's spin state by color.
 */
public class Circle {

    @SuppressWarnings("unused")
    private MechanismLigament2d rollerViz;

    @Getter private MechanismLigament2d[] circleBackground;
    @Getter private int backgroundLines;

    private double diameterInches;
    private MechanismRoot2d root;
    /** Color new background lines get when the circle is drawn. */
    @Setter private Color8Bit color = new Color8Bit(Color.kBlack);

    @Setter private String name;

    public Circle(
            int backgroundLines,
            double diameterInches,
            String name,
            MechanismRoot2d root,
            Mechanism2d mech) {
        this(mech, backgroundLines, diameterInches, name, root, new Color8Bit(Color.kBlack));
    }

    public Circle(
            Mechanism2d mech,
            int backgroundLines,
            double diameterInches,
            String name,
            MechanismRoot2d root,
            Color8Bit color) {
        this.backgroundLines = backgroundLines;
        this.diameterInches = diameterInches;
        this.name = name;
        this.root = root;
        this.circleBackground = new MechanismLigament2d[this.backgroundLines];
        this.color = color;
        drawCircle();
    }

    public void drawCircle() {
        for (int i = 0; i < backgroundLines; i++) {
            circleBackground[i] =
                    root.append(
                            new MechanismLigament2d(
                                    name + " Background " + i,
                                    Units.inchesToMeters(diameterInches) / 2.0,
                                    (360.0 / backgroundLines) * i,
                                    diameterInches,
                                    color));
        }
    }

    /** Appends a white radius line, so the spin direction is visible in the canvas. */
    public void drawViz() {
        rollerViz =
                root.append(
                        new MechanismLigament2d(
                                name + " Roller",
                                Units.inchesToMeters(diameterInches) / 2.0,
                                0.0,
                                5.0,
                                new Color8Bit(Color.kWhite)));
    }

    public void setBackgroundColor(Color8Bit color) {
        for (int i = 0; i < backgroundLines; i++) {
            circleBackground[i].setColor(color);
        }
    }

    /**
     * Alternates two colors across the radial lines, which reads as a two-tone circle.
     *
     * @param color8Bit color for even-indexed lines
     * @param color8Bit2 color for odd-indexed lines
     */
    public void setHalfBackground(Color8Bit color8Bit, Color8Bit color8Bit2) {
        for (int i = 0; i < backgroundLines; i++) {
            if (i % 2 == 0) {
                circleBackground[i].setColor(color8Bit);
            } else {
                circleBackground[i].setColor(color8Bit2);
            }
        }
    }
}
