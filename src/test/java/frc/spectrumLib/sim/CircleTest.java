package frc.spectrumLib.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class CircleTest {
    /** Verifies circle initialization. */
    @Test
    @DisplayName("Test Circle creation and line background initialization")
    void testCircleInitialization() {
        Mechanism2d mech = new Mechanism2d(1.0, 1.0);
        MechanismRoot2d root = mech.getRoot("TestRoot", 0.5, 0.5);

        Circle circle = new Circle(8, 4.0, "TestCircle", root, mech);

        assertEquals(8, circle.getBackgroundLines());
        assertNotNull(circle.getCircleBackground());
        assertEquals(8, circle.getCircleBackground().length);

        for (int i = 0; i < 8; i++) {
            assertNotNull(circle.getCircleBackground()[i]);
        }
    }
    /** Verifies color settings. */
    @Test
    @DisplayName("Test Circle setBackgroundColor and setHalfBackground")
    void testColorSettings() {
        Mechanism2d mech = new Mechanism2d(1.0, 1.0);
        MechanismRoot2d root = mech.getRoot("TestRoot", 0.5, 0.5);

        Color8Bit colorRed = new Color8Bit(255, 0, 0);
        Color8Bit colorBlue = new Color8Bit(0, 0, 255);

        Circle circle = new Circle(mech, 4, 4.0, "TestCircle", root, colorRed);

        // Assert initial background is red as specified in constructor
        for (int i = 0; i < 4; i++) {
            assertEquals(colorRed, circle.getCircleBackground()[i].getColor());
        }

        circle.setBackgroundColor(colorBlue);
        for (int i = 0; i < 4; i++) {
            assertEquals(colorBlue, circle.getCircleBackground()[i].getColor());
        }

        circle.setHalfBackground(colorRed, colorBlue);
        assertEquals(colorRed, circle.getCircleBackground()[0].getColor());
        assertEquals(colorBlue, circle.getCircleBackground()[1].getColor());
        assertEquals(colorRed, circle.getCircleBackground()[2].getColor());
        assertEquals(colorBlue, circle.getCircleBackground()[3].getColor());
    }
}
