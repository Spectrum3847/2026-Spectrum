package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;

/** Power cycle or code restart: the decision behind {@code Turret.seedFromZeroReference}. */
class TurretBootZeroTest {

    /** One rotor turn of turret on PM_2026: 360 / 39.78. */
    private static final double ROTOR_TURN = 9.0498;

    @Test
    void readingOutsidePowerOnBandIsKeptWhateverTheFileSays() {
        assertTrue(
                Turret.decideBootZero(150.0, OptionalDouble.empty(), ROTOR_TURN)
                        .startsWith("kept"));
        assertTrue(
                Turret.decideBootZero(-30.0, OptionalDouble.of(3.0), ROTOR_TURN)
                        .startsWith("kept"));
        assertTrue(
                Turret.decideBootZero(9.2, OptionalDouble.empty(), ROTOR_TURN).startsWith("kept"));
    }

    @Test
    void readingInBandMatchingTheFileIsACodeRestart() {
        assertTrue(
                Turret.decideBootZero(4.22, OptionalDouble.of(4.30), ROTOR_TURN)
                        .startsWith("kept"));
        assertTrue(
                Turret.decideBootZero(0.0, OptionalDouble.of(0.1), ROTOR_TURN).startsWith("kept"));
    }

    @Test
    void readingInBandNotMatchingTheFileIsAPowerCycle() {
        // Chezy Q17 boot: 1.143 raw, turret last left near zero.
        assertTrue(
                Turret.decideBootZero(1.143, OptionalDouble.of(0.05), ROTOR_TURN)
                        .startsWith("seeded"));
        // Q11 boot: 5.977 raw, turret last left at 120 deg.
        assertTrue(
                Turret.decideBootZero(5.977, OptionalDouble.of(120.0), ROTOR_TURN)
                        .startsWith("seeded"));
        assertTrue(
                Turret.decideBootZero(5.977, OptionalDouble.empty(), ROTOR_TURN)
                        .startsWith("seeded"));
    }
}
