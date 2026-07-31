package frc.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

public class ShiftHelpersTest {

    private Supplier<Optional<Boolean>> originalAllianceWinOverride;

    @Test
    @DisplayName("Test ShiftEnum values and ordering")
    void testShiftEnum() {
        ShiftHelpers.ShiftEnum[] values = ShiftHelpers.ShiftEnum.values();
        assertEquals(8, values.length);
        assertEquals(ShiftHelpers.ShiftEnum.TRANSITION, values[0]);
        assertEquals(ShiftHelpers.ShiftEnum.SHIFT1, values[1]);
        assertEquals(ShiftHelpers.ShiftEnum.SHIFT2, values[2]);
        assertEquals(ShiftHelpers.ShiftEnum.SHIFT3, values[3]);
        assertEquals(ShiftHelpers.ShiftEnum.SHIFT4, values[4]);
        assertEquals(ShiftHelpers.ShiftEnum.ENDGAME, values[5]);
        assertEquals(ShiftHelpers.ShiftEnum.AUTO, values[6]);
        assertEquals(ShiftHelpers.ShiftEnum.DISABLED, values[7]);
    }

    @Test
    @DisplayName("Test ShiftInfo record instantiation")
    void testShiftInfoRecord() {
        ShiftHelpers.ShiftInfo info =
                new ShiftHelpers.ShiftInfo(ShiftHelpers.ShiftEnum.SHIFT1, 15.0, 10.0, true);
        assertEquals(ShiftHelpers.ShiftEnum.SHIFT1, info.currentShift());
        assertEquals(15.0, info.elapsedTime(), 1e-6);
        assertEquals(10.0, info.remainingTime(), 1e-6);
        assertTrue(info.active());
    }

    @AfterEach
    void restoreStaticState() {
        if (originalAllianceWinOverride != null) {
            ShiftHelpers.setAllianceWinOverride(originalAllianceWinOverride);
            originalAllianceWinOverride = null;
        }
    }

    @Test
    @DisplayName("Test alliance win override setter and getter")
    void testAllianceWinOverride() {
        originalAllianceWinOverride = () -> ShiftHelpers.getAllianceWinOverride();

        ShiftHelpers.setAllianceWinOverride(() -> Optional.of(true));
        assertEquals(Optional.of(true), ShiftHelpers.getAllianceWinOverride());

        ShiftHelpers.setAllianceWinOverride(() -> Optional.empty());
        assertFalse(ShiftHelpers.getAllianceWinOverride().isPresent());
    }

    @Test
    @DisplayName("Test getOfficialShiftInfo non-null response")
    void testGetOfficialShiftInfo() {
        ShiftHelpers.initialize();
        ShiftHelpers.ShiftInfo info = ShiftHelpers.getOfficialShiftInfo();
        assertNotNull(info);
        assertNotNull(info.currentShift());
    }
}
