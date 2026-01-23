package frc.spectrumLib.util;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertFalse;
import frc.spectrumLib.util.Network;
import org.junit.jupiter.api.Test;

public class TestNetwork {

    @Test
    public void testGetMacAddress() {
        String mac = Network.getMACaddress();
        assertNotNull(mac);
        assertFalse(mac.isEmpty());
    }

    @Test
    public void testGetIpAddress() {
        String ip = Network.getIPaddress();
        assertNotNull(ip);
        assertFalse(ip.isEmpty());
    }
}
