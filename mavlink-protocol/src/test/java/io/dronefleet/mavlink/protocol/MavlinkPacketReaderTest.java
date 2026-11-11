package io.dronefleet.mavlink.protocol;

import org.junit.Test;

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class MavlinkPacketReaderTest {

    @Test
    public void itCanReadMaximumSizedMavlink2Packet() throws IOException {
        MavlinkPacket expected = MavlinkPacket.createSignedMavlink2Packet(
                0, 0, 0, 0, 0, new byte[255], 0, 0, new byte[32]);

        InputStream in = new ByteArrayInputStream(expected.getRawBytes());
        MavlinkPacketReader reader = new MavlinkPacketReader(in);
        MavlinkPacket actual = reader.next();

        assertEquals(expected, actual);
    }

    @Test
    public void signatureGenerationRejectsInvalidVersionMarker() {
        byte[] packet = MavlinkPacket.createSignedMavlink2Packet(
                0, 0, 0, 0, 0, new byte[0], 0, 0, new byte[32]).getRawBytes();
        packet[0] = (byte) 0xff;

        assertEquals(0, MavlinkPacket.generateSignature(packet, 0, 0, new byte[32]).length);
    }

    @Test
    public void signedPacketSignatureValidates() {
        MavlinkPacket packet = MavlinkPacket.createSignedMavlink2Packet(
                0, 0, 0, 0, 0, new byte[]{1, 2, 3}, 7, 12345, new byte[32]);

        assertTrue(packet.validateSignature(new byte[32]));
    }
}