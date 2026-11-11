package io.dronefleet.mavlink.protocol.util;

import java.nio.charset.StandardCharsets;
import java.util.Objects;

public class CrcX25 {
    private int crc;

    public CrcX25() {
        crc = 0xffff;
    }

    public void accumulate(String str) {
        accumulate(str.getBytes(StandardCharsets.UTF_8));
    }

    public void accumulate(byte[] bytes) {
        accumulate(bytes, 0, bytes.length);
    }

    /**
     * Accumulates {@code length} bytes starting at {@code offset}.
     */
    public void accumulate(byte[] bytes, int offset, int length) {
        Objects.checkFromIndexSize(offset, length, bytes.length);
        for (int i = offset, end = offset + length; i < end; i++) {
            accumulate(bytes[i]);
        }
    }

    public void accumulate(int b) {
        b = b ^ (crc & 0xff);
        b ^= (b << 4) & 0xff;
        b &= 0xff;
        crc = (crc >> 8) ^ (b << 8) ^ (b << 3) ^ (b >> 4);
    }

    public int get() {
        return crc & 0xffff;
    }
}
