package io.dronefleet.mavlink.protocol.util;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class CrcX25Test {

    @Test
    public void accumulateUsesOffsetAndLength() {
        CrcX25 sliceCrc = new CrcX25();
        sliceCrc.accumulate(new byte[]{99, 1, 2, 3, 88}, 1, 3);

        CrcX25 exactCrc = new CrcX25();
        exactCrc.accumulate(new byte[]{1, 2, 3});

        assertEquals(exactCrc.get(), sliceCrc.get());
    }

    @Test(expected = IndexOutOfBoundsException.class)
    public void accumulateRejectsRangesOutsideArray() {
        new CrcX25().accumulate(new byte[]{1, 2}, 1, 2);
    }
}
