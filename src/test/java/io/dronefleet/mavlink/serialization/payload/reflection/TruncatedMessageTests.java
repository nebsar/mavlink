package io.dronefleet.mavlink.serialization.payload.reflection;

import io.dronefleet.mavlink.common.MavMissionResult;
import io.dronefleet.mavlink.common.MavMissionType;
import io.dronefleet.mavlink.common.MissionAck;
import io.dronefleet.mavlink.util.EnumValue;
import org.junit.Test;

import java.util.HexFormat;

import static org.junit.Assert.assertEquals;

public class TruncatedMessageTests {

    private final ReflectionPayloadDeserializer deserializer= new ReflectionPayloadDeserializer();

    @Test
    public void testTruncatedPayloadDeserialization() {
        byte[] payload = HexFormat.of().parseHex("ff");
        MissionAck message = deserializer.deserialize(payload, MissionAck.class);

        assertEquals(255, message.targetSystem());
        assertEquals(0, message.targetComponent());
        assertEquals(EnumValue.of(MavMissionResult.MAV_MISSION_ACCEPTED), message.type());
        assertEquals(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION), message.missionType());
    }

    @Test
    public void testNonTruncatedPayloadDeserialization() {
        byte[] payload = HexFormat.of().parseHex("ff000000");
        MissionAck message = deserializer.deserialize(payload, MissionAck.class);

        assertEquals(255, message.targetSystem());
        assertEquals(0, message.targetComponent());
        assertEquals(EnumValue.of(MavMissionResult.MAV_MISSION_ACCEPTED), message.type());
        assertEquals(EnumValue.of(MavMissionType.MAV_MISSION_TYPE_MISSION), message.missionType());
    }
}
