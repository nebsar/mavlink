package io.dronefleet.mavlink.serialization.payload.reflection;

import io.dronefleet.mavlink.annotations.MavlinkFieldInfo;
import io.dronefleet.mavlink.annotations.MavlinkMessageInfo;
import io.dronefleet.mavlink.serialization.MavlinkSerializationException;
import io.dronefleet.mavlink.serialization.payload.MavlinkPayloadSerializer;
import io.dronefleet.mavlink.util.EnumValue;
import io.dronefleet.mavlink.util.WireFieldInfoComparator;
import io.dronefleet.mavlink.util.reflection.MavlinkReflection;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.math.BigInteger;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class ReflectionPayloadSerializer implements MavlinkPayloadSerializer {

    private static final WireFieldInfoComparator wireComparator = new WireFieldInfoComparator();
    private static final ClassValue<List<SerializedField>> FIELDS = new ClassValue<List<SerializedField>>() {
        @Override
        protected List<SerializedField> computeValue(Class<?> messageClass) {
            List<SerializedField> fields = new ArrayList<>();
            Arrays.stream(messageClass.getMethods())
                    .filter(method -> method.isAnnotationPresent(MavlinkFieldInfo.class))
                    .forEach(method -> fields.add(new SerializedField(
                            method, method.getAnnotation(MavlinkFieldInfo.class))));
            fields.sort(Comparator.comparing(SerializedField::getInfo, wireComparator));
            return fields;
        }
    };

    @Override
    public byte[] serialize(Object message) {
        Class<?> messageClass = message.getClass();
        if (!messageClass.isAnnotationPresent(MavlinkMessageInfo.class)) {
            throw new IllegalArgumentException(messageClass.getName() + " is not annotated with @MavlinkMessageInfo");
        }

        List<SerializedField> fields = FIELDS.get(messageClass);
        int payloadLength = fields.stream()
                .mapToInt(field -> field.info.unitSize() * Math.max(1, field.info.arraySize()))
                .sum();
        if (payloadLength > 255) {
            throw new IllegalStateException("payload length > 255 for message" + messageClass.getName());
        }
        byte[] payload = new byte[payloadLength];

        int offset = 0;
        for (SerializedField serializedField : fields) {
            MavlinkFieldInfo field = serializedField.info;
            int length = field.unitSize() * Math.max(field.arraySize(), 1);
            try {
                Object fieldValue = serializedField.method.invoke(message);
                if (fieldValue != null) {
                    write(fieldValue, payload, offset, offset + length, field.unitSize());
                }
            } catch (IllegalAccessException | InvocationTargetException e) {
                throw new IllegalStateException("Unable to serialize field " + serializedField.method, e);
            }
            offset += length;
        }

        return payload;
    }

    private static final class SerializedField {
        private final Method method;
        private final MavlinkFieldInfo info;

        private SerializedField(Method method, MavlinkFieldInfo info) {
            this.method = method;
            this.info = info;
        }

        private MavlinkFieldInfo getInfo() {
            return info;
        }
    }

    private void write(Object value, byte[] buffer, int offset, int length, int unitSize) {
        Class<?> type = value.getClass();
        if (Integer.class.isAssignableFrom(type)) {
            write((Integer) value, buffer, offset, length);
        } else if (Long.class.isAssignableFrom(type)) {
            write((Long) value, buffer, offset, length);
        } else if (BigInteger.class.isAssignableFrom(type)) {
            write((BigInteger) value, buffer, offset, length);
        } else if (Float.class.isAssignableFrom(type)) {
            write((Float) value, buffer, offset, length);
        } else if (Double.class.isAssignableFrom(type)) {
            write((Double) value, buffer, offset, length);
        } else if (Enum.class.isAssignableFrom(type)) {
            write((Enum) value, buffer, offset, length);
        } else if (EnumValue.class.isAssignableFrom(type)) {
            write((EnumValue<? extends Enum>) value, buffer, offset, length);
        } else if (String.class.isAssignableFrom(type)) {
            write((String) value, buffer, offset, length);
        } else if (byte[].class.isAssignableFrom(type)) {
            write((byte[]) value, buffer, offset, length);
        } else if (List.class.isAssignableFrom(type)) {
            write((List<?>) value, buffer, offset, length, unitSize);
        } else {
            throw new MavlinkSerializationException("unrecognized field type " + type.getName());
        }
    }

    private void write(long value, byte[] buffer, int offset, int length) {
        for (int i = offset, shift = 0; i < length; i++, shift++) {
            buffer[i] = (byte) ((value >> (shift * 8)) & 0xff);
        }
    }

    private void write(BigInteger bigInt, byte[] buffer, int offset, int length) {
        write(bigInt.longValue(), buffer, offset, length);
    }

    private void write(float value, byte[] buffer, int offset, int length) {
        if ((length - offset) != 4) throw new IllegalArgumentException("length != 4");
        ByteBuffer.wrap(buffer)
                .order(ByteOrder.LITTLE_ENDIAN)
                .putFloat(offset, value);
    }

    private void write(double value, byte[] buffer, int offset, int length) {
        if ((length - offset) != 8) throw new IllegalArgumentException("length != 8");
        ByteBuffer.wrap(buffer)
                .order(ByteOrder.LITTLE_ENDIAN)
                .putDouble(offset, value);
    }

    private void write(Enum value, byte[] buffer, int offset, int length) {
        write(MavlinkReflection.getEnumValue(value), buffer, offset, length);
    }

    private void write(EnumValue<? extends Enum> value, byte[] buffer, int offset, int length) {
        write(value.value(), buffer, offset, length);
    }

    private void write(String str, byte[] buffer, int offset, int length) {
        write(str.getBytes(StandardCharsets.UTF_8), buffer, offset, length);
    }

    private void write(byte[] bytes, byte[] buffer, int offset, int length) {
        System.arraycopy(bytes, 0, buffer, offset, Math.min(length - offset, bytes.length));
    }

    private void write(List<?> list, byte[] buffer, int offset, int length, int unitSize) {
        if (list.size() > (length / unitSize)) {
            throw new IllegalStateException("too many items in list");
        }
        for (Object value : list) {
            write(value, buffer, offset, offset + unitSize, 1);
            offset += unitSize;
        }
    }
}
