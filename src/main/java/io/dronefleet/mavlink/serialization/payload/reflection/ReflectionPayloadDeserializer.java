package io.dronefleet.mavlink.serialization.payload.reflection;

import io.dronefleet.mavlink.annotations.MavlinkFieldInfo;
import io.dronefleet.mavlink.annotations.MavlinkMessageBuilder;
import io.dronefleet.mavlink.annotations.MavlinkMessageInfo;
import io.dronefleet.mavlink.serialization.MavlinkSerializationException;
import io.dronefleet.mavlink.serialization.payload.MavlinkPayloadDeserializer;
import io.dronefleet.mavlink.util.EnumValue;
import io.dronefleet.mavlink.util.WireFieldInfoComparator;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;
import java.math.BigInteger;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.*;

public class ReflectionPayloadDeserializer implements MavlinkPayloadDeserializer {

    private static final WireFieldInfoComparator wireComparator = new WireFieldInfoComparator();
    private static final ClassValue<Method> BUILDERS = new ClassValue<Method>() {
        @Override
        protected Method computeValue(Class<?> messageType) {
            return Arrays.stream(messageType.getMethods())
                    .filter(method -> method.isAnnotationPresent(MavlinkMessageBuilder.class))
                    .findFirst()
                    .orElseThrow(() -> new MavlinkSerializationException(
                            "Message " + messageType.getName() + " does not have a builder"));
        }
    };
    private static final ClassValue<List<DeserializedField>> FIELDS =
            new ClassValue<List<DeserializedField>>() {
                @Override
                protected List<DeserializedField> computeValue(Class<?> builderType) {
                    List<DeserializedField> fields = new ArrayList<>();
                    Arrays.stream(builderType.getMethods())
                            .filter(method -> method.isAnnotationPresent(MavlinkFieldInfo.class))
                            .forEach(method -> {
                                Type[] parameterTypes = method.getGenericParameterTypes();
                                if (parameterTypes.length != 1) {
                                    throw new MavlinkSerializationException(
                                            "Method " + method.getName() + " of " + builderType.getName()
                                                    + " is annotated with @MavlinkFieldInfo, however does not "
                                                    + "accept a single parameter.");
                                }
                                fields.add(new DeserializedField(method,
                                        method.getAnnotation(MavlinkFieldInfo.class), parameterTypes[0]));
                            });
                    fields.sort((a, b) -> wireComparator.compare(a.info, b.info));
                    return fields;
                }
            };

    @Override
    public <T> T deserialize(byte[] payload, Class<T> messageType) {
        MavlinkMessageInfo message = messageType.getAnnotation(MavlinkMessageInfo.class);
        if (message == null) {
            throw new IllegalArgumentException(String.format(
                    "class %s is not annotated with @MavlinkMessageInfo", messageType.getName()));
        }

        Object builder;
        try {
            builder = BUILDERS.get(messageType).invoke(null);
        } catch (IllegalAccessException | InvocationTargetException e) {
            throw serializationFailure("Unable to create builder for " + messageType.getName(), e);
        }

        int offset = 0;
        for (DeserializedField deserializedField : FIELDS.get(builder.getClass())) {
            MavlinkFieldInfo field = deserializedField.info;
            int length = field.unitSize() * Math.max(field.arraySize(), 1);
            byte[] data = new byte[length];
            if (offset < payload.length) {
                int copyLength = Math.min(length, payload.length - offset);
                System.arraycopy(payload, offset, data, 0, copyLength);
            }

            try {
                deserializedField.method.invoke(builder,
                        deserialize(deserializedField.type, data, 0, data.length, field));
            } catch (IllegalAccessException | InvocationTargetException e) {
                throw serializationFailure("Unable to deserialize field "
                        + deserializedField.method.getName() + " of " + messageType.getName(), e);
            }
            offset += length;
        }

        try {
            //noinspection unchecked
            return (T) builder.getClass().getMethod("build").invoke(builder);
        } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
            throw serializationFailure("Unable to build message " + messageType.getName(), e);
        }
    }

    private static MavlinkSerializationException serializationFailure(String message, Exception cause) {
        Throwable actualCause = cause instanceof InvocationTargetException
                && ((InvocationTargetException) cause).getCause() != null
                ? ((InvocationTargetException) cause).getCause()
                : cause;
        return new MavlinkSerializationException(message, actualCause);
    }

    private static final class DeserializedField {
        private final Method method;
        private final MavlinkFieldInfo info;
        private final Type type;

        private DeserializedField(Method method, MavlinkFieldInfo info, Type type) {
            this.method = method;
            this.info = info;
            this.type = type;
        }
    }

    private Object deserialize(Type fieldType, byte[] data, int offset, int length, MavlinkFieldInfo field) {
        if (fieldType instanceof Class) {
            Class fieldClass = (Class<?>) fieldType;
            if (int.class.isAssignableFrom(fieldClass) || Integer.class.isAssignableFrom(fieldClass)) {
                return (int) integerValue(data, offset, length, field.signed());
            } else if (long.class.isAssignableFrom(fieldClass) || Long.class.isAssignableFrom(fieldClass)) {
                return integerValue(data, offset, length, field.signed());
            } else if (float.class.isAssignableFrom(fieldClass) || Float.class.isAssignableFrom(fieldClass)) {
                return floatValue(data, offset);
            } else if (double.class.isAssignableFrom(fieldClass) || Double.class.isAssignableFrom(fieldClass)) {
                return doubleValue(data, offset);
            } else if (String.class.isAssignableFrom(fieldClass)) {
                return stringValue(data);
            } else if (byte[].class.isAssignableFrom(fieldClass)) {
                return data;
            } else if (BigInteger.class.isAssignableFrom(fieldClass)) {
                return bigIntValue(data);
            }
        } else if (fieldType instanceof ParameterizedType) {
            ParameterizedType parameterizedType = (ParameterizedType) fieldType;
            Class<?> fieldClass = (Class<?>) parameterizedType.getRawType();
            if (EnumValue.class.isAssignableFrom(fieldClass)) {
                return enumValue(field.enumType(), data, offset, length, field.signed());
            } else if (List.class.isAssignableFrom(fieldClass)) {
                return listValue((Class<?>) parameterizedType.getActualTypeArguments()[0], data, field);
            }
        }
        throw new MavlinkSerializationException("Unsupported field type " + fieldType);
    }

    private long unsignedIntegerValue(byte[] data, int offset, int length) {
        long value = 0;
        for (int i = offset; i < Math.min(data.length, length); i++) {
            value = value | ((long) ((data[i] & 0xff)) << ((i - offset) * 8));
        }
        return value;
    }

    private long signedIntegerValue(byte[] data, int offset, int length) {
        long value = unsignedIntegerValue(data, offset, length);
        int signBitIndex = (length - offset) * Byte.SIZE - 1;
        if ((value >> signBitIndex) == 1) {
            value |= (-1L << signBitIndex);
        }
        return value;
    }

    private long integerValue(byte[] data, int offset, int length, boolean signed) {
        if (signed) {
            return signedIntegerValue(data, offset, length);
        } else {
            return unsignedIntegerValue(data, offset, length);
        }
    }

    private double doubleValue(byte[] data, int offset) {
        return ByteBuffer.wrap(data)
                .order(ByteOrder.LITTLE_ENDIAN)
                .getDouble(offset);
    }

    private float floatValue(byte[] data, int offset) {
        return ByteBuffer.wrap(data)
                .order(ByteOrder.LITTLE_ENDIAN)
                .getFloat(offset);
    }

    private String stringValue(byte[] data) {
        for (int i = 0; i < data.length; i++) {
            if (data[i] == 0) {
                return new String(data, 0, i, StandardCharsets.UTF_8);
            }
        }
        return new String(data, StandardCharsets.UTF_8);
    }


    private BigInteger bigIntValue(byte[] data) {
        // Invert to big-endian, for BigInteger constructor
        for (int i = 0; i < data.length / 2; ++i) {
            byte tmp = data[i];
            data[i] = data[data.length - 1 - i];
            data[data.length - 1 - i] = tmp;
        }

        return new BigInteger(data);
    }

    private Object enumValue(Class<?> enumType, byte[] data, int offset, int length, boolean signed) {
        return EnumValue.create(
                (Class<? extends Enum>) enumType,
                (int) integerValue(data, offset, length, signed));
    }

    private List<?> listValue(Class<?> listType, byte[] data, MavlinkFieldInfo field) {
        int unitSize = field.unitSize();
        List<Object> result = new ArrayList<>(data.length / unitSize);
        for (int offset = 0; offset < data.length; offset += unitSize) {
            Object value = deserialize(listType, data, offset, offset + unitSize, field);
            result.add(value);
        }
        return Collections.unmodifiableList(result);
    }
}
