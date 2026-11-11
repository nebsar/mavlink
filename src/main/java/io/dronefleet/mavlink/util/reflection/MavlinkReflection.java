package io.dronefleet.mavlink.util.reflection;

import io.dronefleet.mavlink.annotations.MavlinkEntryInfo;
import io.dronefleet.mavlink.annotations.MavlinkMessageInfo;

import java.lang.reflect.Field;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class MavlinkReflection {

    private static final ClassValue<EnumMetadata> ENUM_METADATA = new ClassValue<EnumMetadata>() {
        @Override
        protected EnumMetadata computeValue(Class<?> type) {
            Map<Enum, MavlinkEntryInfo> entries = new HashMap<>();
            Map<Integer, Enum> values = new HashMap<>();
            for (Field field : type.getFields()) {
                if (!field.isEnumConstant()) {
                    continue;
                }

                MavlinkEntryInfo info = field.getAnnotation(MavlinkEntryInfo.class);
                if (info == null) {
                    continue;
                }

                try {
                    Enum constant = (Enum) field.get(null);
                    entries.put(constant, info);
                    values.putIfAbsent(info.value(), constant);
                } catch (IllegalAccessException e) {
                    throw new IllegalStateException("Unable to read enum constant " + field, e);
                }
            }
            return new EnumMetadata(entries, values);
        }
    };

    public static int getEnumValue(Enum entry) {
        return getEnumEntry(entry)
                .orElseThrow(() -> new IllegalArgumentException(
                        "The specified entry is not annotated with @MavlinkEntryInfo"))
                .value();
    }

    public static Optional<MavlinkEntryInfo> getEnumEntry(Enum entry) {
        return Optional.ofNullable(ENUM_METADATA.get(entry.getDeclaringClass()).entries.get(entry));
    }

    public static <T extends Enum> Optional<T> getEntryByValue(Class<T> enumType, int value) {
        return Optional.ofNullable(enumType.cast(ENUM_METADATA.get(enumType).values.get(value)));
    }

    public static boolean isMavlinkMessage(Object o) {
        return getMessageInfo(o).isPresent();
    }

    public static Optional<MavlinkMessageInfo> getMessageInfo(Object message) {
        return Optional.ofNullable(message.getClass().getAnnotation(MavlinkMessageInfo.class));
    }

    private static final class EnumMetadata {
        private final Map<Enum, MavlinkEntryInfo> entries;
        private final Map<Integer, Enum> values;

        private EnumMetadata(Map<Enum, MavlinkEntryInfo> entries, Map<Integer, Enum> values) {
            this.entries = Collections.unmodifiableMap(entries);
            this.values = Collections.unmodifiableMap(values);
        }
    }
}
