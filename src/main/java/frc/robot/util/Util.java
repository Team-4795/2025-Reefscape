package frc.robot.util;

import java.util.function.DoubleConsumer;

public class Util {
    // takes a Double and passes it into the consumer if the value is not null
    public static void nullOrDo(Double value, DoubleConsumer consumer) {
        if(value != null) {
            consumer.accept(value);
        }
    }
}
