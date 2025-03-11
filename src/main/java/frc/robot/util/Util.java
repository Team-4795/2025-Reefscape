package frc.robot.util;

import java.util.function.DoubleConsumer;

public class Util {
    public static void checkNull(Double value, DoubleConsumer consumer) {
        if(value != null) {
            consumer.accept(value);
        }
    }
}
