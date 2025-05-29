package frc.robot.util;

import java.util.function.DoubleConsumer;

import org.littletonrobotics.junction.Logger;

public class Util {
    // takes a Double and passes it into the consumer if the value is not null
    public static void nullOrDo(Double value, DoubleConsumer consumer) {
        if(value != null) {
            consumer.accept(value);
            Logger.recordOutput("null", false);
        } else {
            Logger.recordOutput("null", true);
        }
    }
}
