package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs{
        public double voltage = 0;
        public double pos = 0;
        public double velocity = 0;
    }

    public default void setVoltage(double voltage){}

    public default double getPosition(){
        return 0;
    }

    public default void updateInputs(WristIOInputs inputs){}

    public default void setGoal(double angle){}

    public default void updateMotionProfile(){}
}