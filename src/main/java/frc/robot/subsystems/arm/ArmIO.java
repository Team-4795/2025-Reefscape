package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs{
        public double voltage = 0.0;
        public double angularPosition = 0.0;
        public double angularVelocity = 0.0;
        public double current = 0.0;
    }
    public default void setVoltage(double voltage) {

    }
    public default void setVelocity(double velocity){
        
    }
}
