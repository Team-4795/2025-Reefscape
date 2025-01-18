package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs{
        public double voltage = 0.0;
        public double angularPosition = 0.0;
        public double angularVelocity = 0.0;
        public double current = 0.0;
        public double goal = 0.0;
        public boolean openLoop = false;
    }

    public default void updateInputs(ArmIOInputs inputs) {
        
    }
    public default void setVoltage(double voltage) {

    }
    public default void setVelocity(double velocity){
        
    }
    public default void setGoal(double angle) {

    }
    public default void updateLoop() {

    }
}
