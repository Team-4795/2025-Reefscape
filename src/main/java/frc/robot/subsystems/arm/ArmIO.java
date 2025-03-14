package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;
public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs{
        public double voltage = 0.0;
        public double angularPosition = 0.0;
        public double angularVelocity = 0.0;
        public double current = 0.0;
        public double goalAngle = 0.0;
        public double setpointVelocity = 0.0;
        public double appliedOutput = 0.0; 
        public double busVoltage = 0.0;
        public double relativeEncoderPosition = 0.0;
        public double relativeEncoderVelocity = 0.0;
        public double setpointPosition = 0.0;
        public double angularPositionDegrees = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {
        
    }

    public default void resetEncoder() {
    }

    public default void setVoltage(double voltage) {

    }

    public default double getGoal() {
        return 0.0;
    }

    public default void updateMotionProfile(){
        
    }
    
    public default void setGoal(double angle) {
    }

    public default void hold() {}

    public default void setFFValues(double kS, double kG, double kV, double kA) {

    }
}
