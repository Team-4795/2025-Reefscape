package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorInputVolts = 0.0;
        public double elevatorAppliedVolts = 0.0;
        public double elevatorPositionMetersPerSecond = 0.0;
        public double elevatorVelocityMetersPerSecond = 0.0;
        public double elevatorCurrent = 0.0;
    
        public double elevatorMotorVelocityMetersPerSecond = 0.0;
        public double elevatorMotorPositionMeters = 0.0;
        public boolean openLoop = false;
    }
    

        public 
        
        public default void updateInputs(ElevatorIOInputs inputs) {
            
        }
    
        public default void moveElevator(double speed) {

        }
    
        public default void setVoltage(double volts) {}   
        
        // True is brake, false is coast
        public default void setIdleMode(boolean mode) {}
    
        public default void resetEncoders() {}

    } 
    
