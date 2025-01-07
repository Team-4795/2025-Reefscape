package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog

    public static class ElevatorIOInputs {
        public double ElevatorInputVolts = 0.0;
        public double ElevatorAppliedVolts = 0.0;
        public double ElevatorPositionRads = 0.0;
        public double ElevatorVelocityRads = 0.0;
        public double ElevatorCurrent = 0.0;
    
        public double ElevatorMotorVelocityRadPerSec = 0.0;
        public double ElevatorMotorPositionRads = 0.0;
    }
    
        public default void updateInputs(ElevatorIOInputs inputs) {}
    
        public default void moveElevator(double speed) {}
    
        public default void setVoltage(double volts) {}   
        
        // True is brake, false is coast
        public default void setIdleMode(boolean mode) {}
    
        public default void resetEncoders() {}
    } 
    
