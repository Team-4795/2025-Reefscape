package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double elevatorRightAppliedVolts = 0.0;
        public double elevatorRightPositionMeters= 0.0;
        public double elevatorRightVelocityMetersPerSecond = 0.0;
        public double elevatorRightCurrent = 0.0;

        public double elevatorLeftAppliedVolts = 0.0;
        public double elevatorLeftPositionMeters = 0.0;
        public double elevatorLeftVelocityMetersPerSecond = 0.0;
        public double elevatorLeftCurrent = 0.0;

        public double goalHeight = 0.0;
        public double setpointVelocity = 0.0;

        public boolean openLoop = true;
    }
    


        public default void updateInputs(ElevatorIOInputs inputs) {
            
        }

        public default void setGoal(double goal) {

    }        
    
        public default void moveElevator(double speed) {

        }
    
        public default void setVoltage(double volts) {}   
        
        // True is brake, false is coast
        public default void setIdleMode(boolean mode) {}
    
        public default void resetEncoders() {}

    } 
    
