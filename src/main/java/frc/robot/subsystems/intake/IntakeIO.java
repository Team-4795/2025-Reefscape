package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.units.measure.Velocity;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double voltage = 0.0;
        public double angularPositionRot = 0.0;
        public double angularVelocityRPM = 0.0;
        public double currentAmps = 0.0;
        public boolean hasGamepiece = false;
    }
    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void setMotorSpeed(double speed) {
    }

    public default SparkAbsoluteEncoder getArmAbsoluteEncoder() {
        return null;
    }

    public default boolean hasGamepiece() {
        return false;
    }

    public default double voltage() {
        return 0.0;
    }

    public default boolean velocitySensing(double RPM) {
        return false; 
    }

    
}

