package frc.robot.Drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;


public interface GyroIO {
    @AutoLog
public static class GyroIOInputs{
    public boolean conected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
}
    
public default void reset() {}

public default void updateInputs(GyroIOInputs inputs) {}
} 
    