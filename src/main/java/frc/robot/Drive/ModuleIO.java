package frc.robot.Drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {

  @AutoLog
  public static class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnAppliedVolts = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedRadPerSec = 0.0;
    public double[] turnCurrentAmps = new double[] {};
  }  

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setDriveVoltage(double volts) {}

  public default void runTalonPID (double desiredStateRotPerSecond) {}

  public default void setTurnVoltage (double volts) {}

  public default void setDriveBrakeMode(boolean enable) {}

  public default void setTurnBrakeMode(boolean enable) {}
} 