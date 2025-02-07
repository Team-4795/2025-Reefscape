package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

  public default void updateInputs(ModuleIOInputs inputs) {
  }

  public default void setDriveVoltage(double volts) {
  }

  public default void runMovementPID(double desiredStateRotPerSecond, double rotationDesiredPosition) {
  }

  public default void setDesiredState(SwerveModuleState state) {
  }

  public default void setTurnVoltage(double volts) {
  }

  public default void setTurnAngleReference(Rotation2d angle){

  }

  public default void setDriveBrakeMode(boolean enable) {
  }

  public default void setTurnBrakeMode(boolean enable) {
  }

  public default void getError(double error){
    
  }
}
