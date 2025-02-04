package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Module {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    private final SimpleMotorFeedforward drivFeedforward;
    private final PIDController driveFeddbackPidController;
    private final PIDController turnFeePidController;
    private Rotation2d angleSetpoint = null;
    private Double speedSetPoint = null;
    private Rotation2d turnRelativeOffset = null;

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        switch (Constants.currentMode) {
            case REAL:
                drivFeedforward = new SimpleMotorFeedforward(DriveConstants.TranslationKS, DriveConstants.TranslationKV);
                driveFeddbackPidController = new PIDController(DriveConstants.TranslationKP, DriveConstants.TranslationKI, DriveConstants.TranslationKD);
                turnFeePidController = new PIDController(DriveConstants.RotationKP, DriveConstants.RotationKI, DriveConstants.RotationKD);
                break;
            case REPLAY:
                drivFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
                driveFeddbackPidController = new PIDController(0.05, 0.0, 0.0);
                turnFeePidController = new PIDController(7, 0.0, 0.0);
                break;
            case SIM:
                drivFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
                driveFeddbackPidController = new PIDController(0.05, 0.0, 0.0);
                turnFeePidController = new PIDController(10, 0.0, 0.0);
                break;
            default:
                drivFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
                driveFeddbackPidController = new PIDController(0.0, 0.0, 0.0);
                turnFeePidController = new PIDController(0.0, 0.0, 0.0);
                break;
        }

        turnFeePidController.enableContinuousInput(-Math.PI, Math.PI);
        setBrakeMode(true);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
        }

        if (angleSetpoint != null) {
            io.setTurnVoltage(turnFeePidController.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));
        }

        if (speedSetPoint != null) {
            double adjustSpeedSetpoint = speedSetPoint * Math.cos(turnFeePidController.getPositionError());

            double velocityRadPerSec = adjustSpeedSetpoint / DriveConstants.WHEEL_RADIUS;
            io.setDriveVoltage(
                    drivFeedforward.calculate(velocityRadPerSec)
                            + driveFeddbackPidController.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
        }
    }

    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        state.optimize(getAngle());
        angleSetpoint = state.angle;
        speedSetPoint = state.speedMetersPerSecond;
        return state;
    }

    public void runCharacterization(double volts) {

        angleSetpoint = new Rotation2d();

        io.setDriveVoltage(volts);
        speedSetPoint = null;
    }

    public void stop() {
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);

        angleSetpoint = null;
        speedSetPoint = null;
    }

    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }

    public Rotation2d getAngle() {
        if (turnRelativeOffset == null) {
            return new Rotation2d();
        } else {
            return inputs.turnAbsolutePosition;
        }
    }

    public double getVelocityRadPerSec() {
        return inputs.driveVelocityRadPerSec * DriveConstants.WHEEL_RADIUS;
    }

    public double getPositionMeters() {
        return inputs.drivePositionRad * DriveConstants.WHEEL_RADIUS;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityRadPerSec(), getAngle());
    }

    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        this.io.setDesiredState(desiredState);
    }
}
