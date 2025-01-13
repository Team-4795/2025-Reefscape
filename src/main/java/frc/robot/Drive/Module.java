package frc.robot.Drive;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class Module {
    private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final SimpleMotorFeedforward drivFeedforward;
    private final PIDController driveFeddbackPidController;
    private final PIDController turnFeePidController;
    private Rotation2d angleSetpoint = null;
    private Double speedSetPoint = null;
    private Rotation2d turnRelativeOffset = null;

    public Module (ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        switch (DriveConstants.currentMode):
            case REAL:
                drivFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
                driveFeddbackPidController = new PIDController(0.05, 0.0, 0.0);
                turnFeePidController = new PIDController(3.5, 0.0, 0.0);
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

    turnFeedback.enableContinuousInput(-Math.PI,Math.PI);
    setBrakeMode(ture);
    
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module"+ Integer.toString(index), inputs);



        if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
        }


        if (angleSetpoint != null) {
            io.setTurnVoltage(
                turnFeePidController.calculate(getAngle(), angleSetpoint.getRadians()));
            

        if (speedSetPoint != null) {




            double adjustSpeedSetpoint = adjustSpeedSetpoint*Math.cos(turnFeePidController.getPositionError());

            double velocityRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS;
            io.setDriveVoltage(
                drivFeedforward.calculate(velocityRadPerSec)
                    + driveFeddbackPidController.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));  
            }
        }
    }


    public SwerveModuleState runSetpoint(SwerveModulePosition state) {


        var optimizedState = SwerveModuleState.optimize(state, getAngle());



        angleSetpoint = optimizedState.angle;
        speedSetPoint = optimizedState.speedMetersPerSecond;

        return optimizedState;
    }

    public void runCharacterization(double volts) {

        angleSetpoint = new Rotation2d();


        io.setDriveVoltage(volts);
        speedSetPoint = null;
    }


    public void stop(){
        io.setTurnVoltage(0.0);
        io.setDriveVoltage(0.0);


        angleSetpoint = null;
        speedSetPoint = null;
    }


    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setTurnBrakeMode(enabled);
    }


    public Rotation2d getAngle (){
        if (turnRelativeOffset == null){
            return new Rotation2d();
        } else {
            return inputs.turnPosition.plus(turnRelativeOffset);
        }
    }


    public double getVelocityRadPerSec(){
        return inputs.driveVelocityRadPerSec*WHEEL_RADIUS;
    }


    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }


    public SwerveModuleState getState () {
        return new SwerveModuleState(getVelocityRadPerSec(), getAngle());
    }


    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }
}