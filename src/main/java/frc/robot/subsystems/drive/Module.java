package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Module {
    private static final LoggedTunableNumber drivekS =
    new LoggedTunableNumber("Drive/Module/DrivekS");
private static final LoggedTunableNumber drivekV =
    new LoggedTunableNumber("Drive/Module/DrivekV");
private static final LoggedTunableNumber drivekP =
    new LoggedTunableNumber("Drive/Module/DrivekP");
    private static final LoggedTunableNumber drivekI =
    new LoggedTunableNumber("Drive/Module/DrivekI");
    private static final LoggedTunableNumber drivekD =
    new LoggedTunableNumber("Drive/Module/DrivekD");
private static final LoggedTunableNumber turnkP = new LoggedTunableNumber("Drive/Module/TurnkP");
private static final LoggedTunableNumber turnkI = new LoggedTunableNumber("Drive/Module/TurnkI");
private static final LoggedTunableNumber turnkD = new LoggedTunableNumber("Drive/Module/TurnkD");

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
    //  private final SimpleMotorFeedforward drivFeedforward;
    //  private final PIDController driveFeedBackPidController;
    private Rotation2d angleSetpoint = null;
    private Double speedSetPoint = null;
    private Rotation2d turnRelativeOffset = null;
    
    

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        switch (Constants.currentMode) {
            case REAL:
            drivekS.initDefault(0.014);
            drivekV.initDefault(0.134);
            drivekP.initDefault(0.1);
            drivekI.initDefault(0);
            drivekD.initDefault(0);
            turnkP.initDefault(10);
            turnkI.initDefault(0);
            turnkD.initDefault(0);
            // drivFeedforward = new SimpleMotorFeedforward(DriveConstants.TranslationKS, DriveConstants.TranslationKV);
               // driveFeedBackPidController = new PIDController(DriveConstants.TranslationKP, DriveConstants.TranslationKI, DriveConstants.TranslationKD);
                 break;
            case REPLAY:
                //  drivFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
                //  driveFeedBackPidController = new PIDController(0.05, 0.0, 0.0);
                break;
            case SIM:
                //  drivFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
                //  driveFeedBackPidController = new PIDController(0.05, 0.0, 0.0);
                break;
            default:
                //  drivFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
                //  driveFeedBackPidController = new PIDController(0.0, 0.0, 0.0);
                break;
        }

        
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
        }

        if (angleSetpoint != null) {
            io.setTurnAngleReference(angleSetpoint);;
        }

        // if (speedSetPoint != null) {
        //     double adjustSpeedSetpoint = speedSetPoint * Math.cos(io.getError()) ; // add equation with onboard DrivePid

            
        //     double velocityRadPerSec = adjustSpeedSetpoint / DriveConstants.WHEEL_RADIUS;
        //     io.setDriveVoltage(
        //             drivFeedforward.calculate(velocityRadPerSec)
        //                     + driveFeedBackPidController.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
        // }
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

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * DriveConstants.WHEEL_RADIUS;
    }

    public double getPositionMeters() {
        return inputs.drivePositionRad * DriveConstants.WHEEL_RADIUS;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        this.io.setDesiredState(desiredState);
    }
}
