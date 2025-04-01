
package frc.robot.commands;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristConstants;
import frc.robot.subsystems.state.StateManager.OperationStates;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.AprilTag.Vision;
import frc.robot.util.LoggedTunableNumber;

public class AutoAlignBarge extends Command {
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final double minDistance = 0;

    private ProfiledPIDController translationController;
    private ProfiledPIDController rotationController;

    private double mult;
    private Pose2d currentPose;
    private Pose2d targetPose;
    private double distance;
    private double rotationError;
    private Alliance alliance = DriverStation.getAlliance().orElse(null);

    private double maxDistance = 1;

    public AutoAlignBarge(ProfiledPIDController translation, ProfiledPIDController rotation) {
        translationController = translation;
        translationController.setTolerance(Units.inchesToMeters(0.5));
        rotationController = rotation;
        rotationController.setTolerance(Units.degreesToRadians(0.5));
        addRequirements(Swerve.getInstance());
    }

    @Override
    public void initialize() {
        currentPose = Swerve.getInstance().getState().Pose;

        if(alliance != null) {
            mult = (alliance == Alliance.Red) ? -1.0 : 1.0;
        }
    
        if(alliance == Alliance.Red) {
            OperationStates.isBargeFowards = Math.abs(currentPose.getRotation().getDegrees()) >= 90;
            targetPose = new Pose2d(9.65, MathUtil.clamp(currentPose.getY(), 0.66, 3.5), new Rotation2d(OperationStates.isBargeFowards ? Math.PI : 0));
        }
        else if(alliance == Alliance.Blue){
            OperationStates.isBargeFowards = Math.abs(currentPose.getRotation().getDegrees()) <= 90;
            targetPose = new Pose2d(7.972670439618758, MathUtil.clamp(currentPose.getY(), 4.36, 7.47), new Rotation2d(OperationStates.isBargeFowards ? 0 : Math.PI));
        }
        
        double velocity = mult * projection(new Translation2d(Swerve.getInstance().getState().Speeds.vxMetersPerSecond, Swerve.getInstance().getState().Speeds.vyMetersPerSecond), targetPose.getTranslation().minus(currentPose.getTranslation()));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        Logger.recordOutput("AutoAlign/Robot velocity", new Translation2d(Swerve.getInstance().getState().Speeds.vxMetersPerSecond, Swerve.getInstance().getState().Speeds.vyMetersPerSecond));
        Logger.recordOutput("AutoAlign/Translation", targetPose.getTranslation().minus(currentPose.getTranslation()));
        Logger.recordOutput("AutoAlign/velocity", velocity);

        distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        translationController.reset(distance, velocity);

        rotationError = currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians();
        rotationController.reset(MathUtil.angleModulus(currentPose.getRotation().getRadians()), Swerve.getInstance().getState().Speeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        currentPose = Swerve.getInstance().getState().Pose;
        distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        translationController.reset(distance, translationController.getSetpoint().velocity);

        rotationError = MathUtil.angleModulus(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());
        double rotationPIDOutput = rotationController.calculate(MathUtil.angleModulus(currentPose.getRotation().getRadians()), targetPose.getRotation().getRadians());
        double omega = rotationController.getSetpoint().velocity + rotationPIDOutput;
        
        double scalar =  scalar(distance);
        double drivePIDOutput = translationController.calculate(distance, 0);
        double driveSpeed = mult * scalar * translationController.getSetpoint().velocity + drivePIDOutput;
        Rotation2d direction = new Rotation2d(currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());

        Logger.recordOutput("AutoAlignBarge/target pose", targetPose);
        Logger.recordOutput("AutoAlignBarge/Translation x direction", driveSpeed * direction.getCos());
        Logger.recordOutput("AutoAlignBarge/Translation y direction", driveSpeed * direction.getSin());
        Logger.recordOutput("AutoAlignBarge/Rotation setpoint position", rotationController.getSetpoint().position);
        Logger.recordOutput("AutoAlignBarge/Rotation setpoint velocity", rotationController.getSetpoint().velocity);
        Logger.recordOutput("AutoAlignBarge/Rotation", MathUtil.angleModulus(currentPose.getRotation().getRadians()));
        Logger.recordOutput("AutoAlignBarge/Rotation at goal", rotationController.atGoal());

        Logger.recordOutput("AutoAlignBarge/Translation setpoint position", translationController.getSetpoint().position);
        Logger.recordOutput("AutoAlignBarge/Translation setpoint velocity", translationController.getSetpoint().velocity);
        Logger.recordOutput("AutoAlignBarge/Distance", currentPose.getTranslation().getDistance(targetPose.getTranslation()));
        Logger.recordOutput("AutoAlignBarge/Distance at goal", distance < Units.inchesToMeters(0.5));
        Logger.recordOutput("AutoAlignBarge/PID input", drivePIDOutput);
        Logger.recordOutput("AutoAlignBarge/Rotation error", rotationError);

        Swerve.getInstance().setControl(
            drive.withVelocityX(driveSpeed * direction.getCos())
            .withVelocityY(driveSpeed * direction.getSin())
            .withRotationalRate(omega));
        
        OperationStates.aligned = finishedAligning();
        OperationStates.inScoringDistance = inScoringDistance();
    }

    @Override
    public void end(boolean interrupted) {
        // Swerve.getInstance().setControl(
        //     drive.withVelocityX(0)
        //     .withVelocityY(0)
        //     .withRotationalRate(0));
    }

    public boolean finishedAligning() {
        return (Math.abs(currentPose.getX() - targetPose.getX()) < Units.inchesToMeters(0.5)) && (Math.abs(rotationError) < Units.degreesToRadians(0.5)) && (Wrist.getInstance().atGoal(Wrist.getInstance().getGoal()));
    }

    public boolean inScoringDistance() {
        return (distance < 1 + Units.inchesToMeters(6));
    }

    private double projection(Translation2d v1, Translation2d onto){
        Vector<N2> velocity = VecBuilder.fill(v1.getX(), v1.getY());
        Vector<N2> translation = VecBuilder.fill(onto.getX(), onto.getY());
        Vector<N2> projection = velocity.projection(translation);
        if(projection.dot(translation) > 0) {
              return -Math.sqrt(projection.dot(projection));  
        } else {
           return Math.sqrt(projection.dot(projection));     
        }
    }

    private double scalar(double distance){
        if(distance > maxDistance){
            return 1.0;
        } else if (minDistance < distance && distance < maxDistance){
            return MathUtil.clamp((1 / (maxDistance - minDistance)) * (distance - minDistance), 0, 1);
        } else {
            return 0.0;
        }
    }
}
