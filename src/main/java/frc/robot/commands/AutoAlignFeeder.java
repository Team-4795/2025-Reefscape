
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.AprilTag.Vision;


public class AutoAlignFeeder extends Command{
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final Pose2d RED_FEEDER = new Pose2d(1.12, 6.92, new Rotation2d(-0.942477796076938));
    private static final Pose2d BLUE_FEEDER = new Pose2d(16.51, 1.14, new Rotation2d(2.199114857512855));

    private final double maxDistance = 0.6;
    private final double minDistance = -0.1;

    private ProfiledPIDController translationController;
    private ProfiledPIDController rotationController;


    private double mult;
    private Pose2d currentPose;
    private Pose2d targetPose;
    private double distance;

    public AutoAlignFeeder(ProfiledPIDController translation, ProfiledPIDController rotation) {
        translationController = translation;
        translationController.setTolerance(0.1);
        rotationController = rotation;
        rotationController.setTolerance(0.01);
        addRequirements(Swerve.getInstance());
    }


    @Override
    public void initialize(){
        DriverStation.getAlliance().ifPresent((alliance) -> {
            targetPose = (alliance == Alliance.Blue) ? RED_FEEDER : BLUE_FEEDER;
            mult = (alliance == Alliance.Red) ? -1.0 : 1.0;
        });
        
        currentPose = Swerve.getInstance().getState().Pose;
        double velocity = mult * projection(new Translation2d(Swerve.getInstance().getState().Speeds.vxMetersPerSecond, Swerve.getInstance().getState().Speeds.vyMetersPerSecond), targetPose.getTranslation().minus(currentPose.getTranslation()));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        Logger.recordOutput("AutoAlign/Robot velocity", new Translation2d(Swerve.getInstance().getState().Speeds.vxMetersPerSecond, Swerve.getInstance().getState().Speeds.vyMetersPerSecond));
        Logger.recordOutput("AutoAlign/Translation", targetPose.getTranslation().minus(currentPose.getTranslation()));
        Logger.recordOutput("AutoAlign/velocity", velocity);

        distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        translationController.reset(distance, velocity);
        rotationController.reset(MathUtil.angleModulus(currentPose.getRotation().getRadians()), Swerve.getInstance().getState().Speeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        // driveTrain.registerTelemetry(logger::telemeterize);

        currentPose = Swerve.getInstance().getState().Pose;
        distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        translationController.reset(distance, translationController.getSetpoint().velocity);

        double rotationPIDOutput = rotationController.calculate(MathUtil.angleModulus(currentPose.getRotation().getRadians()), targetPose.getRotation().getRadians());
        double omega = rotationController.getSetpoint().velocity + rotationPIDOutput;
        
        double scalar =  scalar(distance);
        double drivePIDOutput = translationController.calculate(distance, 0);
        double driveSpeed = mult * scalar * translationController.getSetpoint().velocity + drivePIDOutput;
        Rotation2d direction = new Rotation2d(currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());

        Logger.recordOutput("AutoAlign/target pose", targetPose);
        Logger.recordOutput("AutoAlign/Translation x direction", driveSpeed * direction.getCos());
        Logger.recordOutput("AutoAlign/Translation y direction", driveSpeed * direction.getSin());
        Logger.recordOutput("AutoAlign/Rotation setpoint position", rotationController.getSetpoint().position);
        Logger.recordOutput("AutoAlign/Rotation setpoint velocity", rotationController.getSetpoint().velocity);
        Logger.recordOutput("AutoAlign/Rotation", MathUtil.angleModulus(currentPose.getRotation().getRadians()));
        Logger.recordOutput("AutoAlign/Rotation at goal", rotationController.atGoal());

        Logger.recordOutput("AutoAlign/Translation setpoint position", translationController.getSetpoint().position);
        Logger.recordOutput("AutoAlign/Translation setpoint velocity", translationController.getSetpoint().velocity);
        Logger.recordOutput("AutoAlign/Distance", currentPose.getTranslation().getDistance(targetPose.getTranslation()));
        Logger.recordOutput("AutoAlign/Distance at goal", translationController.atGoal());
        Logger.recordOutput("AutoAlign/PID input", drivePIDOutput);

        Swerve.getInstance().setControl(
            drive.withVelocityX(driveSpeed * direction.getCos())
            .withVelocityY(driveSpeed * direction.getSin())
            .withRotationalRate(omega));
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
