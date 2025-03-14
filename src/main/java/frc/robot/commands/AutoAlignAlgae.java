
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.AprilTag.Vision;


public class AutoAlignAlgae extends Command{
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final double maxDistance = 0.6;
    private final double minDistance = -0.1;

    private ProfiledPIDController translationController;
    private ProfiledPIDController rotationController;

    private double mult;
    private Pose2d currentPose;
    private Pose2d targetPose;
    private double distance;

    public AutoAlignAlgae(ProfiledPIDController translation, ProfiledPIDController rotation) {
        translationController = translation;
        translationController.setTolerance(Units.inchesToMeters(1));
        rotationController = rotation;
        rotationController.setTolerance(Units.degreesToRadians(1));
        addRequirements(Swerve.getInstance());
    }


    @Override
    public void initialize(){
        // Vision.getInstance().toggleShouldUpdate(0);
        // Vision.getInstance().toggleShouldUpdate(2);
        // Vision.getInstance().toggleShouldUpdate(3);
        // Vision.getInstance().toggleIsReefAligning();

        DriverStation.getAlliance().ifPresent((alliance) -> {
            mult = (alliance == Alliance.Red) ? -1.0 : 1.0;
        });

        targetPose = Vision.getInstance().getBestReefPose();

        currentPose = Swerve.getInstance().getState().Pose;
        double velocity = mult * projection(new Translation2d(Swerve.getInstance().getState().Speeds.vxMetersPerSecond, Swerve.getInstance().getState().Speeds.vyMetersPerSecond), targetPose.getTranslation().minus(currentPose.getTranslation()));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        Logger.recordOutput("AutoAlignAlgae/Robot velocity", new Translation2d(Swerve.getInstance().getState().Speeds.vxMetersPerSecond, Swerve.getInstance().getState().Speeds.vyMetersPerSecond));
        Logger.recordOutput("AutoAlignAlgae/Translation", targetPose.getTranslation().minus(currentPose.getTranslation()));
        Logger.recordOutput("AutoAlignAlgae/velocity", velocity);

        distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        translationController.reset(distance, velocity);
        rotationController.reset(MathUtil.angleModulus(currentPose.getRotation().getRadians()), Swerve.getInstance().getState().Speeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        targetPose = Vision.getInstance().getBestReefPose();

        currentPose = Swerve.getInstance().getState().Pose;
        distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        translationController.reset(distance, translationController.getSetpoint().velocity);

        double rotationPIDOutput = rotationController.calculate(MathUtil.angleModulus(currentPose.getRotation().getRadians()), targetPose.getRotation().getRadians());
        double omega = rotationController.getSetpoint().velocity + rotationPIDOutput;
        
        double scalar =  scalar(distance);
        double drivePIDOutput = translationController.calculate(distance, 0);
        double driveSpeed = mult * scalar * translationController.getSetpoint().velocity + drivePIDOutput;
        Rotation2d direction = new Rotation2d(currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());

        Logger.recordOutput("AutoAlignAlgae/target pose", targetPose);
        Logger.recordOutput("AutoAlignAlgae/Translation x direction", driveSpeed * direction.getCos());
        Logger.recordOutput("AutoAlignAlgae/Translation y direction", driveSpeed * direction.getSin());
        Logger.recordOutput("AutoAlignAlgae/Rotation setpoint position", rotationController.getSetpoint().position);
        Logger.recordOutput("AutoAlignAlgae/Rotation setpoint velocity", rotationController.getSetpoint().velocity);
        Logger.recordOutput("AutoAlignAlgae/Rotation", MathUtil.angleModulus(currentPose.getRotation().getRadians()));
        Logger.recordOutput("AutoAlignAlgae/Rotation at goal", rotationController.atGoal());

        Logger.recordOutput("AutoAlignAlgae/Translation setpoint position", translationController.getSetpoint().position);
        Logger.recordOutput("AutoAlignAlgae/Translation setpoint velocity", translationController.getSetpoint().velocity);
        Logger.recordOutput("AutoAlignAlgae/Distance", currentPose.getTranslation().getDistance(targetPose.getTranslation()));
        Logger.recordOutput("AutoAlignAlgae/Distance at goal", translationController.atGoal());
        Logger.recordOutput("AutoAlignAlgae/PID input", drivePIDOutput);

        Swerve.getInstance().setControl(
            drive.withVelocityX(driveSpeed * direction.getCos())
            .withVelocityY(driveSpeed * direction.getSin())
            .withRotationalRate(omega));
        
    }

    @Override
    public void end(boolean interrupted) {

    }

    public boolean finishedAligning() {
        return (translationController.atGoal() && rotationController.atGoal());
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
