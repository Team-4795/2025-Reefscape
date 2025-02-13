
package frc.robot.commands;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.AprilTag.Vision;
import frc.robot.subsystems.vision.AprilTag.VisionConstants;


public class AutoAlignReef extends Command{
    private static final Pose2d[] RED_SCORING_AREAS = VisionConstants.redReefScoringPoses;
    private static final Pose2d[] BLUE_SCORING_AREAS = VisionConstants.blueReefScoringPoses;

    private boolean isScoringLeft = true;
    private double offset = 0.0;

    private final double maxDistance = 0.6;
    private final double minDistance = -0.1;

    private ProfiledPIDController translationController;
    private ProfiledPIDController rotationController;

    private double mult;
    private Pose2d currentPose;
    private Pose2d targetPose;
    private double distance;

    private CommandSwerveDrivetrain driveTrain;
    private SwerveRequest.FieldCentric drive;

    public AutoAlignReef(CommandSwerveDrivetrain driveTrain, SwerveRequest.FieldCentric drive, ProfiledPIDController translation, ProfiledPIDController rotation) {
        translationController = translation;
        translationController.setTolerance(0.1);
        rotationController = rotation;
        this.driveTrain = driveTrain;
        this.drive = drive;
        // addRequirements(driveTrain);
    }


    @Override
    public void initialize(){
        DriverStation.getAlliance().ifPresent((alliance) -> {
            targetPose = (alliance == Alliance.Blue) ? BLUE_SCORING_AREAS[0] : RED_SCORING_AREAS[0];
            mult = (alliance == Alliance.Red) ? -1.0 : 1.0;
        });
        
        offset = (isScoringLeft) ? 0.33 / 2.0 :  -0.33 / 2.0;
        targetPose = targetPose.plus(new Transform2d(0, offset, new Rotation2d(0)));
        
        currentPose = driveTrain.getState().Pose;
        double velocity = mult * projection(new Translation2d(driveTrain.getState().Speeds.vxMetersPerSecond, driveTrain.getState().Speeds.vyMetersPerSecond), targetPose.getTranslation().minus(currentPose.getTranslation()));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        Logger.recordOutput("AutoAlign/Robot velocity", new Translation2d(driveTrain.getState().Speeds.vxMetersPerSecond, driveTrain.getState().Speeds.vyMetersPerSecond));
        Logger.recordOutput("AutoAlign/Translation", targetPose.getTranslation().minus(currentPose.getTranslation()));
        Logger.recordOutput("AutoAlign/velocity", velocity);

        distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        translationController.reset(distance, velocity);
        rotationController.reset(MathUtil.angleModulus(currentPose.getRotation().getRadians()), driveTrain.getState().Speeds.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        // driveTrain.registerTelemetry(logger::telemeterize);

        currentPose = driveTrain.getState().Pose;
        distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        translationController.reset(distance, translationController.getSetpoint().velocity);

        double rotationPIDOutput = rotationController.calculate(MathUtil.angleModulus(currentPose.getRotation().getRadians()), targetPose.getRotation().getRadians());
        double omega = rotationController.getSetpoint().velocity + rotationPIDOutput;
        
        double scalar = scalar(distance);
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

        driveTrain.applyRequest(() -> drive.withVelocityX(driveSpeed * direction.getCos()).withVelocityY(driveSpeed * direction.getCos()).withRotationalRate(omega));
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
