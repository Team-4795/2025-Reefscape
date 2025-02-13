
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


public class AutoAlignTest extends Command{
    private CommandSwerveDrivetrain driveTrain;
    private SwerveRequest.FieldCentric drive;
    private CommandXboxController joystick;
    private Telemetry logger;

    public AutoAlignTest(CommandSwerveDrivetrain driveTrain, SwerveRequest.FieldCentric drive, CommandXboxController joystick, Telemetry logger) {
        this.driveTrain = driveTrain;
        this.drive = drive;
        this.joystick = joystick;
        this.logger = logger;
        // addRequirements(driveTrain);
    }


    @Override
    public void initialize(){
       
    }

    @Override
    public void execute() {
        // driveTrain.registerTelemetry(logger::telemeterize);
        driveTrain.applyRequest(() ->
        drive.withVelocityX(-joystick.getLeftY() * 5) // Drive forward with negative Y (forward)
        .withVelocityY(-joystick.getLeftX() * 5) // Drive left with negative X (left)
        .withRotationalRate(-joystick.getRightX() * 5)); // Drive counterclockwise with negative X (left)
    }

    @Override
    public void end(boolean interuppted){
        
    }

   }
