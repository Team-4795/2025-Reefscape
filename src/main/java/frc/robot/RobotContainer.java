// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import choreo.auto.AutoChooser;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAlignFeeder;
import frc.robot.commands.AutoAlignReef;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIORealVortex;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.vision.AprilTag.Vision;
import frc.robot.subsystems.vision.AprilTag.VisionIOReal;
import frc.robot.subsystems.vision.AprilTag.VisionIOSim;
import frc.robot.util.NamedCommandManager;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOReal;

public class RobotContainer {
  private RobotVisualizer visualizer = new RobotVisualizer();
  private final Vision vision;

  // private final Vision vision;
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(SwerveConstants.MaxSpeed * 0.1).withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

  public final Swerve drivetrain;

  private Elevator elevator;
  // private Drive drive;
  private Intake intake;
  LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() throws IOException, ParseException {
    switch (Constants.currentMode) {
      case REAL:
        elevator = Elevator.initialize(new ElevatorIOReal());
        intake = Intake.initialize(new IntakeIORealVortex());
        Arm.initialize(new ArmIOReal());
        drivetrain = Swerve.initialize(TunerConstants.createDrivetrain());
        // vision = Vision.initialize(
        //   new VisionIOReal(0),
        //   new VisionIOReal(1),
        //   new VisionIOReal(2),
        //   new VisionIOReal(3)
        // );
        vision = Vision.initialize(new VisionIOSim());
        break;

      case SIM:
        elevator = Elevator.initialize(new ElevatorIOSim());
        intake = Intake.initialize(new IntakeIOSim());
        Arm.initialize(new ArmIOSim());
        drivetrain = Swerve.initialize(TunerConstants.createDrivetrain());
        vision = Vision.initialize(new VisionIOSim());
        break;

      default:
        elevator = Elevator.initialize(new ElevatorIOSim());
        intake = Intake.initialize(new IntakeIOSim());
        drivetrain = Swerve.initialize(TunerConstants.createDrivetrain());
        vision = Vision.initialize(new VisionIOSim());
        Arm.initialize(new ArmIOSim());
    }

    NamedCommandManager.registerNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser("TopBarge UFUB Score 2"));
    autoChooser.addOption("BottomBarge C BB BF", AutoBuilder.buildAuto("BottomBarge C BB BF"));
    autoChooser.addOption("raiseL4", AutoCommands.raiseL4());
    configureBindings();
    
  }

  private void configureBindings() {
    // Drive command
    drivetrain.setDefaultCommand(
        drivetrain
            .applyRequest(() -> drive.withVelocityX(-Constants.OIConstants.driverController.getLeftY() * SwerveConstants.MaxSpeed * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier: 1))
                .withVelocityY(-Constants.OIConstants.driverController.getLeftX() * SwerveConstants.MaxSpeed * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier: 1))
                .withRotationalRate(-Constants.OIConstants.driverController.getRightX() * SwerveConstants.MaxAngularRate * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier: 1))));

    // Zero heading
    Constants.OIConstants.driverController.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Reef/Feeder align
    Constants.OIConstants.driverController.leftBumper().whileTrue(
      Commands.either(
        AutoCommands.alignReef(), 
        AutoCommands.alignFeeder(), 
        () -> Intake.getInstance().hasGamepiece()
      )
    );

    // Slow mode
    Constants.OIConstants.driverController.leftTrigger().onTrue(Commands.runOnce(() -> drivetrain.setSlowMode(true)));
    Constants.OIConstants.driverController.leftTrigger().onFalse(Commands.runOnce(() -> drivetrain.setSlowMode(false)));

    // Outtake
    Constants.OIConstants.driverController.rightTrigger().whileTrue(
      Commands.startEnd(
        () -> intake.setIntakeSpeed(1),
        () -> intake.setIntakeSpeed(0), 
        intake
      )
    );

    // Elevator manual control
    elevator.setDefaultCommand(
      Commands.startEnd(
        () -> elevator.moveElevator(OIConstants.operatorController.getLeftY() / 1.5),
        () -> elevator.setVoltage(ElevatorConstants.kg),
        elevator
      )
    );

    // L4 setpoint - ADD OTHERS
    Constants.OIConstants.operatorController.povUp().onTrue(AutoCommands.raiseL4());

    // Stow
    Constants.OIConstants.operatorController.rightStick().onTrue(AutoCommands.stow());

    // Reverse intake
    Constants.OIConstants.operatorController.a().whileTrue(
      Commands.startEnd(
        () -> intake.setIntakeSpeed(-1),
        () ->  intake.setIntakeSpeed(0), 
        intake
      )
    );

    // Intake
    Constants.OIConstants.operatorController.y().whileTrue(
      Commands.sequence(
        intake.intake().until(() -> intake.GamePieceInitial()),
        intake.intakeSlow().until(() -> intake.GamePieceFinal()),
        Commands.runOnce(() -> intake.setIntakeSpeed(0))
      )
    );

    // OIConstants.operatorController.leftBumper().whileTrue(
    // Arm.getInstance().sysIDRoutine().quasistatic(SysIdRoutine.Direction.kForward)
    // );

    // OIConstants.operatorController.rightBumper().whileTrue(
    // Arm.getInstance().sysIDRoutine().quasistatic(SysIdRoutine.Direction.kReverse)
    // );

    // OIConstants.operatorController.leftTrigger().whileTrue(
    // Arm.getInstance().sysIDRoutine().dynamic(SysIdRoutine.Direction.kForward)
    // );

    // OIConstants.operatorController.rightTrigger().whileTrue(
    // Arm.getInstance().sysIDRoutine().dynamic(SysIdRoutine.Direction.kReverse)
    // );

    // Constants.OIConstants.driverController.back().and(Constants.OIConstants.driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // Constants.OIConstants.driverController.back().and(Constants.OIConstants.driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // Constants.OIConstants.driverController.start().and(Constants.OIConstants.driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // Constants.OIConstants.driverController.start().and(Constants.OIConstants.driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Constants.OIConstants.operatorController.povUp()
    // .whileTrue(
    // Commands.sequence(
    // intake.intake().until(() -> intake.GamePieceInitial()),
    // intake.intakeSlow().until(() -> intake.GamePieceFinal()),
    // Commands.run(() -> intake.setIntakeSpeed(0))
    // )
    // );

    // Constants.OIConstants.operatorController.x().whileTrue(Commands.startEnd(() -> intake.setIntakeSpeed(-1),
    //     () -> intake.setIntakeSpeed(0), intake));

    // Constants.OIConstants.operatorController.b().whileTrue(Commands.startEnd(() -> intake.setIntakeSpeed(1),
    //     () -> intake.setIntakeSpeed(0), intake));

    // OIConstants.driverController.povUp().onTrue(Commands.runOnce(() ->
    // Arm.getInstance().setGoal(ArmConstants.CORAL_L3)));
    // OIConstants.driverController.leftBumper().onTrue(Commands.runOnce(() ->
    // Arm.getInstance().setGoal(-1.8)));
    // OIConstants.driverController.rightBumper().onTrue(Commands.runOnce(() ->
    // Arm.getInstance().setGoal(ArmConstants.CORAL_L2)));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
