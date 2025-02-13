// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIORealVortex;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.NamedCommandManager;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOReal;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private RobotVisualizer visualizer = new RobotVisualizer();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final Swerve drivetrain = TunerConstants.createDrivetrain();

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
        break;

      case SIM:
        elevator = Elevator.initialize(new ElevatorIOSim());
        intake = Intake.initialize(new IntakeIOSim());
        Arm.initialize(new ArmIOSim());
        break;

      default:
        elevator = Elevator.initialize(new ElevatorIOSim());
        intake = Intake.initialize(new IntakeIOSim());
        Arm.initialize(new ArmIOSim());


            
    }

    NamedCommandManager.registerNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser("BTopBarge DFDB"));
    autoChooser.addOption("BottomBarge C BB BF", AutoBuilder.buildAuto("BottomBarge C BB BF"));
    autoChooser.addOption("raiseL4", AutoCommands.raiseL4());
    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain
            .applyRequest(() -> drive.withVelocityX(-Constants.OIConstants.driverController.getLeftY() * MaxSpeed)
                .withVelocityY(-Constants.OIConstants.driverController.getLeftX() * MaxSpeed)
                .withRotationalRate(-Constants.OIConstants.driverController.getRightX() * MaxAngularRate)));

    // Constants.OIConstants.driverController.back().and(Constants.OIConstants.driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // Constants.OIConstants.driverController.back().and(Constants.OIConstants.driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // Constants.OIConstants.driverController.start().and(Constants.OIConstants.driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // Constants.OIConstants.driverController.start().and(Constants.OIConstants.driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    Constants.OIConstants.driverController.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);

    OIConstants.operatorController.leftTrigger()
        .whileTrue(
            Commands.startEnd(() -> elevator.moveElevator(OIConstants.operatorController.getLeftTriggerAxis() / 1.5),
                () -> elevator.moveElevator(0),
                elevator));

    OIConstants.operatorController.rightTrigger()
        .whileTrue(Commands.startEnd(
            () -> elevator.moveElevator(-OIConstants.operatorController.getRightTriggerAxis() / 1.5),
            () -> elevator.moveElevator(0),
            elevator));

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

    Constants.OIConstants.operatorController.povRight().onTrue(
        Commands.sequence(
            Commands.parallel(
                Commands.runOnce(() -> Arm.getInstance().setGoal(1.02), Arm.getInstance()),
                Elevator.getInstance().setGoal(0.57))));

    Constants.OIConstants.operatorController.povLeft().onTrue(
        Commands.sequence(
            Elevator.getInstance().setGoal(ElevatorConstants.minDistance),
            Commands.runOnce(() -> Arm.getInstance().setGoal(-1.8), Arm.getInstance()),
            Commands.waitSeconds(1.5)));

    // Constants.OIConstants.operatorController.povUp()
    // .whileTrue(
    // Commands.sequence(
    // intake.intake().until(() -> intake.GamePieceInitial()),
    // intake.intakeSlow().until(() -> intake.GamePieceFinal()),
    // Commands.run(() -> intake.setIntakeSpeed(0))
    // )
    // );

    Constants.OIConstants.operatorController.x().whileTrue(Commands.startEnd(() -> intake.setIntakeSpeed(-1),
        () -> intake.setIntakeSpeed(0), intake));

    Constants.OIConstants.operatorController.b().whileTrue(Commands.startEnd(() -> intake.setIntakeSpeed(1),
        () -> intake.setIntakeSpeed(0), intake));

    // OIConstants.driverController.povUp().onTrue(Commands.runOnce(() ->
    // Arm.getInstance().setGoal(ArmConstants.CORAL_L3)));
    // OIConstants.driverController.leftBumper().onTrue(Commands.runOnce(() ->
    // Arm.getInstance().setGoal(-1.8)));
    // OIConstants.driverController.rightBumper().onTrue(Commands.runOnce(() ->
    // Arm.getInstance().setGoal(ArmConstants.CORAL_L2)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
