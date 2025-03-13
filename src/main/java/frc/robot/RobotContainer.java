// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.RainbowCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.GenericRequirement;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIORealVortex;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.vision.AprilTag.Vision;
import frc.robot.subsystems.vision.AprilTag.VisionIOReal;
import frc.robot.subsystems.vision.AprilTag.VisionIOSim;
import frc.robot.util.NamedCommandManager;

public class RobotContainer {
  private RobotVisualizer visualizer;
  private Vision vision;
  private LEDs leds; 

  // private final Vision vision;
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(SwerveConstants.MaxSpeed * 0.04).withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.04) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

  public final Swerve drivetrain;

  public int autoScoreMode = 1;

  private Elevator elevator;
  // private Drive drive;
  private Intake intake;
  LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() throws IOException, ParseException {
    GenericRequirement.initialize();
    switch (Constants.currentMode) {
      case REAL:
        elevator = Elevator.initialize(new ElevatorIOReal());
        intake = Intake.initialize(new IntakeIORealVortex());
        Arm.initialize(new ArmIOReal());
        drivetrain = Swerve.initialize(new Swerve(TunerConstants.DrivetrainConstants, 50, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight));
        vision = Vision.initialize(
          new VisionIOReal(0), new VisionIOReal(1)
          // new VisionIOReal(2),
          // new VisionIOReal(3)
        );  
        break;

      case SIM:
        elevator = Elevator.initialize(new ElevatorIOSim());
        intake = Intake.initialize(new IntakeIOSim());
        Arm.initialize(new ArmIOSim());
        drivetrain = Swerve.initialize(TunerConstants.createDrivetrain());
        visualizer = new RobotVisualizer();
        if(Constants.visonSimEnabled) {
          vision = Vision.initialize(new VisionIOSim());
        }
        break;

      default:
        elevator = Elevator.initialize(new ElevatorIOSim());
        intake = Intake.initialize(new IntakeIOSim());
        drivetrain = Swerve.initialize(TunerConstants.createDrivetrain());
        Arm.initialize(new ArmIOSim());
        break;
    }
    leds = LEDs.getInstance();

    NamedCommandManager.registerNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser("Driver Forward Straight"));
    autoChooser.addOption("BottomBarge C BB BF", AutoBuilder.buildAuto("BottomBarge C BB BF"));
    configureBindings();
    
  }

  public void zeroArm() {
    Arm.getInstance().seedRelativeEncoder();
  }

  private void configureBindings() {
    // Drive command
    drivetrain.setDefaultCommand(
      drivetrain
          .applyRequest(() -> drive.withVelocityX(-Constants.OIConstants.driverController.getLeftY() * SwerveConstants.MaxSpeed * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))
              .withVelocityY(-Constants.OIConstants.driverController.getLeftX() * SwerveConstants.MaxSpeed * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))
              .withRotationalRate(-Constants.OIConstants.driverController.getRightX() * SwerveConstants.MaxAngularRate * (drivetrain.isSlowMode() ? SwerveConstants.slowModeMultiplier : 1))));

    // Zero heading
    Constants.OIConstants.driverController.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Reef/Feeder align
    Constants.OIConstants.driverController.leftBumper().whileTrue(
      AutoCommands.autoScore()
    );

    // Algae align
    Constants.OIConstants.driverController.rightBumper().whileTrue(
      AutoCommands.alignAlgae()
    );

    // Slow mode
    Constants.OIConstants.driverController.leftTrigger().onTrue(Commands.runOnce(() -> drivetrain.setSlowMode(true)));
    Constants.OIConstants.driverController.leftTrigger().onFalse(Commands.runOnce(() -> drivetrain.setSlowMode(false)));

    // Outtake
    Constants.OIConstants.driverController.rightTrigger().whileTrue(
      Commands.startEnd(
        () -> intake.setIntakeSpeed(-1),
        () -> intake.setIntakeSpeed(0), 
        intake
      ).alongWith(Commands.runOnce(() -> intake.outtake())));           

    // Coral Setpoints
    Constants.OIConstants.operatorController.povUp().onTrue(
        Commands.either(
            Commands.runOnce(() -> OIConstants.autoScoreMode = 1), 
            AutoCommands.raiseL4(), 
            () -> vision.isVisionUpdating()));
    Constants.OIConstants.operatorController.povRight().onTrue(
        Commands.either(
            Commands.runOnce(() -> OIConstants.autoScoreMode = 0), 
            AutoCommands.raiseL3(), 
            () -> vision.isVisionUpdating()));
    Constants.OIConstants.operatorController.povLeft().onTrue(AutoCommands.raiseL2());
    Constants.OIConstants.operatorController.povDown().onTrue(AutoCommands.stow());

    // Algae setpoints
    Constants.OIConstants.operatorController.rightTrigger().onTrue(AutoCommands.AlgaeLow());
    Constants.OIConstants.operatorController.leftTrigger().onTrue(AutoCommands.algaeHigh());
    Constants.OIConstants.operatorController.x().onTrue((AutoCommands.processor()));

    // Reverse intake
    Constants.OIConstants.operatorController.y().whileTrue(
      Commands.startEnd(
        () -> intake.setIntakeSpeed(1),
        () ->  intake.setIntakeSpeed(0), 
        intake
      ));

    // Intake
    OIConstants.operatorController.a().onTrue(intake.intakeCommand());
      
    // Change reef scoring stem
    OIConstants.operatorController.leftBumper().onTrue(
        Commands.runOnce(() -> drivetrain.setScoringLeft()
      ));
    OIConstants.operatorController.rightBumper().onTrue(
        Commands.runOnce(() -> drivetrain.setScoringRight()
      ));

    // No vision toggle
    OIConstants.operatorController.x().onTrue(
      Commands.run(() -> vision.toggleShouldUpdate()).alongWith(
      new RainbowCommand(() -> 1))
      );

    // Seed arm
    OIConstants.driverController.povUp().onTrue(Commands.runOnce(() -> Arm.getInstance().seedRelativeEncoder()));

    // Vertical stow
    OIConstants.operatorController.b()
     .onTrue(AutoCommands.vstow());
     
    // Drive sysid
    Constants.OIConstants.driverController.povRight().and(Constants.OIConstants.driverController.y())
      .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    Constants.OIConstants.driverController.povRight().and(Constants.OIConstants.driverController.x())
      .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    Constants.OIConstants.driverController.povLeft().and(Constants.OIConstants.driverController.y())
      .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    Constants.OIConstants.driverController.povLeft().and(Constants.OIConstants.driverController.x())
      .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    //  return autoChooser.get();
     return AutoCommands.algaeHigh();
  }
}
