// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIORealVortex;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.vision.AprilTag.Vision;
import frc.robot.subsystems.vision.AprilTag.VisionIOReal;
import frc.robot.subsystems.vision.AprilTag.VisionIOSim;
import frc.robot.util.NamedCommandManager;

public class RobotContainer {
  private RobotVisualizer visualizer = new RobotVisualizer();
  private Vision vision;

  // private final Vision vision;
  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

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
        drivetrain = Swerve.initialize(new Swerve(TunerConstants.DrivetrainConstants, 50, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight));
        vision = Vision.initialize(
          new VisionIOReal(0)
          // ,new VisionIOReal(1),
          // new VisionIOReal(2),
          // new VisionIOReal(3)
        );
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
        break;
    }

    NamedCommandManager.registerNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser("Driver Forward Straight"));
    autoChooser.addOption("BottomBarge C BB BF", AutoBuilder.buildAuto("BottomBarge C BB BF"));
    autoChooser.addOption("raiseL4", AutoCommands.raiseL4());
    configureBindings();
    
  }


  private void configureBindings() {
    // Drive command
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(
        () -> drive.withVelocityX(drivetrain.processJoystickInput().getX() * SwerveConstants.MaxSpeed)
              .withVelocityY(drivetrain.processJoystickInput().getY() * SwerveConstants.MaxSpeed)
              .withRotationalRate(drivetrain.processJoystickInput().getRotation().getRotations() * SwerveConstants.MaxAngularRate)
      )
    );

    // Zero heading
    Constants.OIConstants.driverController.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Reef/Feeder align
    Constants.OIConstants.driverController.leftBumper().whileTrue(
        AutoCommands.alignReef()
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
      )
    );                                                          

    // Arm manual control
    Constants.OIConstants.operatorController.leftTrigger().whileTrue(
      Commands.run(() -> Elevator.getInstance().setVoltage(3), Elevator.getInstance())
    );

    Constants.OIConstants.operatorController.rightTrigger().whileTrue(
      Commands.run(() -> Elevator.getInstance().setVoltage(-3), Elevator.getInstance())
    );

    Constants.OIConstants.operatorController.axisGreaterThan(4, 0.1).whileTrue(
      Commands.run(() -> Arm.getInstance().manualVoltage(OIConstants.operatorController.getRightY() * 3), Arm.getInstance())
    );

    Constants.OIConstants.operatorController.axisLessThan(4, -0.1).whileTrue(
      Commands.run(() -> Arm.getInstance().manualVoltage(OIConstants.operatorController.getRightY() * 3), Arm.getInstance())
    );

    // Coral Setpoints
    Constants.OIConstants.operatorController.povUp().onTrue(AutoCommands.raiseL4());
    Constants.OIConstants.operatorController.povLeft().onTrue(AutoCommands.raiseL2());
    Constants.OIConstants.operatorController.povRight().onTrue(AutoCommands.raiseL3());
    Constants.OIConstants.operatorController.povDown().onTrue(AutoCommands.stow());
    // Constants.OIConstants.operatorController.povLeft().onTrue(AutoCommands.raiseL3());

    // Constants.OIConstants.operatorController.povRight().onTrue(AutoCommands.raiseL2());

    // arm setpoint testing
    // Constants.OIConstants.operatorController.povUp().onTrue(
    //   Arm.getInstance().setGoalCommand(.8)
    // );

    // Constants.OIConstants.operatorController.povDown().onTrue(
    //   Arm.getInstance().setGoalCommand(-1)
    // );

    // Constants.OIConstants.operatorController.povUp().onTrue(
    //   Elevator.getInstance().setGoal(.4)
    // );

    // Constants.OIConstants.operatorController.povDown().onTrue(
    //   Elevator.getInstance().setGoal(0)
    // );

    // Stow


    // Reverse intake
    // Constants.OIConstants.operatorController.a().whileTrue(
    //   Commands.startEnd(
    //     () -> intake.setIntakeSpeed(-1),
    //     () ->  intake.setIntakeSpeed(0), 
    //     intake
    //   )
    // );

    Constants.OIConstants.operatorController.y().whileTrue(
      Commands.startEnd(
        () -> intake.setIntakeSpeed(1),
        () ->  intake.setIntakeSpeed(0), 
        intake
      )
    );

    // OIConstants.operatorController.a().whileTrue(
    //   Commands.startEnd(
    //     () -> intake.setIntakeSpeed(-1),
    //     () -> intake.setIntakeSpeed(0), 
    //     intake
    //   )
    // );

    OIConstants.operatorController.a().whileTrue(intake.intakeCommand())
    .andThen(Commands.runOnce(() -> intake.setIntakeSpeed(0)), intake);

    OIConstants.operatorController.leftBumper()
        .onTrue(
          Commands.runOnce(() -> drivetrain.setScoringLeft()
        ));
    
    OIConstants.operatorController.rightBumper()
        .onTrue(
          Commands.runOnce(() -> drivetrain.setScoringRight()
        ));


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

    Constants.OIConstants.driverController.povRight().and(Constants.OIConstants.driverController.y())
      .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    Constants.OIConstants.driverController.povRight().and(Constants.OIConstants.driverController.x())
      .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    Constants.OIConstants.driverController.povLeft().and(Constants.OIConstants.driverController.y())
      .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    Constants.OIConstants.driverController.povLeft().and(Constants.OIConstants.driverController.x())
      .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

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
