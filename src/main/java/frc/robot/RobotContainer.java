// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristConstants;
import frc.robot.subsystems.Wrist.WristIOReal;
import frc.robot.subsystems.Wrist.WristIOSim;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.RainbowCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.GenericRequirement;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIORealVortex;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.state.State;
import frc.robot.subsystems.state.StateManager;
import frc.robot.subsystems.state.StateManager.OperationStates;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.vision.AprilTag.Vision;
import frc.robot.subsystems.vision.AprilTag.VisionIOReal;
import frc.robot.subsystems.vision.AprilTag.VisionIOSim;
import frc.robot.util.NamedCommandManager;

public class RobotContainer {
  private RobotVisualizer visualizer;
  private LEDs leds; 

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(SwerveConstants.MaxSpeed * 0.04).withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.04) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public final Telemetry logger = new Telemetry(SwerveConstants.MaxSpeed);

  private StateManager stateManager;
  public int autoScoreMode = 1;

  private Wrist wrist; 
  private Elevator elevator;
  private Intake intake;
  private Arm arm;
  public final Swerve drivetrain;
  private Vision vision;

  LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() throws IOException, ParseException {
    GenericRequirement.initialize();
    switch (Constants.currentMode) {
      case REAL:
        wrist = Wrist.initialize(new WristIOReal());
        elevator = Elevator.initialize(new ElevatorIOReal());
        intake = Intake.initialize(new IntakeIORealVortex());
        arm = Arm.initialize(new ArmIOReal());
        drivetrain = Swerve.initialize(new Swerve(TunerConstants.DrivetrainConstants, 50, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight));
        vision = Vision.initialize(
          new VisionIOReal(0), 
          new VisionIOReal(1)
        );
        break;

      case SIM:
        elevator = Elevator.initialize(new ElevatorIOSim());
        intake = Intake.initialize(new IntakeIOSim());
        Arm.initialize(new ArmIOSim());
        drivetrain = Swerve.initialize(TunerConstants.createDrivetrain());
        visualizer = new RobotVisualizer();
        vision = Vision.initialize(new VisionIOSim());
        wrist = Wrist.initialize(new WristIOSim());
        break;

      default:
        elevator = Elevator.initialize(new ElevatorIOSim());
        intake = Intake.initialize(new IntakeIOSim());
        drivetrain = Swerve.initialize(TunerConstants.createDrivetrain());
        Arm.initialize(new ArmIOSim());
        wrist = Wrist.initialize(new WristIOSim());
        break;
    }

    stateManager = StateManager.initalize();
    NamedCommandManager.registerNamedCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser("Driver Forward Straight"));
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
    
    // Bindings to score coral
    Constants.OIConstants.operatorController.povUp().onTrue(AutoCommands.scoreCoralL4());
    Constants.OIConstants.operatorController.povLeft().onTrue(AutoCommands.scoreCoralL2());
    Constants.OIConstants.operatorController.povRight().onTrue(AutoCommands.scoreCoralL3());

    // Set new goal of elevator
    Constants.OIConstants.operatorController.povUp().onTrue(Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.CORAL_L4_SETPOINT)));
    Constants.OIConstants.operatorController.povLeft().onTrue(Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.CORAL_L2_SETPOINT)));
    Constants.OIConstants.operatorController.povRight().onTrue(Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.CORAL_L3_SETPOINT)));
    Constants.OIConstants.operatorController.povDown().onTrue(Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.CORAL_L1_SETPOINT)));
    // Set new goal of elevator>
    // Send voltage to make elevator move up
    // Send voltage to make elevator move down
    Constants.OIConstants.operatorController.b().whileTrue(Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.STOW)));
    // Intake gamepiece at half speed
    Constants.OIConstants.operatorController.x().whileTrue(Commands.startEnd(()-> intake.setIntakeSpeed(0.5), ()-> intake.setIntakeSpeed(0), intake));
    // Outtake gamepiece at half speed
    Constants.OIConstants.operatorController.y().whileTrue(Commands.startEnd(()-> intake.setIntakeSpeed(-0.5), ()-> intake.setIntakeSpeed(0), intake));
    //move set wrist goal
    //move wrist in either direction

  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
