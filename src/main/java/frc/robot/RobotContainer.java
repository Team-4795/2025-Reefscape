// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.Constants.Gamepiece;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIORealVortex;
import frc.robot.subsystems.intake.IntakeIOSim;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOReal;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {  

  private Elevator elevator;
  private Drive drive;
  private Intake intake;

  public RobotContainer() throws IOException, ParseException {
    switch (Constants.currentMode) {
      case REAL:
        elevator = Elevator.initialize(new ElevatorIOReal());
        intake = Intake.initialize(new IntakeIORealVortex());
        Arm.initialize(new ArmIOReal());

        drive = new Drive(
            new GyroIOPigeon(),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));

        break;
      case SIM:
        elevator = Elevator.initialize(new ElevatorIOSim());
        intake = Intake.initialize(new IntakeIOSim());
        Arm.initialize(new ArmIOSim());

        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());

        break;

      default:

        elevator = Elevator.initialize(new ElevatorIOSim());
        intake = Intake.initialize(new IntakeIOSim());
        Arm.initialize(new ArmIOSim());

        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());

            
    }
    Arm.getInstance().setFFMode(Gamepiece.CORAL);
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> Constants.OIConstants.driverController.getLeftY(),
            () -> Constants.OIConstants.driverController.getLeftX(),
            () -> -Constants.OIConstants.driverController.getRightX()));

    Constants.OIConstants.driverController.a().whileTrue(Commands.runOnce(() -> drive.zeroHeading(), drive));

    // OIConstants.operatorController.leftTrigger()
    //     .whileTrue(Commands.startEnd(() -> elevator.moveElevator(OIConstants.operatorController.getLeftTriggerAxis() / 1.5), 
    //     () -> elevator.moveElevator(0), 
    //     elevator));

    // OIConstants.operatorController.rightTrigger()
    //     .whileTrue(Commands.startEnd(() -> elevator.moveElevator(-OIConstants.operatorController.getRightTriggerAxis() / 1.5),
    //     () -> elevator.moveElevator(0), 
    //     elevator));

    OIConstants.operatorController.rightBumper().whileTrue(
      Commands.run(
        () -> elevator.setVoltage(4), 
        elevator
      )
    );

    OIConstants.operatorController.leftBumper().whileTrue(
      Commands.run(
        () -> elevator.setVoltage(-4), 
        elevator
      )
    );


     Constants.OIConstants.operatorController.povUp()
     .whileTrue(
        Commands.sequence(
          intake.intake().until(() -> intake.GamePieceInitial()),
          intake.intakeSlow().until(() -> intake.GamePieceFinal()),
          Commands.run(() -> intake.setIntakeSpeed(0))
        )
     );

     Constants.OIConstants.operatorController.x().whileTrue(Commands.startEnd(() -> intake.setIntakeSpeed(-0.5),
     () -> intake.setIntakeSpeed(0), intake));

     Constants.OIConstants.operatorController.a().whileTrue(Commands.startEnd(() -> intake.setIntakeSpeed(0.5), 
     () -> intake.setIntakeSpeed(0), intake));

    //  OIConstants.driverController.povUp().whileTrue(
    //     Commands.startEnd(
    //       () -> Arm.getInstance().manualVoltage(9), 
    //       () -> Arm.getInstance().manualVoltage(ArmConstants.DEFAULTkG * Math.cos(Arm.getInstance().getAngle() - Math.PI/2)),
    //       Arm.getInstance()
    //     )
    //   );
      
      OIConstants.driverController.povDown().onTrue(
        new InstantCommand(() -> Arm.getInstance().setGoal(0)));

      OIConstants.driverController.povRight().onTrue(
        new InstantCommand(() -> Arm.getInstance().setGoal(0)));

      OIConstants.driverController.povUp().onTrue(
        new InstantCommand(() -> Arm.getInstance().setGoal(ArmConstants.CORAL_L3)));


      // OIConstants.driverController.leftTrigger().whileTrue(
      //   Commands.startEnd(
      //   () -> Arm.getInstance().manualVoltage(OIConstants.operatorController.getLeftTriggerAxis() * 6), 
      //   () -> Arm.getInstance().manualVoltage(0), 
      //   Arm.getInstance()
      // ));


      // OIConstants.driverController.rightTrigger().whileTrue(
      //   Commands.startEnd(
      //   () -> Arm.getInstance().manualVoltage(OIConstants.operatorController.getRightTriggerAxis() * -6), 
      //   () -> Arm.getInstance().manualVoltage(0), 
      //   Arm.getInstance()
      // ));

      OIConstants.driverController.leftTrigger().whileTrue(
        Commands.run(
          () -> Arm.getInstance().manualVoltage(OIConstants.operatorController.getLeftTriggerAxis() * 6), 
          Arm.getInstance()
        )
      );
      
      OIConstants.driverController.rightTrigger().whileTrue(
        Commands.run(
          () -> Arm.getInstance().manualVoltage(OIConstants.operatorController.getLeftTriggerAxis() * -6 + ArmConstants.DEFAULTkG), 
          Arm.getInstance()
        )
      );
  }

  public Command getAutonomousCommand() {
    return Commands.runOnce(() -> elevator.setVoltage(ElevatorConstants.kg));
  }
}
