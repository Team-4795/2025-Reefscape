// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;

public class RobotContainer {

  Elevator elevator;

  private Drive drive;

  public RobotContainer() throws IOException, ParseException {
    switch (Constants.currentMode) {
      case REAL:
        elevator = Elevator.initialize(new ElevatorIOReal());

        drive = new Drive(
            new GyroIOPigeon(),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));

        break;
      case SIM:
        elevator = Elevator.initialize(new ElevatorIOSim());

        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());

        break;
      case REPLAY:
        elevator = Elevator.initialize(new ElevatorIOReal());
        break;
      default:

        elevator = Elevator.initialize(new ElevatorIOSim());

        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
    }

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

    CommandXboxController xboxController = new CommandXboxController(1);
    xboxController.leftTrigger()
        .whileTrue(Commands.run(() -> elevator.moveElevator(xboxController.getLeftTriggerAxis() / 2)));
    xboxController.rightTrigger()
        .whileTrue(Commands.run(() -> elevator.moveElevator(-xboxController.getRightTriggerAxis() / 2)));

  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
