// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;

public class RobotContainer {

  Elevator elevator;

  public RobotContainer() {
    switch (Constants.currentMode){
      case REAL:
      elevator = Elevator.initialize(new ElevatorIOReal());
      break;
      case SIM:
      elevator = Elevator.initialize(new ElevatorIOSim());
      break;
      case REPLAY:
      elevator = Elevator.initialize(new ElevatorIOReal());
      break;
      default: 
    }
    configureBindings();
  }


  private void configureBindings() {
    CommandXboxController xboxController = new CommandXboxController(1);
    xboxController.leftTrigger().whileTrue(Commands.run(() -> elevator.moveElevator(xboxController.getLeftTriggerAxis()/2)));
    xboxController.rightTrigger().whileTrue(Commands.run(() -> elevator.moveElevator(-xboxController.getRightTriggerAxis()/2)));


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
