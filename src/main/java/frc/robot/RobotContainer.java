// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;

public class RobotContainer {  
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        Arm.initialize(new ArmIOReal());
        break;
      case SIM:
        Arm.initialize(new ArmIOSim());
        break;
      default:  
        Arm.initialize(new ArmIOReal());
        break;

    }
    configureBindings();
  }

  private void configureBindings() {
    if(OIConstants.driverController.isConnected()) {
      OIConstants.driverController.povUp().onTrue(Arm.getInstance().setGoal(Math.PI / 2));
      OIConstants.driverController.povDown().onTrue(Arm.getInstance().setGoal(0));
    }
  }

  public Command getAutonomousCommand() {
    return Arm.getInstance().setGoal(Math.PI / 2);
  }
}
