// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmConstants;

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
      OIConstants.driverController.povUp().whileTrue(
        Commands.startEnd(
          () -> Arm.getInstance().manualVoltage(6), 
          () -> Arm.getInstance().manualVoltage(0),
          Arm.getInstance()
        )
      );
      
      OIConstants.driverController.povDown().whileTrue(
        Commands.startEnd(
          () -> Arm.getInstance().manualVoltage(-6), 
          () -> Arm.getInstance().manualVoltage(0),
          Arm.getInstance()
        )
      );
  }

  public Command getAutonomousCommand() {
    return Commands.runOnce(() -> Arm.getInstance().manualVoltage(ArmConstants.DEFAULTkG));
  }
}
