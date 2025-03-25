// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristConstants;
import frc.robot.subsystems.Wrist.WristIOReal;
import frc.robot.subsystems.Wrist.WristIOSim;

public class RobotContainer {
  private final Wrist wrist;
  
  public RobotContainer() {
    
    switch (Constants.currentMode) {
      case REAL:
        wrist = Wrist.initialize(new WristIOReal());
        break;
      case SIM:
        wrist = Wrist.initialize(new WristIOSim());
        break;
      default:
        wrist = Wrist.initialize(new WristIOSim());
  
    }
    configureBindings();
  }

  private void configureBindings() {
    // placeholder wrist
  
    Constants.OIConstants.operatorController.povUp().onTrue(
      Commands.runOnce( 
        ()-> wrist.setGoal(WristConstants.VFBAngle), wrist));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
