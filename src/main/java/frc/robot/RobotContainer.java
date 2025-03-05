// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristIOReal;
import frc.robot.subsystems.Wrist.WristIOSim;

public class RobotContainer {
  private final Wrist wrist;
  
  private final CommandXboxController m_driverController =
    new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final CommandXboxController m_operatorController = new CommandXboxController(1);
  
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
    // do this later
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
