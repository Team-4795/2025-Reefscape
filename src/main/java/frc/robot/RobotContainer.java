// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
public class RobotContainer {
  

  // private final Vision vision;
  /* Setting up bindings for necessary control of the swerve drive platform */
  LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() throws IOException, ParseException {
    
    switch (Constants.currentMode) {
      case REAL:
        
        break;

      case SIM:
        
        break;

      default:
        break;
    }
  }
  

 
  private void configureBindings() {

 }

  

  public Command getAutonomousCommand() {
     return autoChooser.get();
  }

  public void periodic() {

}
  }

