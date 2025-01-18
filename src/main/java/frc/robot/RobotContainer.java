// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive.Drive.Drive;
import frc.robot.subsystems.Drive.Drive.GyroIO;
import frc.robot.subsystems.Drive.Drive.GyroIOPigeon;
import frc.robot.subsystems.Drive.Drive.ModuleIO;
import frc.robot.subsystems.Drive.Drive.ModuleIOSim;
import frc.robot.subsystems.Drive.Drive.ModuleIOTalonFX;

public class RobotContainer {
  
  private final Drive drive;
   
    public RobotContainer() {
  
      switch (Constants.currentMode) {
        case REAL:
          
        drive = 
          new Drive(
            new GyroIOPigeon(), 
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1), 
            new ModuleIOTalonFX(2), 
            new ModuleIOTalonFX(3));
          break;
  
      case SIM:
       
      // drive =
        new Drive(
          new GyroIO() {}, 
          new ModuleIOSim(), 
          new ModuleIOSim(), 
          new ModuleIOSim(), 
          new ModuleIOSim());
      
        default:
        
        drive =
        new Drive(
          new GyroIO() {}, 
          new ModuleIO() {}, 
          new ModuleIO(){}, 
          new ModuleIO() {}, 
          new ModuleIO() {});
        break;
    }


    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
      DriveCommands.joystickDrive(
        drive,
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
