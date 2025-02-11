// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

public class RobotContainer {
  
  private  Drive drive;
  LoggedDashboardChooser<Command> autoChooser;
   
    public RobotContainer() throws IOException, ParseException {
  
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
       
      drive =
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
        new ModuleIOSim(), 
        new ModuleIOSim(), 
        new ModuleIOSim(), 
        new ModuleIOSim());
    
        break;
    }
    
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser("BTopBarge DFDB"));
    autoChooser.addOption("BottomBarge C BB BF", AutoBuilder.buildAuto("BottomBarge C BB BF"));

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
      DriveCommands.joystickDrive(
        drive,
        () -> Constants.OIConstants.driverController.getLeftY(),
        () -> Constants.OIConstants.driverController.getLeftX(),
        () -> -Constants.OIConstants.driverController.getRightX() 
        ));

    Constants.OIConstants.driverController.a().whileTrue(Commands.runOnce(()-> drive.zeroHeading(), drive));

      }

  

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
