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
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.vision.AprilTag.*;;

public class RobotContainer {
  private Drive drive;
  private Vision vision;

  public RobotContainer() throws IOException, ParseException {
  
      switch (Constants.currentMode) {
      case REAL:
      vision = Vision.initialize(new VisionIOReal[0]);
      drive = 
          new Drive(
            new GyroIOPigeon(), 
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1), 
            new ModuleIOTalonFX(2), 
            new ModuleIOTalonFX(3));
          break;
  
      case SIM:
      vision = Vision.initialize(new VisionIOSim());
        drive = Drive.initialize(
            new GyroIO() {},
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
            break;
      
      default:
      vision = Vision.initialize(new VisionIOSim());
      drive =
      new Drive(
        new GyroIO() {}, 
        new ModuleIOSim(), 
        new ModuleIOSim(), 
        new ModuleIOSim(), 
        new ModuleIOSim());
    
        break;
    }

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

    Constants.OIConstants.driverController.b().whileTrue(DriveCommands.driveToBestReefPos(drive));

      }

  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
