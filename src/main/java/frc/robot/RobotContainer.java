// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;

public class RobotContainer {
  private final Intake intake;

  public RobotContainer() {
    configureBindings();
    switch (Constants.currentMode) {
      case REAL:
        intake = Intake.initialize(new IntakeIOSim());
        break;
      case SIM:
        intake = Intake.initialize(new IntakeIOSim());
        break;
      default:
        intake = Intake.initialize(new IntakeIOSim());
  
    }
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
