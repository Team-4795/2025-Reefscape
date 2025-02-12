// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.Constants.Gamepiece;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIORealVortex;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOReal;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {  

  private Elevator elevator;
  //private Drive drive;
  private Intake intake;

  public RobotContainer() throws IOException, ParseException {
    switch (Constants.currentMode) {
      case REAL:
        elevator = Elevator.initialize(new ElevatorIOReal());
        intake = Intake.initialize(new IntakeIORealVortex());
        Arm.initialize(new ArmIOReal());

        // drive = new Drive(
        //     new GyroIOPigeon(),
        //     new ModuleIOTalonFX(0),
        //     new ModuleIOTalonFX(1),
        //     new ModuleIOTalonFX(2),
        //     new ModuleIOTalonFX(3));

        break;
      case SIM:
        elevator = Elevator.initialize(new ElevatorIOSim());
        intake = Intake.initialize(new IntakeIOSim());
        Arm.initialize(new ArmIOSim());

        // drive = new Drive(
        //     new GyroIO() {
        //     },
        //     new ModuleIOSim(),
        //     new ModuleIOSim(),
        //     new ModuleIOSim(),
        //     new ModuleIOSim());

        break;

      default:

        elevator = Elevator.initialize(new ElevatorIOSim());
        intake = Intake.initialize(new IntakeIOSim());
        Arm.initialize(new ArmIOSim());

        // drive = new Drive(
        //     new GyroIO() {
        //     },
        //     new ModuleIOSim(),
        //     new ModuleIOSim(),
        //     new ModuleIOSim(),
        //     new ModuleIOSim());

            
    }
    RobotVisualizer robotVisualizer = new RobotVisualizer();
    configureBindings();
  }

  private void configureBindings() {
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> Constants.OIConstants.driverController.getLeftY(),
    //         () -> Constants.OIConstants.driverController.getLeftX(),
    //         () -> -Constants.OIConstants.driverController.getRightX()));

    //Constants.OIConstants.driverController.a().whileTrue(Commands.runOnce(() -> drive.zeroHeading(), drive));

    OIConstants.operatorController.leftTrigger()
        .whileTrue(Commands.startEnd(() -> elevator.moveElevator(OIConstants.operatorController.getLeftTriggerAxis() / 1.5), 
        () -> elevator.moveElevator(0), 
        elevator));

    OIConstants.operatorController.rightTrigger()
        .whileTrue(Commands.startEnd(() -> elevator.moveElevator(-OIConstants.operatorController.getRightTriggerAxis() / 1.5),
        () -> elevator.moveElevator(0), 
        elevator));

    // OIConstants.operatorController.rightBumper().whileTrue(
    //   Commands.run(
    //     () -> elevator.setVoltage(1), 
    //     elevator
    //   )
    // );

    // OIConstants.operatorController.leftBumper().whileTrue(
    //   Commands.run(
    //     () -> elevator.setVoltage(-1), 
    //     elevator
    //   )
    // );


    // OIConstants.operatorController.leftBumper().whileTrue(
    //   Arm.getInstance().sysIDRoutine().quasistatic(SysIdRoutine.Direction.kForward)
    // );

    // OIConstants.operatorController.rightBumper().whileTrue(
    //   Arm.getInstance().sysIDRoutine().quasistatic(SysIdRoutine.Direction.kReverse)
    // );

    // OIConstants.operatorController.leftTrigger().whileTrue(
    //   Arm.getInstance().sysIDRoutine().dynamic(SysIdRoutine.Direction.kForward)
    // );

    // OIConstants.operatorController.rightTrigger().whileTrue(
    //   Arm.getInstance().sysIDRoutine().dynamic(SysIdRoutine.Direction.kReverse)
    // );
    

    Constants.OIConstants.operatorController.povRight().onTrue(
      Commands.sequence(
        Commands.parallel(
          Commands.runOnce(() -> Arm.getInstance().setGoal(1.02)),
          Elevator.getInstance().setGoal(0.57)
        )
      )
    );

    Constants.OIConstants.operatorController.povLeft().onTrue(
      Commands.sequence(
        Elevator.getInstance().setGoal(ElevatorConstants.minDistance),
        Commands.runOnce(() -> Arm.getInstance().setGoal(-1.8)),
        Commands.waitSeconds(1.5)
      )
    );


     Constants.OIConstants.operatorController.povUp()
     .whileTrue(
        Commands.sequence(
          intake.intake().until(() -> intake.GamePieceInitial()),
          intake.intakeSlow().until(() -> intake.GamePieceFinal()),
          Commands.run(() -> intake.setIntakeSpeed(0))
        )
     );

     Constants.OIConstants.operatorController.x().whileTrue(Commands.startEnd(() -> intake.setIntakeSpeed(-1),
     () -> intake.setIntakeSpeed(0), intake));

     Constants.OIConstants.operatorController.b().whileTrue(Commands.startEnd(() -> intake.setIntakeSpeed(1), 
     () -> intake.setIntakeSpeed(0), intake));

     Constants.OIConstants.operatorController.y().onTrue(
      elevator.setGoal(ElevatorConstants.maxDistance/2)
     );

     Constants.OIConstants.operatorController.a().onTrue(
      elevator.setGoal(0)
     );
      
      // OIConstants.driverController.povDown().onTrue((Arm.getInstance().setGoal(0)));
      // OIConstants.driverController.povRight().onTrue((Arm.getInstance().setGoal(ArmConstants.CORAL_L2)));
      OIConstants.driverController.povUp().onTrue(Commands.runOnce(() -> Arm.getInstance().setGoal(ArmConstants.CORAL_L3)));


      // OIConstants.operatorController.povDown().onTrue((elevator.setGoal(0.05)));
      // OIConstants.operatorController.povRight().onTrue((elevator.setGoal(0.4)));
      // OIConstants.operatorController.povUp().onTrue((elevator.setGoal(0.69)));

      OIConstants.driverController.leftBumper().onTrue(Commands.runOnce(() -> Arm.getInstance().setGoal(-1.8)));
      OIConstants.driverController.rightBumper().onTrue(Commands.runOnce(() -> Arm.getInstance().setGoal(ArmConstants.CORAL_L2)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("hi");
  }
}
