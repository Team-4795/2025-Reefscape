// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

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
import frc.robot.subsystems.CANCoderTest;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOReal;

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
        CANCoderTest canCoderTest = new CANCoderTest();

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

    // OIConstants.operatorController.leftTrigger()
    //     .whileTrue(Commands.startEnd(() -> elevator.moveElevator(OIConstants.operatorController.getLeftTriggerAxis() / 1.5), 
    //     () -> elevator.moveElevator(0), 
    //     elevator));

    // OIConstants.operatorController.rightTrigger()
    //     .whileTrue(Commands.startEnd(() -> elevator.moveElevator(-OIConstants.operatorController.getRightTriggerAxis() / 1.5),
    //     () -> elevator.moveElevator(0), 
    //     elevator));

    OIConstants.operatorController.rightBumper().whileTrue(
      Commands.run(
        () -> elevator.setVoltage(4), 
        elevator
      )
    );

    OIConstants.operatorController.leftBumper().whileTrue(
      Commands.run(
        () -> elevator.setVoltage(-4), 
        elevator
      )
    );

    Constants.OIConstants.operatorController.povRight().onTrue(
      Commands.parallel(
        Arm.getInstance().setGoal(ArmConstants.CORAL_L4),
        Elevator.getInstance().setGoal(ElevatorConstants.maxDistance)
      )
    );

    Constants.OIConstants.operatorController.povLeft().onTrue(
      Commands.parallel(
        Arm.getInstance().setGoal(ArmConstants.Sim.INIT_ANGLE),
        Elevator.getInstance().setGoal(ElevatorConstants.minDistance)
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

     Constants.OIConstants.operatorController.x().whileTrue(Commands.startEnd(() -> intake.setIntakeSpeed(-0.5),
     () -> intake.setIntakeSpeed(0), intake));

     Constants.OIConstants.operatorController.a().whileTrue(Commands.startEnd(() -> intake.setIntakeSpeed(0.5), 
     () -> intake.setIntakeSpeed(0), intake));

    //  OIConstants.driverController.povUp().whileTrue(
    //     Commands.startEnd(
    //       () -> Arm.getInstance().manualVoltage(9), 
    //       () -> Arm.getInstance().manualVoltage(ArmConstants.DEFAULTkG * Math.cos(Arm.getInstance().getAngle() - Math.PI/2)),
    //       Arm.getInstance()
    //     )
    //   );
      
      OIConstants.driverController.povDown().onTrue((Arm.getInstance().setGoal(0)));
      OIConstants.driverController.povRight().onTrue((Arm.getInstance().setGoal(ArmConstants.CORAL_L2)));
      OIConstants.driverController.povUp().onTrue((Arm.getInstance().setGoal(ArmConstants.CORAL_L3)));


      // OIConstants.operatorController.povDown().onTrue((elevator.setGoal(0.05)));
      // OIConstants.operatorController.povRight().onTrue((elevator.setGoal(0.4)));
      // OIConstants.operatorController.povUp().onTrue((elevator.setGoal(0.69)));


      // OIConstants.driverController.leftTrigger().whileTrue(
      //   Commands.startEnd(
      //   () -> Arm.getInstance().manualVoltage(OIConstants.operatorController.getLeftTriggerAxis() * 6), 
      //   () -> Arm.getInstance().manualVoltage(0), 
      //   Arm.getInstance()
      // ));


      // OIConstants.driverController.rightTrigger().whileTrue(
      //   Commands.startEnd(
      //   () -> Arm.getInstance().manualVoltage(OIConstants.operatorController.getRightTriggerAxis() * -6), 
      //   () -> Arm.getInstance().manualVoltage(0), 
      //   Arm.getInstance()
      // ));
      OIConstants.driverController.y().whileTrue(
        Commands.run(
          () -> Arm.getInstance().manualVoltage( 6), 
          Arm.getInstance()
        )
      );
      
      // up
      OIConstants.driverController.a().whileTrue(
        Commands.run(
          () -> Arm.getInstance().manualVoltage(-6), 
          Arm.getInstance()
        )
      );

      OIConstants.driverController.leftBumper().onTrue(Arm.getInstance().setGoal(0));
      OIConstants.driverController.rightBumper().onTrue(Arm.getInstance().setGoal(ArmConstants.CORAL_L2));
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
    );
  }
}
