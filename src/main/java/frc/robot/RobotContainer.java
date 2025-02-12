// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
   private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


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

  private void configureBindings() {{
      // Note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setDefaultCommand(
          // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() ->
              drive.withVelocityX(-Constants.OIConstants.driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(-Constants.OIConstants.driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(-Constants.OIConstants.driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
          )
      );
  
      Constants.OIConstants.driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
      Constants.OIConstants.driverController.b().whileTrue(drivetrain.applyRequest(() ->
          point.withModuleDirection(new Rotation2d(-Constants.OIConstants.driverController.getLeftY(), -Constants.OIConstants.driverController.getLeftX()))
      ));
  
      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.
      Constants.OIConstants.driverController.back().and(Constants.OIConstants.driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      Constants.OIConstants.driverController.back().and(Constants.OIConstants.driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      Constants.OIConstants.driverController.start().and(Constants.OIConstants.driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      Constants.OIConstants.driverController.start().and(Constants.OIConstants.driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  
      // reset the field-centric heading on left bumper press
      Constants.OIConstants.driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
  
      drivetrain.registerTelemetry(logger::telemeterize);
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
          Commands.runOnce(() -> Arm.getInstance().setGoal(1.02), Arm.getInstance()),
          Elevator.getInstance().setGoal(0.57)
        )
      )
    );

    Constants.OIConstants.operatorController.povLeft().onTrue(
      Commands.sequence(
        Elevator.getInstance().setGoal(ElevatorConstants.minDistance),
        Commands.runOnce(() -> Arm.getInstance().setGoal(-1.8), Arm.getInstance()),
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
}

  public Command getAutonomousCommand() {
    return Commands.print("hi");
  }
}
