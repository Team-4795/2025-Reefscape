package frc.robot.commands;

import java.lang.invoke.WrongMethodTypeException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.GenericRequirement;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.state.State;
import frc.robot.subsystems.state.StateManager;
import frc.robot.subsystems.state.StateManager.OperationStates;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.Util.LoggedTunableNumber;

public class AutoCommands {
    private static Swerve drive = Swerve.getInstance();
    private static Arm arm = Arm.getInstance();
    private static Elevator elevator = Elevator.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Wrist wrist = Wrist.getInstance();
    private static StateManager stateManager = StateManager.getInstance();
    

    private static LoggedTunableNumber maxAccel = new LoggedTunableNumber("AutoAlign/maxAccel", 3.2);

     //DO NOT MIND THIS FOR NOW   
    public static Command followTrajectory(PathPlannerPath PathName) {
        return AutoBuilder.followPath(PathName);
      }

    public static Command raiseL4() {
        Command command = Commands.sequence(
        Commands.runOnce(() -> wrist.setGoal(WristConstants.CORAL_L4_SETPOINT)),
        Commands.either(
        Commands.parallel(
        Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.CORAL_L4_SETPOINT)),
        Commands.waitUntil(() -> elevator.getPosition() > 0.4)
        .andThen(Commands.runOnce( () -> arm.setGoal(ArmConstants.CORAL_L4))
        )),
        Commands.parallel(
            Commands.runOnce(() -> arm.setGoal(ArmConstants.CORAL_L4)),
            
            Commands.sequence(Commands.waitUntil(() -> arm.getAngle() > -Math.PI/4),
            Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.CORAL_L4_SETPOINT)))), 
        () -> arm.getAngle() > ArmConstants.CORAL_L4));

        command.addRequirements(GenericRequirement.getInstance());

        return command;
    }

    public static Command vstow() {
        Command command = Commands.sequence(
            Commands.runOnce(() -> arm.setGoal(ArmConstants.VSTOW)),
            Commands.waitUntil( () -> arm.atGoal(ArmConstants.VSTOW)))
                .andThen(Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.STOW)));

        command.addRequirements(GenericRequirement.getInstance());

        return command;
    }

    public static Command raiseL3() {
        Command command = Commands.sequence(
        Commands.runOnce(() -> wrist.setGoal(WristConstants.CORAL_L3_SETPOINT)),
        Commands.either(
            Commands.sequence(
                Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.STOW)),
                Commands.runOnce(() -> arm.setGoal(ArmConstants.CORAL_L3))
            ),
            Commands.sequence(
                Commands.runOnce(() -> arm.setGoal(ArmConstants.CORAL_L3)),
                Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.STOW))
            ),
            () -> elevator.getPosition() >= ElevatorConstants.STOW
        ));

        command.addRequirements(GenericRequirement.getInstance());

        return command;
    }

    public static Command noElevatorRaiseL3() {
        return arm.setGoalCommand(ArmConstants.CORAL_L3).until(() -> arm.atGoal(ArmConstants.CORAL_L3));
    }
    
    public static Command raiseL2() {
        Command command = Commands.sequence(Commands.runOnce(() -> wrist.setGoal(WristConstants.CORAL_L2_SETPOINT)),
        Commands.either(
            Commands.sequence(
                Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.CORAL_L2_SETPOINT)),
                Commands.runOnce(() -> arm.setGoal(ArmConstants.CORAL_L2))
            ),
            Commands.sequence(
                Commands.runOnce(() -> arm.setGoal(ArmConstants.CORAL_L2)),
                Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.CORAL_L2_SETPOINT))
            ),
            () -> elevator.getPosition() >= ElevatorConstants.CORAL_L2_SETPOINT
        ));

        command.addRequirements(GenericRequirement.getInstance());

        return command;
    }

    public static Command AlgaeLow() {
        Command command = Commands.either(
            Commands.sequence(
                Commands.runOnce(() -> arm.setGoal(ArmConstants.ALGAE_LOW)),
                Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.STOW))
            ),
            Commands.sequence(
                Commands.runOnce( ()-> elevator.setGoalHeight(ElevatorConstants.STOW)),
                Commands.runOnce(() -> arm.setGoal(ArmConstants.ALGAE_LOW))),
            () -> ElevatorConstants.STOW <= elevator.getPosition()
        ).andThen(Commands.runOnce(
            () -> intake.setIntakeSpeed(1)
        ));

        command.addRequirements(GenericRequirement.getInstance());

        return command;
    }

    public static Command processor() {
        Command command = Commands.either(
            Commands.sequence(
                Commands.runOnce(() -> arm.setGoal(ArmConstants.CORAL_L2)),
                Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.PROCESSOR_SETPOINT))
            ),
            Commands.sequence(
                Commands.runOnce( ()-> elevator.setGoalHeight(ElevatorConstants.PROCESSOR_SETPOINT)),
                Commands.runOnce(() -> arm.setGoal(ArmConstants.CORAL_L2))),
            () -> ElevatorConstants.ALGEA_SETPOINT <= elevator.getPosition()
        ).alongWith(Commands.runOnce(() -> intake.setIntakeSpeed(1)));

        command.addRequirements(GenericRequirement.getInstance());

        return command;
    }

    public static Command algaeHigh() {
        Command command = Commands.either(
            Commands.sequence(
                Commands.runOnce(() -> arm.setGoal(ArmConstants.ALGAE_HIGH)),
            Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.HIGH_ALGAE_SETPOINT))
            ),
            Commands.sequence(
                Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.HIGH_ALGAE_SETPOINT)),
                Commands.runOnce(() -> arm.setGoal(ArmConstants.ALGAE_HIGH))
            ),
            () -> ElevatorConstants.HIGH_ALGAE_SETPOINT <= elevator.getPosition()
        ).alongWith(Commands.runOnce(
            () -> intake.setIntakeSpeed(1)
        ));

        command.addRequirements(GenericRequirement.getInstance());

        return command;
    }

    
    public static Command autoStow() {
        return Commands.parallel(
            Commands.runOnce(() -> elevator.setGoalHeight(0)),
            Commands.waitUntil(() -> elevator.getPosition() < .2)
                .andThen(Commands.runOnce(() -> arm.setGoal(ArmConstants.STOW)))    
                ).until(() -> elevator.atGoal(0) && arm.atGoal(ArmConstants.STOW))
        .alongWith(
        Commands.runOnce(() -> intake.setIntakeSpeed(IntakeConstants.intake)));
    }

        public static Command intakeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.setIntakeSpeed(IntakeConstants.intake)), 
            Commands.waitUntil(() -> intake.hasGamepiece()),
            Commands.waitUntil(() -> !intake.hasGamepiece()),
            Commands.parallel(
                Commands.startEnd(
                    () -> {
                        OIConstants.driverController.setRumble(RumbleType.kBothRumble, 0.6);
                        OIConstants.operatorController.setRumble(RumbleType.kBothRumble, 0.6);
                    },
                    () -> {
                        OIConstants.driverController.setRumble(RumbleType.kBothRumble, 0);
                        OIConstants.operatorController.setRumble(RumbleType.kBothRumble, 0);
                    }
                ),
                intake.reverseCoral()
            ).until(() ->intake.hasGamepiece()),
            Commands.runOnce(() -> intake.isStoring()),
            Commands.runOnce(() -> arm.setGoal(ArmConstants.STOW + 0.2)),
            Commands.runOnce(() -> wrist.setGoal(WristConstants.VFBAngle))
        );  
    }

    public static Command setIntakeSpeed() {
        return Commands.runOnce(() -> intake.setIntakeSpeed(IntakeConstants.intake));
    }

    
    public static Command stow() {
        Command command = Commands.sequence(Commands.runOnce(() -> wrist.setGoal(WristConstants.intakePosition)), 
        Commands.parallel(
            Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.STOW)),
            Commands.waitUntil(() -> elevator.getPosition() < 0.2)
                .andThen(Commands.runOnce(() -> arm.setGoal(ArmConstants.STOW))),
            Commands.runOnce(()-> wrist.setGoal(0)))
        );

        command.addRequirements(GenericRequirement.getInstance());

        return command;
    }

    public static Command intake() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.setIntakeSpeed(IntakeConstants.intake)), 
            Commands.waitSeconds(0.3),
            Commands.waitUntil(() -> intake.GamePieceFinal()),
            Commands.waitSeconds(0.15),
            intake.reverseCoral().withTimeout(0.12));
    }

    // private static HashMap<Integer, Command> autoScoreMap() {
    //     HashMap<Integer, Command> map = new HashMap<>();

    //     map.put(2, raiseL2());
    //     map.put(3, raiseL3());
    //     map.put(4, raiseL4());

    //     return map;
    // }

    public static Command scoreNetForward(){
        return Commands.sequence(
            Commands.runOnce(()-> elevator.setGoalHeight(ElevatorConstants.NET_SETPOINT)),
            Commands.runOnce(()-> arm.setGoal(ArmConstants.NET_SETPOINT)),
            Commands.runOnce(()-> wrist.setGoal(WristConstants.NET_SETPOINT))
        );
    }

    public static Command yeet(){
        return Commands.parallel(
            Commands.runOnce(()-> wrist.setGoal(WristConstants.VFBAngle)),
            Commands.waitUntil(()-> wrist.atGoal(WristConstants.VFBAngle))
                .andThen(() -> wrist.setGoal(WristConstants.NET_SETPOINT)),
            Commands.waitUntil(()-> wrist.getPosition() <= -1.55)
                .andThen(()-> intake.setIntakeSpeed(IntakeConstants.intake)));
    }

    public static Command scoreNetBakwards(){
        return Commands.sequence(
            Commands.runOnce(() -> arm.setGoal(ArmConstants.STOW + 0.4)),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.NET_SETPOINT)),
            Commands.waitUntil(()-> elevator.atGoal(0.4))
                .andThen(() -> arm.setGoal(ArmConstants.NET_SETPOINT)),
            Commands.waitUntil(() -> arm.atGoal(ArmConstants.NET_SETPOINT - Units.degreesToRadians(5)))
                .andThen(yeet()));
    }

    public static Command autoScore() {
        return Commands.either(
            Commands.sequence(
                Commands.parallel(
                    alignReefUntil(),
                    Commands.deferredProxy(() -> stateManager.stateCommand(OperationStates.autoScoreMode))
                ),
                //score(),
                Commands.runOnce(() -> OperationStates.aligned = false)
            ),

            Commands.sequence(
                Commands.parallel(
                    alignReefUntil(),
                    Commands.sequence(
                        vstow(),
                        Commands.waitUntil(() -> OperationStates.inScoringDistance),
                        Commands.deferredProxy(() -> stateManager.stateCommand(OperationStates.autoScoreMode))
                    )
                ),
                Commands.waitSeconds(0.6),
                //score(),
                vstow()), 
            () -> OperationStates.autoScoreMode != State.L4).finallyDo(() -> OperationStates.aligned = false);
    }

    public static Command autoAlgae() {
        return Commands.parallel(
            alignAlgae(),
            Commands.deferredProxy(() -> stateManager.stateCommand(OperationStates.autoAlgaeMode))
        ).finallyDo(() -> OperationStates.aligned = false);
    }

    public static Command zeroArm() {
        return Commands.parallel(
            Commands.startEnd(
            () -> arm.manualVoltage(-3),
            () -> arm.manualVoltage(0),
            arm
        ).withTimeout(1), 
        Commands.waitSeconds(0.9).andThen(Commands.runOnce(() -> arm.seedRelativeEncoder())));
    }

    public static Command score() {
        return intake.intake().withTimeout(0.2).alongWith(Commands.runOnce(() -> intake.outtake()));
    }


    public static Command setScoringState() {
        return Commands.runOnce(() -> OperationStates.autoScoreMode = State.L4);
    }

    public static Command alignAlgae() { 
        return new AutoAlignAlgae( 
            new ProfiledPIDController(6,
            0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)), 
            new ProfiledPIDController(7.5, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3))
       ).until(() -> OperationStates.aligned);
    }

    public static Command alignReefUntil() {
        return new AutoAlignReef(
            new ProfiledPIDController(5,
             0, 0, new Constraints(SwerveConstants.MaxSpeed, maxAccel.get())), 
            new ProfiledPIDController(7.5, 0, 0, new Constraints(SwerveConstants.MaxAngularRate, 3))
        ).until(() -> OperationStates.aligned);
    }

    public static Command scoreLeftReef() {
        return Commands.runOnce(() -> Swerve.getInstance().setScoringLeft());
    }

    public static Command scoreRightReef() {
        return Commands.runOnce(() -> Swerve.getInstance().setScoringRight());
    }
}