package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.GenericRequirement;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.util.LoggedTunableNumber;

public class AutoCommands {
    private static Swerve drive = Swerve.getInstance();
    private static Arm arm = Arm.getInstance();
    private static Elevator elevator = Elevator.getInstance();
    private static Intake intake = Intake.getInstance();

    private static LoggedTunableNumber maxAccel = new LoggedTunableNumber("AutoAlign/maxAccel", 3.2);

    public static Command raiseL4() {
        Command command = Commands.either(
        Commands.parallel(
        Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.CORAL_L4_SETPOINT)),
        Commands.waitUntil(() -> elevator.getPosition() > 0.4)
        .andThen(Commands.runOnce( () -> arm.setGoal(ArmConstants.CORAL_L4))
        )),
        Commands.parallel(
            Commands.runOnce(() -> arm.setGoal(ArmConstants.CORAL_L4)),
            Commands.sequence(Commands.waitUntil(() -> arm.getAngle() > -Math.PI/4),
            Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.CORAL_L4_SETPOINT)))), 
        () -> arm.getAngle() > ArmConstants.CORAL_L4);

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
        Command command = Commands.either(
            Commands.sequence(
                Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.STOW)),
                Commands.runOnce(() -> arm.setGoal(ArmConstants.CORAL_L3))
            ),
            Commands.sequence(
                Commands.runOnce(() -> arm.setGoal(ArmConstants.CORAL_L3)),
                Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.STOW))
            ),
            () -> elevator.getPosition() >= ElevatorConstants.STOW
        );

        command.addRequirements(GenericRequirement.getInstance());

        return command;
    }

    public static Command noElevatorRaiseL3() {
        return arm.setGoalCommand(ArmConstants.CORAL_L3).until(() -> arm.atGoal(ArmConstants.CORAL_L3));
    }
    
    public static Command raiseL2() {
        Command command = Commands.either(
            Commands.sequence(
                Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.CORAL_L2_SETPOINT)),
                Commands.runOnce(() -> arm.setGoal(ArmConstants.CORAL_L2))
            ),
            Commands.sequence(
                Commands.runOnce(() -> arm.setGoal(ArmConstants.CORAL_L2)),
                Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.CORAL_L2_SETPOINT))
            ),
            () -> elevator.getPosition() >= ElevatorConstants.CORAL_L2_SETPOINT
        );

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

    public static Command setIntakeSpeed() {
        return Commands.runOnce(() -> intake.setIntakeSpeed(IntakeConstants.intake));
    }

    
    public static Command stow() {
        Command command = Commands.parallel(
            Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.STOW)),
            Commands.waitUntil(() -> elevator.getPosition() < 0.2)
                .andThen(Commands.runOnce(() -> arm.setGoal(ArmConstants.STOW)))
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
            intake.reverse().withTimeout(0.12));
    }

    public static HashMap<Integer, Command> autoScoreMap() {
        HashMap<Integer, Command> map = new HashMap<>();

        map.put(2, raiseL2());
        map.put(3, raiseL3());
        map.put(4, raiseL4());

        return map;
    }

    public static Command autoScore() {
        return Commands.either(
            Commands.sequence(
                Commands.parallel(
                    alignReefUntil(),
                    Commands.select(autoScoreMap(), () -> OIConstants.autoScoreMode)),
                score(),
                Commands.runOnce(() -> OIConstants.aligned = false)), 

            Commands.sequence(
                Commands.parallel(
                    alignReefUntil(),
                    Commands.sequence(
                        vstow(),
                        Commands.waitUntil(() -> OIConstants.inScoringDistance),
                        Commands.select(autoScoreMap(), () -> OIConstants.autoScoreMode))
                    ),
                Commands.waitSeconds(0.2),
                score(),
                vstow()), 
                
            () -> OIConstants.autoScoreMode != 4).finallyDo(() -> OIConstants.aligned = false);
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

    public static Command oneCoralAway() {
        Command command = Commands.sequence(
                Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.ONE_CORAL_AWAY)), 
                Commands.runOnce(() -> arm.setGoal(ArmConstants.ONE_CORAL_AWAY)), 
                Commands.waitUntil(() -> arm.atGoal(ArmConstants.ONE_CORAL_AWAY) && elevator.atGoal(ElevatorConstants.ONE_CORAL_AWAY)),
                score(),
                vstow()
        );

            command.addRequirements(GenericRequirement.getInstance());

            return command; 
        }

    public static Command setScoringState() {
        return Commands.runOnce(() -> OIConstants.autoScoreMode = 4);
    }

    public static Command alignAlgae() { 
        return new AutoAlignAlgae( 
            new ProfiledPIDController(5,
            0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)), 
            new ProfiledPIDController(7.5, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3))
       );
    }

    public static Command alignReefUntil() {
        return new AutoAlignReef(
            new ProfiledPIDController(5,
             0, 0, new Constraints(SwerveConstants.MaxSpeed, maxAccel.get())), 
            new ProfiledPIDController(7.5, 0, 0, new Constraints(SwerveConstants.MaxAngularRate, 3))
        ).until(() -> OIConstants.aligned);
    }

    public static Command scoreLeftReef() {
        return Commands.runOnce(() -> Swerve.getInstance().setScoringLeft());
    }

    public static Command scoreRightReef() {
        return Commands.runOnce(() -> Swerve.getInstance().setScoringRight());
    }
}