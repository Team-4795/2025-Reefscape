package frc.robot.commands;

import com.pathplanner.lib.path.ConstraintsZone;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;

public class AutoCommands {
    private static Swerve drive = Swerve.getInstance();
    private static Arm arm = Arm.getInstance();
    private static Elevator elevator = Elevator.getInstance();
    private static Intake intake = Intake.getInstance();

    public static Command raiseL4() {
        return Commands.parallel(
            Commands.runOnce(() -> arm.setGoal(ArmConstants.CORAL_L4)),
            Commands.waitUntil(() -> arm.getAngle() > -Math.PI/4)
                .andThen(Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.CORAL_L4_SETPOINT + Units.inchesToMeters(1))))
        );
    }

    public static Command vstow() {
        return Commands.sequence(
            Commands.runOnce(() -> arm.setGoal(ArmConstants.VSTOW)),
            Commands.waitSeconds(.25)
                .andThen(Commands.runOnce(() -> elevator.setGoalHeight(0)))
        );
    }

    public static Command raiseL3() {
        return Commands.either(
            Commands.parallel(
                arm.setGoalCommand(ArmConstants.CORAL_L3),
                Commands.waitUntil(() -> arm.getAngle() > -Math.PI/4)
                    .andThen(elevator.setGoal(0))
            ).until(() -> elevator.atGoal(0) && arm.atGoal(ArmConstants.CORAL_L3)),
            Commands.parallel(
                elevator.setGoal(0),
                Commands.waitUntil(() -> elevator.getPosition() < .2)
                    .andThen(arm.setGoalCommand(ArmConstants.CORAL_L3))
            ).until(() -> elevator.atGoal(0) && arm.atGoal(ArmConstants.CORAL_L3)),
            () -> 0 < elevator.getGoalHeight()
        );
    }

    
    public static Command noElevatorRaiseL3() {
        return arm.setGoalCommand(ArmConstants.CORAL_L3).until(() -> arm.atGoal(ArmConstants.CORAL_L3));
    }

    public static Command raiseL2() {
        return Commands.either(
            Commands.parallel(
                arm.setGoalCommand(ArmConstants.CORAL_L2),
                Commands.waitUntil(() -> arm.getAngle() > -Math.PI/4)
                    .andThen(elevator.setGoal(ElevatorConstants.CORAL_L2_SETPOINT))
            ).until(() -> elevator.atGoal(ElevatorConstants.CORAL_L2_SETPOINT) && arm.atGoal(ArmConstants.CORAL_L2)),
            Commands.parallel(
                elevator.setGoal(ElevatorConstants.CORAL_L2_SETPOINT),
                Commands.waitUntil(() -> elevator.getPosition() < .2)
                    .andThen(arm.setGoalCommand(ArmConstants.CORAL_L2))
            ).until(() -> elevator.atGoal(ElevatorConstants.CORAL_L2_SETPOINT) && arm.atGoal(ArmConstants.CORAL_L2)),
            () -> 0 < elevator.getGoalHeight()
        );
    }

    public static Command AlgaeLow() {
        return Commands.either(
            Commands.parallel(
                arm.setGoalCommand(ArmConstants.CORAL_L2),
                Commands.waitUntil(() -> arm.getAngle() > -Math.PI/4)
                    .andThen(elevator.setGoal(ElevatorConstants.ALGEA_SETPOINT))
            ).until(() -> elevator.atGoal(ElevatorConstants.ALGEA_SETPOINT) && arm.atGoal(ArmConstants.CORAL_L2)),
            Commands.parallel(
                elevator.setGoal(ElevatorConstants.ALGEA_SETPOINT),
                Commands.waitUntil(() -> elevator.getPosition() < .2)
                    .andThen(arm.setGoalCommand(ArmConstants.CORAL_L2))
            ).until(() -> elevator.atGoal(ElevatorConstants.ALGEA_SETPOINT) && arm.atGoal(ArmConstants.CORAL_L2)),
            () -> 0 < elevator.getGoalHeight()
        );
    }

    public static Command algaeHigh() {
        return Commands.either(
            Commands.parallel(
                arm.setGoalCommand(ArmConstants.ALGAE_HIGH),
                Commands.waitUntil(() -> arm.getAngle() > -Math.PI/4)
                    .andThen(elevator.setGoal(ElevatorConstants.CORAL_L2_SETPOINT))
            ).until(() -> elevator.atGoal(ElevatorConstants.CORAL_L2_SETPOINT) && arm.atGoal(ArmConstants.ALGAE_HIGH)),
            Commands.parallel(
                elevator.setGoal(ElevatorConstants.CORAL_L2_SETPOINT),
                Commands.waitUntil(() -> elevator.getPosition() < .2)
                    .andThen(arm.setGoalCommand(ArmConstants.ALGAE_HIGH))
            ).until(() -> elevator.atGoal(ElevatorConstants.CORAL_L2_SETPOINT) && arm.atGoal(ArmConstants.ALGAE_HIGH)),
            () -> 0 < elevator.getGoalHeight()
        );
    }

    
    public static Command autoStow() {
        return Commands.parallel(
            elevator.setGoal(0),
            Commands.waitUntil(() -> elevator.getPosition() < .2)
                .andThen(arm.setGoalCommand(ArmConstants.STOW))
        ).until(() -> elevator.atGoal(0) && arm.atGoal(ArmConstants.STOW))
        .andThen(
        Commands.runOnce(() -> intake.setIntakeSpeed(IntakeConstants.intake)));
    }

    public static Command setIntakeSpeed() {
        return Commands.runOnce(() -> intake.setIntakeSpeed(IntakeConstants.intake));
    }

    
    public static Command stow() {
        return Commands.parallel(
            Commands.runOnce(() -> elevator.setGoalHeight(ElevatorConstants.STOW)),
            Commands.waitUntil(() -> elevator.getPosition() < 0.2)
                .andThen(Commands.runOnce(() -> arm.setGoal(ArmConstants.STOW)))
        );
    }

    public static Command intake() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.setIntakeSpeed(IntakeConstants.intake)), 
            Commands.waitSeconds(0.3),
            Commands.waitUntil(() -> intake.GamePieceFinal()),
            Commands.waitSeconds(0.15),
            intake.reverse().withTimeout(0.12));
        //until(() -> elevator.atGoal(0) && arm.atGoal(ArmConstants.STOW));
        // .andThen(
        //     intake.intake().withTimeout(2));
    }

    public static Command score() {
        return intake.intake().withTimeout(0.2);
    }

    public static Command alignReef() {
        return new AutoAlignReef(
            new ProfiledPIDController(5,
             0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)), 
            new ProfiledPIDController(7.5, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3))
        ).withTimeout(2);
    }

    public static Command longerAlignReef() {
        return new AutoAlignReef(
            new ProfiledPIDController(5,
             0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)), 
            new ProfiledPIDController(7.5, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3))
        ).withTimeout(5);
    }

    public static Command alignReefUntil() {
        return new AutoAlignReef(
            new ProfiledPIDController(5,
             0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)), 
            new ProfiledPIDController(7.5, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3))
        ).until(() -> OIConstants.aligned);
    }

    public static Command scoreLeftReef() {
        return Commands.runOnce(() -> Swerve.getInstance().setScoringLeft());
    }

    public static Command scoreRightReef() {
        return Commands.runOnce(() -> Swerve.getInstance().setScoringRight());
    }

    public static Command alignFeeder() {
        return new AutoAlignFeeder
        (
            new ProfiledPIDController(1, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)), 
            new ProfiledPIDController(1, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)));
    }
}