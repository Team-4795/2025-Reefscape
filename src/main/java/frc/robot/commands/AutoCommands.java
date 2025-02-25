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
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;

public class AutoCommands {
    private static Swerve drive = Swerve.getInstance();
    private static Arm arm = Arm.getInstance();
    private static Elevator elevator = Elevator.getInstance();
    private static Intake intake = Intake.getInstance();

    public static Command raiseL4() {
        return Commands.parallel(
            arm.setGoalCommand(ArmConstants.CORAL_L4),
            Commands.waitUntil(() -> arm.getAngle() > -Math.PI/4)
                .andThen(elevator.setGoal(ElevatorConstants.CORAL_L4_SETPOINT))
        ).withTimeout(1);
        //until(() -> elevator.atGoal(ElevatorConstants.CORAL_L4_SETPOINT) && arm.atGoal(ArmConstants.CORAL_L4));
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


    public static Command stow() {
        return Commands.parallel(
            elevator.setGoal(0),
            Commands.waitUntil(() -> elevator.getPosition() < .2)
                .andThen(arm.setGoalCommand(ArmConstants.STOW))
        ).withTimeout(1);
        //until(() -> elevator.atGoal(0) && arm.atGoal(ArmConstants.STOW));
        // .andThen(
        //     intake.intake().withTimeout(2));
    }

    public static Command score() {
        return intake.intake().withTimeout(1);
    }

    public static Command alignReef() {
        return new AutoAlignReef(
            new ProfiledPIDController(5, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)), 
            new ProfiledPIDController(5, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)));
    }

    public static Command alignFeeder() {
        return new AutoAlignFeeder(
            new ProfiledPIDController(1, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)), 
            new ProfiledPIDController(1, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)));
    }
}