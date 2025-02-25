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
            arm.setGoalCommand(.8),
            Commands.waitUntil(() -> arm.getAngle() > -Math.PI/4)
                .andThen(elevator.setGoal(.57))
        ).until(() -> elevator.atGoal(.57) && arm.atGoal(.8));
    }

    public static Command raiseL3() {
        return Commands.parallel(
            Commands.waitSeconds(.3).andThen(Elevator.getInstance().setGoal(0)),
            Commands.runOnce(() -> Arm.getInstance().setGoal(ArmConstants.CORAL_L3), Arm.getInstance())
        );
    }

    public static Command raiseL2() {
        return Commands.parallel(
            Elevator.getInstance().setGoal(0.4211287200450897),
            Commands.runOnce(() -> Arm.getInstance().setGoal(ArmConstants.CORAL_L2), Arm.getInstance())
        );
    }


    public static Command stow() {
        return Commands.sequence(
            elevator.setGoal(ElevatorConstants.minDistance).withTimeout(1),
            arm.setGoalCommand(-Math.PI/2 - Units.degreesToRadians(17)).withTimeout(1.5),
            intake.intake()
        );
    }

    public static Command alignReef() {
        return new AutoAlignReef(
            new ProfiledPIDController(1, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)), 
            new ProfiledPIDController(1, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)));
    }

    public static Command alignFeeder() {
        return new AutoAlignFeeder(
            new ProfiledPIDController(1, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)), 
            new ProfiledPIDController(1, 0, 0, new Constraints(SwerveConstants.MaxSpeed, 3)));
    }
}