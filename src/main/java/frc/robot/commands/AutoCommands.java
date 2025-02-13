package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class AutoCommands {
    private static Arm arm = Arm.getInstance();
    private static Elevator elevator = Elevator.getInstance();
    private static Intake intake = Intake.getInstance();

    public static Command raiseL4() {
        return Commands.sequence(
            Commands.parallel(
                Commands.runOnce(() -> Arm.getInstance().setGoal(1.02), Arm.getInstance()),
                Elevator.getInstance().setGoal(0.57)
            )
        );
    }
}
