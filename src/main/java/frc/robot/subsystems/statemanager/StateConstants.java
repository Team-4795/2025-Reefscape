package frc.robot.subsystems.statemanager;

import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.statemanager.StateManagerV2.StateRequest;

public class StateConstants {
    public static StateRequest L4 = new StateRequest()
        .withArmAngle(ArmConstants.CORAL_L4)
        .withElevatorHeight(ElevatorConstants.CORAL_L4_SETPOINT);

    public static StateRequest L3 = new StateRequest()
        .withArmAngle(ArmConstants.CORAL_L3)
        .withElevatorHeight(ElevatorConstants.STOW);

    public static StateRequest L2 = new StateRequest()
        .withArmAngle(ArmConstants.CORAL_L2)
        .withElevatorHeight(ElevatorConstants.CORAL_L2_SETPOINT);

    public static StateRequest HIGH_ALGAE = new StateRequest()
        .withArmAngle(ArmConstants.ALGAE_HIGH)
        .withElevatorHeight(ElevatorConstants.HIGH_ALGAE_SETPOINT);
        
    public static StateRequest LOW_ALGAE = new StateRequest()
        .withArmAngle(ArmConstants.ALGAE_LOW)
        .withElevatorHeight(ElevatorConstants.STOW);

    public static StateRequest PROCESSOR = new StateRequest()
        .withArmAngle(ArmConstants.PROCESSOR)
        .withElevatorHeight(ElevatorConstants.PROCESSOR_SETPOINT);

    public static StateRequest STOW = new StateRequest()
        .withArmAngle(ArmConstants.STOW)
        .withElevatorHeight(ElevatorConstants.STOW);

    public static StateRequest Intake = new StateRequest()
        .withArmAngle(ArmConstants.STOW)
        .withElevatorHeight(ElevatorConstants.STOW)
        .withIntakeSpeed(IntakeConstants.intake);

    public static StateRequest VSTOW = new StateRequest()
        .withArmAngle(ArmConstants.VSTOW)
        .withElevatorHeight(ElevatorConstants.STOW);
}
