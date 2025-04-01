package frc.robot.subsystems.state;

import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.Wrist.WristConstants;
import frc.robot.subsystems.state.State.Setpoint;

public class StateConstants {
    public static final Setpoint DYNAMIC = new Setpoint(null, null, null, null);
    public static final Setpoint DEFAULT = new Setpoint(null, null, null, null);
    public static final Setpoint L4 = new Setpoint(ArmConstants.CORAL_L4, ElevatorConstants.CORAL_L4_SETPOINT, 0.0, WristConstants.CORAL_L4_SETPOINT);
    public static final Setpoint L3 = new Setpoint(ArmConstants.CORAL_L3, ElevatorConstants.CORAL_L3_SETPOINT, 0.0, WristConstants.CORAL_L3_SETPOINT);
    public static final Setpoint L2 = new Setpoint(ArmConstants.CORAL_L2, ElevatorConstants.CORAL_L2_SETPOINT, 0.0, WristConstants.CORAL_L2_SETPOINT);
    public static final Setpoint HIGH_ALGAE = new Setpoint(ArmConstants.ALGAE_HIGH, ElevatorConstants.HIGH_ALGAE_SETPOINT, 1.0, WristConstants.CORAL_L4_SETPOINT);
    public static final Setpoint LOW_ALGAE = new Setpoint(ArmConstants.ALGAE_LOW, ElevatorConstants.STOW, 1.0, 0.025);
    public static final Setpoint VSTOW = new Setpoint(ArmConstants.VSTOW, ElevatorConstants.STOW, 0.0, null);
    public static final Setpoint STOW = new Setpoint(ArmConstants.STOW, ElevatorConstants.STOW, 0.0, WristConstants.stowPosition);
    public static final Setpoint PROCESSOR = new Setpoint(ArmConstants.PROCESSOR, ElevatorConstants.PROCESSOR_SETPOINT, IntakeConstants.coralReverse, null);
}