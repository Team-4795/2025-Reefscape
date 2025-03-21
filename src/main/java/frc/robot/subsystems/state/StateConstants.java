package frc.robot.subsystems.state;


import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.state.State.Setpoint;

public class StateConstants {
    public static final Setpoint DYNAMIC = new Setpoint(null, null, null);
    public static final Setpoint DEFAULT = new Setpoint(null, null, null);
    public static final Setpoint L4 = new Setpoint(ArmConstants.CORAL_L4, ElevatorConstants.CORAL_L4_SETPOINT, 0.0);
    public static final Setpoint L3 = new Setpoint(ArmConstants.CORAL_L3, ElevatorConstants.STOW, 0.0);
    public static final Setpoint L2 = new Setpoint(ArmConstants.CORAL_L2, ElevatorConstants.CORAL_L2_SETPOINT, 0.0);
    public static final Setpoint HIGH_ALGAE = new Setpoint(ArmConstants.ALGAE_HIGH, ElevatorConstants.HIGH_ALGAE_SETPOINT, 1.0);
    public static final Setpoint LOW_ALGAE = new Setpoint(ArmConstants.ALGAE_LOW, ElevatorConstants.STOW, 1.0);
    public static final Setpoint VSTOW = new Setpoint(ArmConstants.VSTOW, ElevatorConstants.STOW, 0.0);
    public static final Setpoint STOW = new Setpoint(ArmConstants.STOW, ElevatorConstants.STOW, 0.0);

    public static final double MIN_AXIS_HEIGHT = ElevatorConstants.maxDistance/.9;

    public static final Map<State, Translation3d> VFB_TARGETS = Map.ofEntries(
        Map.entry(State.L4_DYNAMIC, new Translation3d(ArmConstants.Sim.LENGTH + 0.1, 0, 1.5))
    );
}
