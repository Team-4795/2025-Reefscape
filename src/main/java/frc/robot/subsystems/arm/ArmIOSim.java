package frc.robot.subsystems.arm;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getNeoVortex(2),
        ArmConstants.Sim.GEARING,
        ArmConstants.Sim.MOI,
        ArmConstants.Sim.LENGTH,
        ArmConstants.Sim.MIN_ANGLE,
        ArmConstants.Sim.MAX_ANGLE,
        ArmConstants.Sim.GRAVITY,
        ArmConstants.Sim.INIT_ANGLE
    );
    private final ArmFeedforward ffmodel = new ArmFeedforward(
        ArmConstants.kS,
        ArmConstants.kG,
        ArmConstants.kV,
        ArmConstants.kA
    );
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0.5, 1);
    private final PIDController controller = new PIDController(0.02, 0,0.00);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(ArmConstants.Sim.INIT_ANGLE, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private double voltage = 0;

    @Override
    public void setVoltage(double voltage) {
        armSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
        this.voltage = voltage;
    }

    @Override
    public void setGoal(double angle) {
        goal = new TrapezoidProfile.State(angle, 0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if(!inputs.openLoop) {
            setpoint = profile.calculate(0.02, setpoint, goal);
            setVoltage(ffmodel.calculate(inputs.angularPosition, setpoint.velocity));
        }

        inputs.angularPosition = armSim.getAngleRads();
        inputs.angularVelocity = armSim.getVelocityRadPerSec();
        inputs.current = armSim.getCurrentDrawAmps();
        inputs.voltage = voltage;
        inputs.goalAngle = goal.position;
        inputs.setpointVelocity = setpoint.velocity;
        armSim.update(0.02);
    }
}
