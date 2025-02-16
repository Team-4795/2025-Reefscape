package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
    private ArmFeedforward ffmodel = new ArmFeedforward(ArmConstants.SIMkS, ArmConstants.SIMkG, ArmConstants.SIMkV, ArmConstants.SIMkA);
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(3, 10);
    private final PIDController controller = new PIDController(5, 0, 2);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(ArmConstants.Sim.INIT_ANGLE, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(ArmConstants.Sim.INIT_ANGLE, 0);
    private double voltage = 0;

    @Override
    public void setVoltage(double voltage) {
        armSim.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
        this.voltage = voltage;
    }

    @Override
    public void resetEncoder() {
        armSim.setState(0, armSim.getVelocityRadPerSec());
    }

    @Override
    public void setGoal(double angle) {
        if(angle != goal.position) {
            setpoint = new TrapezoidProfile.State(armSim.getAngleRads(), armSim.getVelocityRadPerSec());
            goal = new TrapezoidProfile.State(MathUtil.clamp(angle, ArmConstants.Sim.MIN_ANGLE,  ArmConstants.Sim.MAX_ANGLE), 0);
        } 
    }

    @Override
    public void hold() {
        double ffvolts = ffmodel.calculate(armSim.getAngleRads(), 0);
        double pidvolts = controller.calculate(armSim.getAngleRads(), goal.position);
        setVoltage(ffvolts + pidvolts);
    }

    @Override
    public void setFFValues(double kS, double kG, double kV, double kA) {
        ffmodel = new ArmFeedforward(kS, kG, kV, kA);
    }

    @Override
    public void updateMotionProfile() {
        setpoint = profile.calculate(0.02, setpoint, goal);
        setVoltage(ffmodel.calculate(armSim.getAngleRads(), setpoint.velocity) + controller.calculate(armSim.getAngleRads(), setpoint.position));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.angularPosition = armSim.getAngleRads();
        inputs.angularVelocity = armSim.getVelocityRadPerSec();
        inputs.current = armSim.getCurrentDrawAmps();
        inputs.voltage = voltage;
        inputs.goalAngle = goal.position;
        inputs.setpointVelocity = setpoint.velocity;
        inputs.setpointPosition = setpoint.position;
        armSim.update(0.02);
    }
}
