package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularMomentum;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(2),
        ArmConstants.Sim.GEARING,
        ArmConstants.Sim.MOI,
        ArmConstants.Sim.LENGTH,
        ArmConstants.Sim.MIN_ANGLE,
        ArmConstants.Sim.MAX_ANGLE,
        ArmConstants.Sim.GRAVITY,
        ArmConstants.Sim.INIT_ANGLE
    );
    private double voltage = 0;
    private final ArmFeedforward ffmodel = new ArmFeedforward(0.1, 0.1, 0.1);
    private final PIDController controller = new PIDController(0.01, 0.02, 0);
    private double goal = ArmConstants.Sim.INIT_ANGLE;

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
        armSim.setInputVoltage(voltage);
    }

    @Override 
    public void setVelocity(double velocity) {
        setVoltage(ffmodel.calculate(armSim.getAngleRads(), velocity));
    }

    @Override
    public void setGoal(double angle) {
        goal = angle;

    }

    @Override
    public void updateLoop() {
        setVoltage(controller.calculate(armSim.getAngleRads(), goal));
    }


    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.angularPosition = armSim.getAngleRads();
        inputs.angularVelocity = armSim.getVelocityRadPerSec();
        inputs.current = armSim.getCurrentDrawAmps();
        inputs.voltage = voltage;
        inputs.goal = goal;
        armSim.update(0.02);
    }
}
