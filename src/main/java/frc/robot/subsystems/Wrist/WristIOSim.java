package frc.robot.subsystems.Wrist;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim implements WristIO {
    // do this later
    public SingleJointedArmSim sim = new SingleJointedArmSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1),1, 1),
        DCMotor.getNeoVortex(1),
        WristConstants.simGearing, WristConstants.simLength, 0, 2, false, 0);
    private double updateVolts = 0;
    
    @Override
    public void setVoltage(double voltage){
        sim.setInputVoltage(voltage);
        updateVolts = voltage;
    }

    @Override
    public void updateInputs (WristIOInputs inputs){
        inputs.voltage = updateVolts;
        inputs.pos = sim.getAngleRads();
        inputs.velocity = sim.getVelocityRadPerSec();
    }
}
