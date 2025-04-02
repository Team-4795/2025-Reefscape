package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimberIOSim implements ClimberIO {
    public SingleJointedArmSim climberSim = new SingleJointedArmSim(
        DCMotor.getNeoVortex(1), 
        ClimberConstants.gearing,
        1,
         ClimberConstants.SimConstants.length, 
         ClimberConstants.SimConstants.minAngle, 
         ClimberConstants.SimConstants.maxAngle, 
         ClimberConstants.SimConstants.gravity, 
         ClimberConstants.SimConstants.initAngle
         );

    @Override
    public void setClimberVoltage(double voltage){
        climberSim.setInputVoltage(voltage);
    }


}
