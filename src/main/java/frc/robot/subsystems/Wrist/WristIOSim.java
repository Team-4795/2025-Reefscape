package frc.robot.subsystems.Wrist;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim implements WristIO {
    // do this later
    public SingleJointedArmSim wristSim = new SingleJointedArmSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1),1, 1),
        DCMotor.getNeoVortex(1),
        WristConstants.gearing, 
        WristConstants.Sim.length, 
        WristConstants.Sim.minAngle, 
        WristConstants.Sim.maxAngle, 
        WristConstants.Sim.gravity, 
        WristConstants.Sim.initAngle
    );
      private ArmFeedforward ffmodel = new ArmFeedforward(WristConstants.Sim.SIMkS, WristConstants.Sim.SIMkG, WristConstants.Sim.SIMkV, WristConstants.Sim.SIMKA);
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(3, 10);
    private final PIDController controller = new PIDController(5, 0, 2);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(WristConstants.Sim.INIT_ANGLE, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(WristConstants.Sim.INIT_ANGLE, 0);
    private double voltage = 0;
    
    private double updateVolts = 0;
    
    @Override
    public void setVoltage(double voltage){
        wristSim.setInputVoltage(voltage);
        this.updateVolts = voltage;
    }

     @Override
    public void setGoal(double angle){
        if (angle != goal.position){
        setpoint = new TrapezoidProfile.State(getPosition(), getVelocity());
        goal = new TrapezoidProfile.State(MathUtil.clamp(angle, WristConstants.minPosition, WristConstants.maxPosition), WristConstants.maxV);
        }
    }
    //   @Override 
    // public void moveUp (double voltage){
    //     wristMotor.setVoltage(voltage);
    // }

    // @Override
    // public void moveDown (double voltage){
    //     wristMotor.setVoltage(voltage);
    // }

    @Override
    public void updateMotionProfile(){
        setpoint = profile.calculate(0.02, setpoint, goal);
        double pidVolts = controller.calculate(getPosition(), setpoint.position);
        Logger.recordOutput("pid volts", pidVolts);
        double ffVolts = setpoint.velocity * WristConstants.kV;
        Logger.recordOutput("ff volts", ffVolts);
        setVoltage(ffVolts + pidVolts);
    }

    @Override
    public void updateInputs (WristIOInputs inputs){
        inputs.voltage = updateVolts;
        inputs.pos = wristSim.getAngleRads();
        inputs.velocity = wristSim.getVelocityRadPerSec();
    }
}
