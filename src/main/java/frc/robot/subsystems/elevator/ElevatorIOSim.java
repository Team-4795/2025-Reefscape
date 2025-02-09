package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim elevatorSim = new ElevatorSim(ElevatorConstants.kv, ElevatorConstants.ks, DCMotor.getNeoVortex(2), 0, 0.7112, true, 0);

    private double elevatorAppliedVolts = 0.0;


private final ElevatorFeedforward ffmodel = new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg - .3, ElevatorConstants.kv);
private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1);
private final PIDController controller = new PIDController(1, 0, 0);
private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
private TrapezoidProfile.State goal = new TrapezoidProfile.State();

@Override 
public void updateInputs(ElevatorIOInputs inputs) {

if(!inputs.openLoop){
     setVoltage(ffmodel.calculate(setpoint.velocity, setpoint.position) + controller.calculate(setpoint.velocity));
    setpoint = profile.calculate(0.02, setpoint, goal);
    }
    
    inputs.elevatorLeftPositionMeters = elevatorSim.getPositionMeters();
    inputs.elevatorLeftVelocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
    inputs.elevatorLeftCurrent = elevatorSim.getCurrentDrawAmps();
    inputs.elevatorLeftAppliedVolts = elevatorAppliedVolts;
    elevatorSim.update(0.02);
}

@Override
public void moveElevator(double speed) {
    elevatorAppliedVolts = 12 * speed;
    elevatorSim.setInputVoltage(elevatorAppliedVolts);
}

@Override
public void hold() {
    double ffvolts = ffmodel.calculate(0);
    setVoltage(ffvolts);
}

@Override
public void setVoltage(double volts){
    elevatorAppliedVolts = volts;
    // Logger.recordOutput("Setting output", volts);
    elevatorSim.setInputVoltage(volts);
}

@Override
public void setGoal(double height) {
    goal = new TrapezoidProfile.State(height, 0);
        
}

    }