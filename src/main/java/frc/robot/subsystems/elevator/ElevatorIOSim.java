package frc.robot.subsystems.elevator;

import java.lang.System.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim elevatorSim = new ElevatorSim(ElevatorConstants.kv, 1, DCMotor.getNeoVortex(2), 0, 0.7112, true, 0);

    private double elevatorAppliedVolts = 0.0;


private final SimpleMotorFeedforward ffmodel = new SimpleMotorFeedforward(ElevatorConstants.ks, ElevatorConstants.kv);
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
    inputs.elevatorRightAppliedVolts = elevatorAppliedVolts;
    elevatorSim.update(0.02);
}

@Override
public void moveElevator(double speed) {
    elevatorAppliedVolts = 12 * speed;
    elevatorSim.setInputVoltage(elevatorAppliedVolts);
}

@Override
public void setVoltage(double volts){
    elevatorAppliedVolts = volts;
    // Logger.recordOutput("Setting output", volts);
    elevatorSim.setInputVoltage(volts);
}

@Override
public void setGoal(double height) {
    goal = new TrapezoidProfile.State(height, 0.1);
        
}

    }