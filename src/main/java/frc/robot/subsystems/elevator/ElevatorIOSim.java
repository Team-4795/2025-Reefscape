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
    private ElevatorSim elevatorSim = new ElevatorSim(0, 0.1, DCMotor.getNeoVortex(2), 0, 0, false, 0);

    private double elevatorAppliedVolts = 0.0;


private final SimpleMotorFeedforward ffmodel = new SimpleMotorFeedforward(0, 0);
private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1);
private final PIDController controller = new PIDController(1, 0, 0);
private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();


@Override 
public void updateInputs(ElevatorIOInputs inputs) {

if(!inputs.openLoop){
     setVoltage(ffmodel.calculate(setpoint.velocity, setpoint.position) + controller.calculate(setpoint.velocity));
    setpoint = profile.calculate(0.1, setpoint, setpoint);
    }
    
    elevatorSim.update(0.02);
    inputs.elevatorPositionMetersPerSecond = elevatorSim.getPositionMeters();
    inputs.elevatorMotorPositionMeters = elevatorSim.getPositionMeters();
    inputs.elevatorMotorVelocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
    inputs.elevatorCurrent = elevatorSim.getCurrentDrawAmps();
    inputs.elevatorAppliedVolts = elevatorAppliedVolts;
}

@Override
public void moveElevator(double speed) {
    elevatorAppliedVolts = MathUtil.clamp(speed, 0, 0);
    elevatorSim.setInputVoltage(elevatorAppliedVolts);
}

@Override
public void setVoltage(double volts){
    elevatorAppliedVolts = volts;
    // Logger.recordOutput("Setting output", volts);
    elevatorSim.setInputVoltage(volts);
}
        
}