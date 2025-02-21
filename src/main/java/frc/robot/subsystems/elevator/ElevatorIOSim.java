package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim elevatorSim = new ElevatorSim(ElevatorConstants.kv, ElevatorConstants.ks, DCMotor.getNeoVortex(2), 0, 0.7112, true, 0);
    private double elevatorAppliedVolts = 0.0;

    private final ElevatorFeedforward ffmodel = new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg - .1, ElevatorConstants.kv);
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2, 4);
    private final PIDController controller = new PIDController(50, 0, 1);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

@Override 
public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(0.02);
    inputs.elevatorLeftPositionMeters = elevatorSim.getPositionMeters();
    inputs.elevatorLeftVelocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
    inputs.elevatorRightPositionMeters = elevatorSim.getPositionMeters();
    inputs.elevatorRightVelocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
    inputs.elevatorLeftCurrent = elevatorSim.getCurrentDrawAmps();
    inputs.elevatorLeftAppliedVolts = elevatorAppliedVolts;
    inputs.goalHeight = goal.position;
    inputs.setpointVelocity = setpoint.velocity;
    inputs.setpointPosition = setpoint.position;
}

@Override
public void moveElevator(double speed) {
    elevatorAppliedVolts = 12 * speed;
    elevatorSim.setInputVoltage(elevatorAppliedVolts);
}

@Override
public void updateMotionProfile() {
    setVoltage(ffmodel.calculate(setpoint.velocity) + controller.calculate(elevatorSim.getPositionMeters(), setpoint.position));
    setpoint = profile.calculate(0.02, setpoint, goal);
}

@Override
public void hold() {
    double ffvolts = ffmodel.calculate(0);
    double pidvolts = controller.calculate(elevatorSim.getPositionMeters(), setpoint.position);
    setVoltage(pidvolts + ffvolts);
    // updateMotionProfile();
}

@Override
public void setVoltage(double volts){
    elevatorAppliedVolts = MathUtil.clamp(volts, -12, 12);
    // Logger.recordOutput("Setting output", volts);
    elevatorSim.setInputVoltage(elevatorAppliedVolts);
}

@Override
public void setGoal(double height) {
    goal = new TrapezoidProfile.State(height, 0);
        
}

    }