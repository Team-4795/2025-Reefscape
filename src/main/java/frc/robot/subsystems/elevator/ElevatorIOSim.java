package frc.robot.subsystems.elevator;

import java.lang.System.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;


public class ElevatorIOSim implements ElevatorIO {
    private ElevatorSim elevatorSim = new ElevatorSim(0, 0, null, 0, 0, false, 0, null);

    private double elevatorAppliedVolts = 0.0;


@Override 
public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(0.02);
    inputs.elevatorPositionMetersPerSecond = elevatorSim.getPositionMeters();
    inputs.elevatorMotorPositionMeters = elevatorSim.getPositionMeters();
    inputs.elevatorMotorVelocityMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();
    inputs.elevatorCurrent = elevatorSim.getCurrentDrawAmps();
    inputs.elevatorAppliedVolts = elevatorAppliedVolts;
}

@Override
public void moveElevator(double speed) {
    elevatorAppliedVolts = MathUtil.clamp(speed, -1, 1);
    elevatorSim.setInputVoltage(elevatorAppliedVolts);
}

@Override
public void setVoltage(double volts){
    elevatorAppliedVolts = volts;
    // Logger.recordOutput("Setting output", volts);
    elevatorSim.setInputVoltage(volts);
}
        
}