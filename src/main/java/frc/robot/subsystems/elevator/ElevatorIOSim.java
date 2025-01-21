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
    elevatorSim.update(ElevatorConstants.kDt);
    inputs.elevatorPositionRads = elevatorSim.getAngleRads();
    inputs.elevatorMotorPositionRads = elevatorSim.getAngleRads();
    inputs.elevatorMotorVelocityRadPerSec = elevatorSim.getVelocityRadPerSec();
    inputs.elevatorCurrent = elevatorSim.getCurrentDrawAmps();
    inputs.elevatorAppliedVolts = elevatorAppliedVolts;

@Override
public void moveElevator(double speed) {
    ElevatorAppliedVolts = MathUtil.clamp(speed, speed, speed);
    elevatorSim.setInputVoltage(elevatorAppliedVolts)
  
}

@Override
public void setVoltage(double volts){
    ElevatorAppliedVolts = volts;
    Logger.recordOutput("Setting output", volts);
        elevatorSim.setInputVoltage(elevatorAppliedVolts - (voltage, voltage, voltage));
}
        
    }
}