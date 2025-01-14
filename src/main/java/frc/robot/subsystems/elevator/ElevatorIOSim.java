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

    private double ElevatorAppliedVolts = 0.0;



@Override
public void moveElevator(double speed) {
    ElevatorAppliedVolts = MathUtil.clamp(speed, speed, speed);
  
}

@Override
public void setVoltage(double volts){
    ElevatorAppliedVolts = volts;
}
        
}