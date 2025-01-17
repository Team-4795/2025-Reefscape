package frc.robot.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    //change gearbox argument
    private DCMotorSim driveSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(4),  0.025, DriveConstants.DriveGearing), DCMotor.getNeoVortex(4)); //needs to change
    private DCMotorSim turnSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(4), 0.004, DriveConstants.TurnGearing), DCMotor.getKrakenX60(4)); //needs to change

    private final Rotation2d turnAbsoluteInitPositon = new Rotation2d(Math.random()*2.0*Math.PI);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    @Override
    public void updateInputs(ModuleIOInputs inputs){
        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);
    }
}
