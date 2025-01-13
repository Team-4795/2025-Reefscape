package frc.robot.subsystems.intake;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class IntakeIOSim implements IntakeIO {
  // not done
  private final DCMotorSim motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.5, 0.1), null, 0.2);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    motor.update(0.02);

    inputs.angularPositionRot = motor.getAngularPositionRotations();
    inputs.angularVelocityRPM = motor.getAngularVelocityRPM();
    inputs.currentAmps = motor.getCurrentDrawAmps();
  }

  @Override
  public void setMotorSpeed(double speed) {
    motor.setInputVoltage(MathUtil.clamp(12 * speed, -12, 12));
  }
}