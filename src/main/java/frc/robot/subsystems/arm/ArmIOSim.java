package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private SingleJointedArmSim armMotor;
    private double armAppliedVolts = 0.0;
    private ArmFeedforward ff = new ArmFeedforward(0.1, 0.1, 0.1);

    public ArmIOSim() {
        armMotor = new SingleJointedArmSim(
        null,
        0,
        0,
        0,
        0,
        0,
        false,
        0,
        null);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.angularPosition = armMotor.getAngleRads();
        inputs.angularVelocity = armMotor.getVelocityRadPerSec();
        inputs.current = armMotor.getCurrentDrawAmps();
        inputs.voltage = armAppliedVolts;
    }

    @Override
    public void setVoltage(double voltage) {
        armMotor.setInputVoltage(voltage);
    }

    @Override
    public void setVelocity(double velocity) {
        armMotor.setInputVoltage(ff.calculate(armMotor.getAngleRads(), velocity));
    }
}

