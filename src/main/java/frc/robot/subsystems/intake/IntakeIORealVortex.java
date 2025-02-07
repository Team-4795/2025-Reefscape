package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class IntakeIORealVortex implements IntakeIO {
    private final SparkFlex intakeMotor = new SparkFlex(IntakeConstants.canID, MotorType.kBrushless);
    private final RelativeEncoder encoder = intakeMotor.getEncoder();
    
    private SparkFlexConfig config = new SparkFlexConfig();

    public IntakeIORealVortex() {
        intakeMotor.clearFaults();
        config.smartCurrentLimit(IntakeConstants.currentLimit);
        config.idleMode(IdleMode.kCoast);

        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.angularVelocityRPM = encoder.getVelocity();
        inputs.angularPositionRot = encoder.getPosition();
        inputs.currentAmps = intakeMotor.getOutputCurrent();
        inputs.voltage = intakeMotor.getBusVoltage();
    }

    @Override
    public boolean hasGamepiece() {
        return intakeMotor.getOutputCurrent() > IntakeConstants.currentThreshold;
    }

    @Override
    public void setMotorSpeed(double speed) {
        intakeMotor.set(-speed);
    }

}

