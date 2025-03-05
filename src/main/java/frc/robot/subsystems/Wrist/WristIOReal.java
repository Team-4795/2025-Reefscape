package frc.robot.subsystems.Wrist;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;


public class WristIOReal implements WristIO{
    // neo vortex motor
    private SparkFlex wristMotor = new SparkFlex(WristConstants.id, MotorType.kBrushless);
    private SparkFlexConfig config = new SparkFlexConfig();
    private RelativeEncoder encoder = wristMotor.getEncoder();

    public WristIOReal() {
        config.smartCurrentLimit(WristConstants.stallLimit, WristConstants.freeLimit);
        wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs (WristIOInputs inputs){
        inputs.voltage = wristMotor.getBusVoltage();
        inputs.pos = encoder.getPosition();
        inputs.velocity = encoder.getVelocity();
    }
}
