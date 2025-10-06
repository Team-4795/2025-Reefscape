package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIORealVortex implements IntakeIO {
    private final SparkFlex intakeMotor = new SparkFlex(IntakeConstants.canID, MotorType.kBrushless);
    private final RelativeEncoder encoder = intakeMotor.getEncoder();
    private final DigitalInput coralSensor = new DigitalInput(IntakeConstants.sensorChannel);
    
    private SparkFlexConfig config = new SparkFlexConfig();

    public IntakeIORealVortex() {
        intakeMotor.clearFaults();
        config.smartCurrentLimit(IntakeConstants.currentLimit);
        config.idleMode(IdleMode.kCoast);
        config.inverted(true);
        config.absoluteEncoder.positionConversionFactor(2 * Math.PI);
        config.absoluteEncoder.velocityConversionFactor(2 * Math.PI / 60);
        intakeMotor.setCANTimeout(20);
        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.angularVelocityRPM = encoder.getVelocity();
        inputs.angularPositionRot = encoder.getPosition();
        inputs.currentAmps = intakeMotor.getOutputCurrent();
        inputs.voltage = intakeMotor.getBusVoltage();
    }

    // Write method that spins intake (parameter should be between -1 and 1)
    @Override
    public void setMotorSpeed(double speed) {
        
    }

    @Override
    public SparkAbsoluteEncoder getArmAbsoluteEncoder() {
        return intakeMotor.getAbsoluteEncoder();
    }  

    @Override
    public boolean velocitySensing(double RPM) {
        return encoder.getVelocity() < RPM; 
    }

    @Override 
    public boolean hasGamepiece() {
        return !coralSensor.get(); 
    }
}

