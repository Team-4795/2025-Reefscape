package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;


public class ClimberIOReal implements ClimberIO{
    private  SparkFlex climberMotor =  new SparkFlex(ClimberConstants.ClimberID, MotorType.kBrushless);
    private RelativeEncoder climberEncoder = climberMotor.getEncoder();
    private SparkFlexConfig config = new SparkFlexConfig();

    
    
public ClimberIOReal (){

    config.smartCurrentLimit(ClimberConstants.currentLimit);
    climberMotor.setCANTimeout(200);
    
    config.idleMode(IdleMode.kBrake);
    climberMotor.clearFaults();
    config.inverted(ClimberConstants.isInverted);

    config.encoder.positionConversionFactor(2*Math.PI / ClimberConstants.gearing);
    config.encoder.velocityConversionFactor(2*Math.PI/ 60.0 / ClimberConstants.gearing);

    config.voltageCompensation(ClimberConstants.voltageCompensation);

    climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

}

@Override
public double getPosition(){
    return climberEncoder.getPosition();
    }

@Override
public void setGoal(double angle){
    if (angle != climberEncoder.getPosition())
    climberEncoder.setPosition(angle);
    }

@Override
public void setClimberVoltage(double voltage){
    climberMotor.setVoltage(voltage);
    }

@Override
public void updateInputs(ClimberIOInputs inputs){
    inputs.voltage = climberMotor.getBusVoltage();
    inputs.position = getPosition();
    inputs.velocity = climberEncoder.getVelocity();
    inputs.current = climberMotor.getOutputCurrent();
    }

}



