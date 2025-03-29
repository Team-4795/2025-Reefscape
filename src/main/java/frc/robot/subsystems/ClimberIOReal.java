package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.apriltag.AprilTagDetector.Config;

public class ClimberIOReal implements ClimberIO{
    private final  SparkFlex climberMotor =  new SparkFlex(ClimberConstants.ClimberID, MotorType.kBrushless);
    private RelativeEncoder climberEncoder = climberMotor.getEncoder();
    private SparkFlexConfig config = new SparkFlexConfig();

    
    
public ClimberIOReal (){
    config.smartCurrentLimit(ClimberConstants.currentLimit);
    config.idleMode(IdleMode.kBrake);
    climberMotor.clearFaults();
    config.encoder.positionConversionFactor(0);
    config.voltageCompensation(12);
    climberMotor.setCANTimeout(200);

}

}
