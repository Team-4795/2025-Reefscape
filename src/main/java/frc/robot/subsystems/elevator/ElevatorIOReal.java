package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.measure.AngularVelocity;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class ElevatorIOReal implements ElevatorIO {

    private SparkFlex rightElevatorMotor = new SparkFlex(1, MotorType.kBrushless);
    private SparkFlex leftElevatorMotor = new SparkFlex(2, MotorType.kBrushless); 

    private SparkFlexConfig config= new SparkFlexConfig();
     
    private double inputVolts = 0.0;

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private boolean isEnabled;
    private boolean hasPlayed = false;
    
    
    public ElevatorIOReal(){
    
    config.smartCurrentLimit(ElevatorConstants.elevatorCurrentLimits);

    leftElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    leftElevatorMotor.clearFaults();
    rightElevatorMotor.clearFaults();
}

    

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        
    }
}