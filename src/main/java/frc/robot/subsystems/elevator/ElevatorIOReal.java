package frc.robot.subsystems.elevator;

import com.revrobotics.AbsoluteEncoder;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.measure.AngularVelocity;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.CurrentLimits;


import edu.wpi.first.units.measure.Current;


public class ElevatorIOReal implements ElevatorIO {

    private SparkFlex rightElevatorMotor = new SparkFlex(1, MotorType.kBrushless);
    private SparkFlex leftElevatorMotor = new SparkFlex(2, MotorType.kBrushless); 


    leftElevatorMotor.setSmartCurrentLimit(CurrentLimits.elevator);
    rightElevatorMotor.setSmartCurrentLimit(CurrentLimits.elevator);

  
    private double inputVolts = 0.0;

        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

       
        private boolean isEnabled;
        private boolean hasPlayed = false;


        private SparkFlexConfig config(double kV) {
        var SparkFlexConfig = new SparkFlexConfig();

        return SparkFlexConfig;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
    }


    public ElevatorIOReal(){
    
        leftElevatorMotor.setControl(new Follower(rightElevatorMotor.getDeviceID(), true));


    }



}