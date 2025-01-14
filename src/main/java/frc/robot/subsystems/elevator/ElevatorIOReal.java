package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;


public class ElevatorIOReal implements ElevatorIO {

    private TalonFX rightElevatorMotor = new TalonFX(0);
    private TalonFX leftElevatorMotor = new TalonFX(1); 

    private double inputVolts = 0.0;

        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);



    public ElevatorIOReal(){
        leftElevatorMotor.setControl(new Follower(rightElevatorMotor.getDeviceID(), true));


    }



}