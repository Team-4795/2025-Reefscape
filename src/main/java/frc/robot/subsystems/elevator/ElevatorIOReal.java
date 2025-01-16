package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.measure.AngularVelocity;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Current;


public class ElevatorIOReal implements ElevatorIO {

    private SparkFlex rightElevatorMotor = new SparkFlex(1, MotorType.kBrushless);
    private SparkFlex leftElevatorMotor = new SparkFlex(2, MotorType.kBrushless); 
  
    private double inputVolts = 0.0;

        final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

        // private final StatusSignal<AngularVelocity> topRPM = leftElevatorMotor.getVelocity(); 
        // private final StatusSignal<AngularVelocity> bottomRPM = rightElevatorMotor.getVelocity();
        // private final StatusSignal<Current> topCurrent = leftElevatorMotor.getTorqueCurrent();
        // private final StatusSignal<Current> bottomCurrent = rightElevatorMotor.getTorqueCurrent();

        private boolean isEnabled;
        private boolean hasPlayed = false;


        private SparkFlexConfig config(double kV) {
        var SparkFlexConfig = new SparkFlexConfig();

        // SparkFlexConfig.Slot0.kP = ElevatorConstants.kP;
        // SparkFlexConfig.Slot0.kI = 0;
        // SparkFlexConfig.Slot0.kD = 0;
        // SparkFlexConfig.Slot0.kS = 0;
        // SparkFlexConfig.Slot0.kV = kV;

        // SparkFlexConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // SparkFlexConfig.CurrentLimits.StatorCurrentLimit = CurrentLimits.shooter;

        // SparkFlexConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // SparkFlexConfig.Audio.BeepOnBoot = true;

        return SparkFlexConfig;
    }


    public ElevatorIOReal(){
        //leftElevatorMotor.setControl(new Follower(rightElevatorMotor.getDeviceID(), true));


    }



}