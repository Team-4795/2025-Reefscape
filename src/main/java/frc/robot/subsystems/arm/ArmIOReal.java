package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CurrentLimits;

public class ArmIOReal implements ArmIO {
    public final TalonFX armMotor = new TalonFX(ArmConstants.canID);

    public TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();


    public final StatusSignal<Current> current = armMotor.getStatorCurrent();
    public final StatusSignal<Voltage> voltage = armMotor.getMotorVoltage();
    public final StatusSignal<AngularVelocity> velocity = armMotor.getVelocity();

    public ArmIOReal(){
        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = CurrentLimits.armKraken;

        talonFXConfig.Audio.BeepOnBoot = true;

        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        armMotor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll (50,velocity, voltage, current );

        //random number for optimizedfreqhz
        armMotor.optimizeBusUtilization(9,1.0);

        StatusCode response = armMotor.getConfigurator().apply(talonFXConfig);
        if(!response.isOK()){
            System.out.println(
                "Talon ID"
                    + armMotor.getDeviceID()
                    + "failed config with error"
                    + response.toString());
        }
    }
    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocity, voltage, current);

        inputs.angularVelocity = velocity.getValueAsDouble()*60;
        inputs.current = current.getValueAsDouble();
        inputs.voltage = voltage.getValueAsDouble();
        inputs.angularPosition = armMotor.getPosition().getValueAsDouble()*2*Math.PI;
    }

    @Override
    public void setVelocity(double velocity) {
        armMotor.set(-velocity);
    }

    public void setVoltage(double voltage) {
        armMotor.set(-voltage);
    }

     
}
