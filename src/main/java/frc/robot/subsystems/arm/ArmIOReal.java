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
import frc.robot.util.KrakenLogger;

public class ArmIOReal implements ArmIO {
    private final TalonFX armMotor = new TalonFX(ArmConstants.canID);

    private TalonFXConfiguration talonnFXConfig = new TalonFXConfiguration();


    private final StatusSignal<Current> current = armMotor.getStatorCurrent();
    private final StatusSignal<Voltage> voltage = armMotor.getMotorVoltage();
    private final StatusSignal<AngularVelocity> velocity = armMotor.getVelocity();

    public ArmIOReal(){
        TalonFXConfiguration talonFXConfig;
                talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        Object talonFX;
                talonFX.Config.CurrentLimits.StatorCurrentLimit = CurrentLimits.armKraken;

        talonFXConfig.Audio.BeepOnBoot = true;

        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.coast;

        armMotor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll (50,velocity, voltage, current );

        armMotor.optimizeBusUtilization(1.0);

        StatusCode response = armMotor.getConfigurator().apply(talonFXConfig);
        if(!response.isOK()){
            System.out.println(
                "Talon ID"
                    + armMotor.getDeviceID()
                    + "failed config with error"
                    + response.toString());
        }
    }

    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocity, voltage, current);

        inputs.angularVelocity = velocity.getValueAsDouble()*60;
        inputs.current = current.getValueAsDouble();
        inputs.voltage = voltage.getValueAsDouble();
    }

    @Override
    public void setVelocity(double velocity) {
        armMotor.set(-velocity);
    }

    public void setVoltage(double voltage) {
        armMotor.set(-voltage);
    }
}
