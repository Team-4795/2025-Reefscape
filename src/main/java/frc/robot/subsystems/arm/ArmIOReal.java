package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ArmIOReal implements ArmIO {
    private final TalonFX armMotor = new TalonFX(ArmConstants.CAN_ID);
    private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    private final StatusSignal<Current> current = armMotor.getStatorCurrent();
    private final StatusSignal<Voltage> voltage = armMotor.getMotorVoltage();
    private final StatusSignal<AngularVelocity> velocity = armMotor.getVelocity();
    private final StatusSignal<Angle> position = armMotor.getPosition();
    private final ArmFeedforward ffmodel = new ArmFeedforward(0.1, 0.1, 0.1);

    public ArmIOReal(){
        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = ArmConstants.CURRENT_LIMIT;
        talonFXConfig.Audio.BeepOnBoot = true;
        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armMotor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll(50,velocity, voltage, current);

        armMotor.optimizeBusUtilization(50, 1);

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

        inputs.angularPosition = position.getValueAsDouble() * 2 * Math.PI;
        inputs.angularVelocity = velocity.getValueAsDouble() * 2 * Math.PI;
        inputs.current = current.getValueAsDouble();
        inputs.voltage = voltage.getValueAsDouble();
        inputs.angularPosition = armMotor.getPosition().getValueAsDouble()*2*Math.PI;
    }

    @Override
    public void setVelocity(double velocity) {
        setVoltage(ffmodel.calculate(position.getValueAsDouble() * 2 * Math.PI, velocity));
    }

    public void setVoltage(double voltage) {
        armMotor.setVoltage(-voltage);
    }

     
}
