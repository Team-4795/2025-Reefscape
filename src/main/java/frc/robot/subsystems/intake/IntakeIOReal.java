package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;


public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.canID);
    
    private TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    private final StatusSignal<Current> current = intakeMotor.getStatorCurrent();
    private final StatusSignal<Voltage> voltage = intakeMotor.getMotorVoltage();
    private final StatusSignal<AngularVelocity> velocity = intakeMotor.getVelocity();

    public IntakeIOReal() {
        talonFXConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);

        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.currentLimit;


        talonFXConfig.Audio.BeepOnBoot = true;
        
        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        intakeMotor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll(50, velocity, voltage, current);

        intakeMotor.optimizeBusUtilization(4, 1);

        StatusCode response = intakeMotor.getConfigurator().apply(talonFXConfig);
        if (!response.isOK()) {
            System.out.println(
                    "Talon ID "
                            + intakeMotor.getDeviceID()
                            + " failed config with error "
                            + response.toString());
        }
    }

    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocity, voltage, current);
        
        inputs.angularVelocityRPM = velocity.getValueAsDouble() * 60;
        inputs.angularPositionRot = intakeMotor.getPosition().getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.voltage = voltage.getValueAsDouble();
    }

    @Override
    public boolean hasGamepiece() {
        return current.getValueAsDouble() > IntakeConstants.currentThreshold;
    }

    @Override
    public void setMotorSpeed(double speed) {
        intakeMotor.set(-speed);
    }

}

