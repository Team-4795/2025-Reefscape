package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;


public class IntakeIORealVortex implements IntakeIO {
    private final SparkFlex intakeMotor = new SparkFlex(IntakeConstants.canID, MotorType.kBrushless);
    private final RelativeEncoder encoder = intakeMotor.getEncoder();
    
    private SparkFlexConfig config = new SparkFlexConfig();

    public IntakeIORealVortex() {
        config.smartCurrentLimit(IntakeConstants.currentLimit);
        config.idleMode(IdleMode.kCoast);

        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.angularVelocityRPM = encoder.getVelocity();
        inputs.angularPositionRot = encoder.getPosition();
        inputs.currentAmps = intakeMotor.getOutputCurrent();
        inputs.voltage = intakeMotor.getBusVoltage();
    }

    @Override
    public boolean hasGamepiece() {
        return intakeMotor.getOutputCurrent() > IntakeConstants.currentThreshold;
    }

    @Override
    public void setMotorSpeed(double speed) {
        intakeMotor.set(-speed);
    }

}

