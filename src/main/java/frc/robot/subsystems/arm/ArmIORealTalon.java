package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class ArmIORealTalon implements ArmIO {
    private final TalonFX armMotor = new TalonFX(ArmConstants.CAN_ID);
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private SparkAbsoluteEncoder encoder;

    private final StatusSignal<Voltage> voltage = armMotor.getMotorVoltage();
    private final StatusSignal<Current> current = armMotor.getStatorCurrent();
    private final StatusSignal<AngularVelocity> velocity = armMotor.getVelocity();
    private final StatusSignal<Angle> position = armMotor.getPosition();

    public ArmIORealTalon(){
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ArmConstants.CURRENT_LIMIT;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        BaseStatusSignal.setUpdateFrequencyForAll(50, position, velocity, voltage, current);
        
        StatusCode response = armMotor.getConfigurator().apply(config);
        if (!response.isOK()) {
            System.out.println(
                    "Talon ID "
                            + armMotor.getDeviceID()
                            + " failed config with error "
                            + response.toString());
        }
    }

    @Override
    public void updateMotionProfile(){

    }

    @Override
    public void resetEncoder(){

    }

    @Override
    public void setVoltage(double voltage){

    }

    @Override
    public void setFFValues(double kS, double kG, double kA, double kV){

    }

    @Override
    public void setGoal(double angle){

    }

    @Override
    public double getGoal(){
        return 0;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs){ // not done
        BaseStatusSignal.refreshAll(position, velocity, voltage, current);
        inputs.voltage = voltage.getValueAsDouble();
        inputs.angularPosition = encoder.getPosition();
        inputs.angularVelocity = encoder.getVelocity();
        inputs.current = current.getValueAsDouble();
        inputs.goalAngle = 0.0;
        inputs.setpointPosition = 0.0;
        inputs.setpointVelocity = 0.0;
        inputs.appliedOutput = 0.0; 
        inputs.busVoltage = 0.0;
        inputs.relativeEncoderPosition = position.getValueAsDouble();
        inputs.relativeEncoderVelocity = velocity.getValueAsDouble();
        inputs.angularPositionDegrees = Math.toDegrees(position.getValueAsDouble());
    }
}