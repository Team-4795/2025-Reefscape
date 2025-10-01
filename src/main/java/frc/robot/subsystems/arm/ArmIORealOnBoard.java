package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class ArmIORealOnBoard implements ArmIO {
    private final TalonFX armMotor = new TalonFX(ArmConstants.CAN_ID);

    private TalonFXConfiguration config = new TalonFXConfiguration(); 
    private final PositionVoltage positionVoltage = new PositionVoltage(0.0).withEnableFOC(false);

    private final StatusSignal<Current> current = armMotor.getStatorCurrent();
    private final StatusSignal<Voltage> voltage = armMotor.getMotorVoltage();
    private final StatusSignal<AngularVelocity> velocity = armMotor.getVelocity();
    private final StatusSignal<Angle> position = armMotor.getPosition();

    private double goal = 0.0;

    public ArmIORealOnBoard() {
        armMotor.clearStickyFaults(); 

        config.CurrentLimits.StatorCurrentLimitEnable = true; 
        config.CurrentLimits.StatorCurrentLimit = ArmConstants.CURRENT_LIMIT; 

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        BaseStatusSignal.setUpdateFrequencyForAll(50, velocity, voltage, current, position);

        config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MAX_VELOCITY; 
        config.MotionMagic.MotionMagicAcceleration = ArmConstants.MAX_ACCELERATION; 
            
        // PID slot 0
        config.Slot0.kP = ArmConstants.kP;
        config.Slot0.kI = ArmConstants.kI;
        config.Slot0.kD = ArmConstants.kD;
        config.Slot0.kA = ArmConstants.CORALkA;
        config.Slot0.kG = ArmConstants.CORALkG;
        config.Slot0.kV = ArmConstants.CORALkV; 
        config.Slot0.kS = ArmConstants.CORALkS; 

        //replace with the actual gearing
        config.Feedback.RotorToSensorRatio = 1.0; 

        StatusCode response = armMotor.getConfigurator().apply(config);
        if (!response.isOK()) {
            System.out.println(
                    "Talon ID "
                            + armMotor.getDeviceID()
                            + " failed config with error "
                            + response.toString());
        }
        
        armMotor.stopMotor();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocity, position, current, voltage);

        inputs.relativeEncoderPosition = position.getValueAsDouble();
        inputs.relativeEncoderVelocity = velocity.getValueAsDouble();
        inputs.voltage = voltage.getValueAsDouble();
        inputs.current = current.getValueAsDouble();

        inputs.angularPosition = inputs.relativeEncoderPosition;
        inputs.angularVelocity = inputs.relativeEncoderVelocity;
        inputs.goalAngle = goal;
        inputs.appliedOutput = armMotor.getDutyCycle().getValue();
        inputs.busVoltage = armMotor.getSupplyVoltage().getValueAsDouble();
        inputs.angularPositionDegrees = Math.toDegrees(inputs.relativeEncoderPosition);
    }

    @Override
    public void setGoal(double angle) {
     goal = angle;
    }

    @Override
    public void updateMotionProfile() {
        armMotor.setControl(positionVoltage.withPosition(goal));
    }

    @Override
    public void resetEncoder() {
        armMotor.setPosition(0);
    }

    @Override
    public void setVoltage(double voltageVolts) {
        armMotor.setVoltage(voltageVolts);
    }

    @Override
    public double getGoal() {
        return goal;
    }


    
}
