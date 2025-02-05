package frc.robot.subsystems.arm;

import java.util.concurrent.atomic.AtomicReferenceFieldUpdater;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmIOReal implements ArmIO {
    private final SparkFlex armMotor = new SparkFlex(ArmConstants.CAN_ID, MotorType.kBrushless);
    private SparkFlexConfig config = new SparkFlexConfig();
    private SparkAbsoluteEncoder armEncoder;

    private ArmFeedforward ffmodel = new ArmFeedforward(ArmConstants.DEFAULTkS, ArmConstants.DEFAULTkG, ArmConstants.DEFAULTkV, ArmConstants.DEFAULTkA);
    private final PIDController controller = new PIDController(0.04, 0,0.00);
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2, 2);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public ArmIOReal(){
        config.smartCurrentLimit(ArmConstants.CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);
        config.absoluteEncoder.positionConversionFactor(1);
        config.absoluteEncoder.positionConversionFactor(Math.PI);
        config.absoluteEncoder.velocityConversionFactor(Math.PI / 60);
        config.absoluteEncoder.inverted(false);
        config.encoder.positionConversionFactor(Math.PI);
        config.encoder.velocityConversionFactor(Math.PI / 60);
        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimitEnabled(false);
        config.softLimit.forwardSoftLimit(ArmConstants.Sim.MAX_ANGLE);
        config.softLimit.reverseSoftLimit(ArmConstants.Sim.MIN_ANGLE);
        armMotor.clearFaults();
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armEncoder = armMotor.getAbsoluteEncoder();
        armMotor.getEncoder().setPosition(armEncoder.getPosition());
    }

    @Override
    public void setGoal(double angle) {
        goal = new TrapezoidProfile.State(angle, 0);
    }

    @Override
    public void setFFValues(double kS, double kG, double kV, double kA) {
        ffmodel = new ArmFeedforward(kS, kG, kV);
    }

    @Override
    public void resetAbsoluteEncoder() {
        armMotor.getEncoder().setPosition(0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if(!inputs.openLoop) {
            // setVoltage(ffmodel.calculate(setpoint.position, setpoint.velocity) + controller.calculate(inputs.angularVelocity, setpoint.velocity));
            setVoltage(ffmodel.calculate(setpoint.position, setpoint.velocity - ArmConstants.ARM_OFFSET));
            setpoint = profile.calculate(0.02, setpoint, goal);
        }

        inputs.angularPosition = armMotor.getAbsoluteEncoder().getPosition();
        inputs.angularVelocity = armEncoder.getVelocity();
        inputs.current = armMotor.getOutputCurrent();
        inputs.voltage = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
        inputs.setpointVelocity = setpoint.velocity;
    }

    public void setVoltage(double voltage) {
        armMotor.setVoltage(MathUtil.clamp(-voltage, -12, 12));
    }
}
