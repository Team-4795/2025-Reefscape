package frc.robot.subsystems.arm;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ArmIOReal implements ArmIO {
    private final SparkFlex armMotor = new SparkFlex(ArmConstants.CAN_ID, MotorType.kBrushless);
    private SparkFlexConfig config = new SparkFlexConfig();
    private SparkAbsoluteEncoder armEncoder;

    private ArmFeedforward ffmodel = new ArmFeedforward(ArmConstants.DEFAULTkS, ArmConstants.DEFAULTkG, ArmConstants.DEFAULTkV, ArmConstants.DEFAULTkA);
    private final SparkClosedLoopController onboardController = armMotor.getClosedLoopController();
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(ArmConstants.MAX_VELOCITY, ArmConstants.MAX_ACCELERATION);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public ArmIOReal(){
        config.smartCurrentLimit(ArmConstants.CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);

        config.absoluteEncoder.positionConversionFactor(Math.PI);
        config.absoluteEncoder.velocityConversionFactor(Math.PI / 60);
        config.absoluteEncoder.inverted(false);

        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimitEnabled(false);
        config.softLimit.forwardSoftLimit(ArmConstants.Sim.MAX_ANGLE);
        config.softLimit.reverseSoftLimit(ArmConstants.Sim.MIN_ANGLE);

        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.closedLoop.p(0.0);
        config.closedLoop.i(0.0);
        config.closedLoop.d(0.0);

        armMotor.clearFaults();
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armEncoder = armMotor.getAbsoluteEncoder();
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
    public void hold() {
        double ffvolts = ffmodel.calculate(armEncoder.getPosition(), 0);
        onboardController.setReference(ffvolts, ControlType.kVoltage);
    }

    @Override
    public void resetAbsoluteEncoder() {
        armMotor.getEncoder().setPosition(0);
    }

    @Override
    public void setVoltage(double voltage) {
        onboardController.setReference(voltage, ControlType.kVoltage);
        // armMotor.setVoltage(MathUtil.clamp(-voltage, -12, 12));
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if(!inputs.openLoop) {
            // setVoltage(ffmodel.calculate(setpoint.position, setpoint.velocity));
            double ffvolts = ffmodel.calculate(armEncoder.getPosition(), setpoint.velocity);
            onboardController.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffvolts);
            setpoint = profile.calculate(0.02, setpoint, goal);
        }

        inputs.angularPosition = armEncoder.getPosition();
        inputs.angularVelocity = armEncoder.getVelocity();
        inputs.current = armMotor.getOutputCurrent();
        inputs.voltage = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
        inputs.setpointVelocity = setpoint.velocity;
    }
}
