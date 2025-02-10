package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

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

import edu.wpi.first.math.MathUtil;
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
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    public ArmIOReal(){
        config.smartCurrentLimit(ArmConstants.CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);

        config.absoluteEncoder.positionConversionFactor(Math.PI);
        config.absoluteEncoder.velocityConversionFactor(Math.PI / 60);
        config.absoluteEncoder.inverted(false);

        config.encoder.positionConversionFactor(2 * Math.PI / ArmConstants.Sim.GEARING);
        config.encoder.velocityConversionFactor(2 * Math.PI / ArmConstants.Sim.GEARING);

        config.openLoopRampRate(1);

        // config.softLimit.forwardSoftLimitEnabled(true);
        // config.softLimit.reverseSoftLimitEnabled(false);
        // config.softLimit.forwardSoftLimit(ArmConstants.Sim.MAX_ANGLE);
        // config.softLimit.reverseSoftLimit(ArmConstants.Sim.MIN_ANGLE);

        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        config.closedLoop.p(0.0);
        config.closedLoop.i(0.0);
        config.closedLoop.d(0.0);

        config.inverted(true);

        armMotor.clearFaults();
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armEncoder = armMotor.getAbsoluteEncoder();
        armMotor.getEncoder().setPosition(getOffsetAngle());

        goal = new TrapezoidProfile.State(getOffsetAngle(), 0);
        setpoint = new TrapezoidProfile.State();
    }

    @Override
    public void setGoal(double angle) {
        goal = new TrapezoidProfile.State(MathUtil.clamp(angle, ArmConstants.Sim.MIN_ANGLE,  ArmConstants.Sim.MAX_ANGLE), 0);
    }

    @Override
    public double getGoal() {
        return goal.position;
    }
    
    @Override
    public void updateMotionProfile() {
        double ffvolts = ffmodel.calculate(getOffsetAngle(), setpoint.velocity);
        onboardController.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffvolts);
        setpoint = profile.calculate(0.02, setpoint, goal);
    }

    @Override
    public void setFFValues(double kS, double kG, double kV, double kA) {
        ffmodel = new ArmFeedforward(kS, kG, kV);
    }


    @Override
    public void resetEncoder() {
        armMotor.getEncoder().setPosition(0);
    }

    @Override
    public void setVoltage(double voltage) {
        onboardController.setReference(voltage, ControlType.kVoltage);
    }

    public double getOffsetAngle() {
       double withoffset = armEncoder.getPosition() - ArmConstants.ARM_OFFSET;
       if(withoffset > Math.PI) {
        return -1 * (2 * Math.PI - withoffset);
        }
        else {
            return withoffset;
        }
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.angularPosition = getOffsetAngle();
        // inputs.angularPosition = armEncoder.getPosition();
        inputs.angularVelocity = armEncoder.getVelocity();
        inputs.current = armMotor.getOutputCurrent();
        inputs.voltage = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
        inputs.setpointVelocity = setpoint.velocity;
        inputs.goalAngle = goal.position;
        inputs.busVoltage = armMotor.getBusVoltage();
        inputs.appliedOutput = armMotor.getAppliedOutput();
        inputs.relativeEncoderPosition = armMotor.getEncoder().getPosition();
        inputs.relativeEncoderVelocity = armMotor.getEncoder().getVelocity();
    }
}
