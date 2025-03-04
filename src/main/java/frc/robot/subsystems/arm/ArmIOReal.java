package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;
// 
public class ArmIOReal implements ArmIO {
    private final SparkFlex armMotor = new SparkFlex(ArmConstants.CAN_ID, MotorType.kBrushless);
    private SparkFlexConfig config = new SparkFlexConfig();
    private SparkAbsoluteEncoder armEncoder;

    LoggedTunableNumber KP = new LoggedTunableNumber("Arm/KP", ArmConstants.kP);
    LoggedTunableNumber KI = new LoggedTunableNumber("Arm/KI", ArmConstants.kI);
    LoggedTunableNumber KD = new LoggedTunableNumber("Arm/KD", ArmConstants.kD);    



    private ArmFeedforward ffmodel = new ArmFeedforward(ArmConstants.DEFAULTkS, ArmConstants.DEFAULTkG, ArmConstants.DEFAULTkV, ArmConstants.DEFAULTkA, 0.02);
    // private final SparkClosedLoopController onboardController = armMotor.getClosedLoopController();
    private final PIDController controller = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
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
        config.encoder.velocityConversionFactor(2 * Math.PI / ArmConstants.Sim.GEARING / 60);
        config.encoder.quadratureMeasurementPeriod(20);


        config.softLimit.forwardSoftLimitEnabled(true);
        config.softLimit.reverseSoftLimitEnabled(false);
        config.softLimit.forwardSoftLimit(ArmConstants.Sim.MAX_ANGLE);
        config.softLimit.reverseSoftLimit(ArmConstants.Sim.MIN_ANGLE);

        // config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        // config.closedLoop.p(0);
        // config.closedLoop.i(0.0);
        // config.closedLoop.d(0.0);

        config.voltageCompensation(12.0);
        config.inverted(false);
        config.absoluteEncoder.inverted(false);

        armMotor.clearFaults();
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armEncoder = armMotor.getAbsoluteEncoder();
        armMotor.getEncoder().setPosition(getOffsetAngle());

        goal = new TrapezoidProfile.State(getOffsetAngle(), 0);
        setpoint = new TrapezoidProfile.State(getOffsetAngle(), 0);
        
        Logger.recordOutput("arm offset angle", getOffsetAngle());
    }

    @Override
    public void setGoal(double angle) {
        if(angle != goal.position) {
            setpoint = new TrapezoidProfile.State(armMotor.getEncoder().getPosition(), armEncoder.getVelocity());
            goal = new TrapezoidProfile.State(angle, 0);
        }
    }

    @Override
    public void hold() {
        double ffvolts = ffmodel.calculate(armMotor.getEncoder().getPosition(), 0);
        double pidvolts = controller.calculate(armMotor.getEncoder().getPosition(), goal.position);
        setVoltage(ffvolts + pidvolts);
    }
 
    @Override
    public double getGoal() {
        return goal.position;
    }
    
    @Override
    public void updateMotionProfile() {
        // double prevVelocity = setpoint.velocity;
        // double acceleration = (setpoint.velocity - prevVelocity) / 0.02;
        // double ffvolts = ffmodel.calculate(armMotor.getEncoder().getPosition(), setpoint.velocity, acceleration);
        setpoint = profile.calculate(0.02, setpoint, goal);
        double ffvolts = ffmodel.calculate(armMotor.getEncoder().getPosition(), setpoint.velocity);
        double pidvolts = controller.calculate(armMotor.getEncoder().getPosition(), setpoint.position);
  
        setVoltage(ffvolts + pidvolts);
        // onboardController.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffvolts);
        // onboardController.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);

        Logger.recordOutput("Arm/ffvolts", ffvolts);
        Logger.recordOutput("Arm/pidvolts", pidvolts);
        // Logger.recordOutput("Arm/prev velocity", prevVelocity);
        // Logger.recordOutput("Arm/acceleration", acceleration);
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
        armMotor.setVoltage(voltage);
        // onboardController.setReference(voltage, ControlType.kVoltage);
    }

    public double getOffsetAngle() {
        return armEncoder.getPosition() - ArmConstants.ARM_OFFSET;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // inputs.angularPosition = getOffsetAngle();
        inputs.angularPosition = armEncoder.getPosition();
        inputs.angularVelocity = armEncoder.getVelocity();
        inputs.current = armMotor.getOutputCurrent();
        inputs.voltage = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
        inputs.setpointVelocity = setpoint.velocity;
        inputs.goalAngle = goal.position;
        inputs.busVoltage = armMotor.getBusVoltage();
        inputs.appliedOutput = armMotor.getAppliedOutput();
        inputs.relativeEncoderPosition = armMotor.getEncoder().getPosition();
        inputs.relativeEncoderVelocity = armMotor.getEncoder().getVelocity();
        inputs.setpointPosition = setpoint.position;
        inputs.angularPositionDegrees = Units.radiansToDegrees(getOffsetAngle());
    }
}
