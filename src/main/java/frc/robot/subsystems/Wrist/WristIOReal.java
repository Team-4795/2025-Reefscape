package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.LoggedTunableNumber;


public class WristIOReal implements WristIO{

    public SparkFlex wristMotor = new SparkFlex(WristConstants.id, MotorType.kBrushless);
    public SparkAbsoluteEncoder armAbsoluteEncoder = wristMotor.getAbsoluteEncoder();
    private SparkFlexConfig config = new SparkFlexConfig();
    private RelativeEncoder wristEncoder = wristMotor.getEncoder();

    LoggedTunableNumber KP = new LoggedTunableNumber("Wrist/KP", WristConstants.Coral_kP);
    LoggedTunableNumber KI = new LoggedTunableNumber("Wrist/KI", WristConstants.Coral_kI);
    LoggedTunableNumber KD = new LoggedTunableNumber("Wrist/KD", WristConstants.Coral_kD);    
    LoggedTunableNumber KV = new LoggedTunableNumber("Wrist/KP", WristConstants.Coral_kV);
    LoggedTunableNumber KA = new LoggedTunableNumber("Wrist/KI", WristConstants.Coral_KA);
    LoggedTunableNumber KG = new LoggedTunableNumber("Wrist/KD", WristConstants.Coral_kG);    

    private ProfiledPIDController controller = new ProfiledPIDController(WristConstants.Coral_kP, WristConstants.Coral_kI, WristConstants.Coral_kD, 
    new TrapezoidProfile.Constraints(WristConstants.maxV, WristConstants.maxA));
    private ArmFeedforward feedForward = new ArmFeedforward(WristConstants.Coral_kS, WristConstants.Coral_kG, WristConstants.Coral_kV, WristConstants.Coral_KA, 0.02);
    private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(WristConstants.maxV, WristConstants.maxA));
    private TrapezoidProfile.State goal ;
    private TrapezoidProfile.State setpoint;

    public WristIOReal() {
      
        config.smartCurrentLimit(WristConstants.currentLimit);        
        wristMotor.setCANTimeout(20);

        config.encoder.positionConversionFactor(2*Math.PI / WristConstants.gearing);
        config.encoder.velocityConversionFactor(2*Math.PI / 60.0 / WristConstants.gearing);

        // config.softLimit.forwardSoftLimitEnabled(true);
        // config.softLimit.reverseSoftLimitEnabled(true);
        // config.softLimit.forwardSoftLimit(WristConstants.maxPosition);
        // config.softLimit.reverseSoftLimit(WristConstants.minPosition);

        // on board PID if needed later
        // config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        // config.closedLoop.p(0);
        // config.closedLoop.i(0);
        // config.closedLoop.d(0);

        config.absoluteEncoder.positionConversionFactor(2*Math.PI);
        config.absoluteEncoder.velocityConversionFactor(2*Math.PI/60);
        config.absoluteEncoder.inverted(true);

        config.voltageCompensation(WristConstants.voltageCompensation);
        config.inverted(WristConstants.isInverted);
        wristMotor.clearFaults();

        wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        goal = new TrapezoidProfile.State(wristEncoder.getPosition(),0);
        setpoint = new TrapezoidProfile.State(wristEncoder.getPosition(), 0);

    }


    @Override
    public double getPosition(){
        return wristEncoder.getPosition();
    }

    @Override
    public double getVelocity(){
        return wristEncoder.getVelocity();
    }

    @Override
    public SparkAbsoluteEncoder getArmAbsoluteEncoder(){
        return wristMotor.getAbsoluteEncoder();
        
    }
    @Override
    public void setGoal(double angle){
        if (angle != goal.position){
        setpoint = new TrapezoidProfile.State(getPosition(), getVelocity());
        goal = new TrapezoidProfile.State(angle, 0);
        }
    }
//MathUtil.clamp(angle, WristConstants.minPosition, WristConstants.maxPosition)
    @Override
    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }


    @Override
    public void updateMotionProfile(){
        setpoint = profile.calculate(0.02, setpoint, goal);
        double pidVolts = controller.calculate(getPosition(), setpoint.position);
        Logger.recordOutput("pid volts", pidVolts);
        double ffVolts = feedForward.calculate(getPosition(), setpoint.velocity);
        Logger.recordOutput("ff volts", ffVolts);
        setVoltage(ffVolts + pidVolts);
    }

    @Override
    public void updateInputs (WristIOInputs inputs){
        inputs.voltage = wristMotor.getBusVoltage();
        inputs.position = getPosition();
        inputs.velocity = wristEncoder.getVelocity();
        inputs.current = wristMotor.getOutputCurrent();
        inputs.goalPosition = goal.position;
        inputs.setPointVelocity = setpoint.velocity;


    }
}
