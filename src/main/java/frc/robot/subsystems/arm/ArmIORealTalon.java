package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.util.LoggedTunableNumber;

public class ArmIORealTalon implements ArmIO {
    private final TalonFX armMotor = new TalonFX(ArmConstants.CAN_ID);
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private SparkAbsoluteEncoder encoder = Wrist.getInstance().getArmAbsoluteEncoder();;
    private AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();

    LoggedTunableNumber KP = new LoggedTunableNumber("Arm/kP", ArmConstants.kP);
    LoggedTunableNumber KI = new LoggedTunableNumber("Arm/kI", ArmConstants.kI);
    LoggedTunableNumber KD = new LoggedTunableNumber("Arm/kD", ArmConstants.kD);
    LoggedTunableNumber KV = new LoggedTunableNumber("Arm/kV", ArmConstants.DEFAULTkV);
    LoggedTunableNumber KS = new LoggedTunableNumber("Arm/kS", ArmConstants.DEFAULTkS);
    LoggedTunableNumber KA = new LoggedTunableNumber("Arm/kA", ArmConstants.DEFAULTkA);
    LoggedTunableNumber KG = new LoggedTunableNumber("Arm/kG", ArmConstants.DEFAULTkG);

    private final StatusSignal<Voltage> voltage = armMotor.getMotorVoltage();
    private final StatusSignal<Current> current = armMotor.getStatorCurrent();
    private final StatusSignal<AngularVelocity> velocity = armMotor.getVelocity();
    private final StatusSignal<Angle> position = armMotor.getPosition();

    private ArmFeedforward ffModel = new ArmFeedforward(ArmConstants.DEFAULTkS, ArmConstants.DEFAULTkG, ArmConstants.DEFAULTkV);
    private PIDController controller = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(ArmConstants.MAX_VELOCITY, ArmConstants.MAX_ACCELERATION);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;

    public ArmIORealTalon(){
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ArmConstants.CURRENT_LIMIT;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        

        encoderConfig.positionConversionFactor(2 * Math.PI / ArmConstants.Sim.GEARING);
        encoderConfig.velocityConversionFactor(2 * Math.PI / ArmConstants.Sim.GEARING / 60.0);
        
        BaseStatusSignal.setUpdateFrequencyForAll(50, position, velocity, voltage, current);

        goal = new TrapezoidProfile.State(getOffsetAngle(), 0);
        setpoint = new TrapezoidProfile.State(getOffsetAngle(), 0);
        
        // need to finish encoder
        
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
        setpoint = profile.calculate(0.02, setpoint, goal);
        double ffvolts = ffModel.calculate(getOffsetAngle(), setpoint.velocity);
        double pidvolts = controller.calculate(getOffsetAngle(), setpoint.position);
  
        setVoltage(ffvolts + pidvolts);
    }

    @Override
    public void resetEncoder(){

    }

    @Override
    public void setVoltage(double voltage){
        armMotor.setVoltage(voltage);
    }

    @Override
    public void setFFValues(double kS, double kG, double kA, double kV){

    }

    @Override
    public void hold(){
        
    }

    @Override
    public void setGoal(double angle){
        if(angle != goal.position) {
            setpoint = new TrapezoidProfile.State(getOffsetAngle(), encoder.getVelocity());
            goal = new TrapezoidProfile.State(angle, 0);
        }
    }

    @Override
    public double getGoal(){
        return 0;
    }

    public double getOffsetAngle() {
        // return armMotor.getPosition().getValueAsDouble()/60.0 - ArmConstants.ARM_OFFSET;
        return encoder.getPosition() - ArmConstants.ARM_OFFSET;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs){ // not done
        BaseStatusSignal.refreshAll(position, velocity, voltage, current);
        ffModel = new ArmFeedforward(KS.get(), KG.get(), KV.get(), KA.get(), 0.02);
        controller = new PIDController(KP.get(), KI.get(), KD.get());

        inputs.voltage = voltage.getValueAsDouble();
        inputs.angularPosition = getOffsetAngle();
        inputs.angularVelocity = encoder.getVelocity();
        inputs.current = current.getValueAsDouble();
        inputs.goalAngle = goal.position;
        inputs.setpointPosition = setpoint.position;
        inputs.setpointVelocity = setpoint.velocity;
        inputs.appliedOutput = 0.0; 
        inputs.busVoltage = 0.0;
        inputs.relativeEncoderPosition = position.getValueAsDouble();
        inputs.relativeEncoderVelocity = velocity.getValueAsDouble();
        inputs.angularPositionDegrees = Math.toDegrees(position.getValueAsDouble());
    }
}