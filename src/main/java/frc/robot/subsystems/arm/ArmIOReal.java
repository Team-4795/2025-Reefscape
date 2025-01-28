package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
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
    private AbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder();

    private final ArmFeedforward ffmodel = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
    private final PIDController controller = new PIDController(0.04, 0,0.00);
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 2);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(ArmConstants.Sim.INIT_ANGLE, 0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public ArmIOReal(){
        config.smartCurrentLimit(ArmConstants.CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);
        armMotor.clearFaults();
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setGoal(double angle) {
        goal = new TrapezoidProfile.State(angle, 0);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        if(!inputs.openLoop) {
            setVoltage(ffmodel.calculate(setpoint.velocity, setpoint.position) + controller.calculate(inputs.angularVelocity, setpoint.velocity));
            setpoint = profile.calculate(0.02, setpoint, goal);
        }

        inputs.angularPosition = armEncoder.getPosition() * 2 * Math.PI;
        inputs.angularVelocity = armEncoder.getVelocity() * 2 * Math.PI / 60;
        inputs.current = armMotor.getOutputCurrent();
        inputs.voltage = armMotor.getAppliedOutput();
    }

    public void setVoltage(double voltage) {
        armMotor.setVoltage(MathUtil.clamp(-voltage, -12, 12));
    }
}
