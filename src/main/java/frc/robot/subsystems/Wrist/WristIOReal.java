package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class WristIOReal implements WristIO{
    // neo vortex motor
    private SparkFlex wristMotor = new SparkFlex(WristConstants.id, MotorType.kBrushless);
    private SparkFlexConfig config = new SparkFlexConfig();
    private RelativeEncoder encoder = wristMotor.getEncoder();
    private ArmFeedforward ff = new ArmFeedforward(WristConstants.kS, WristConstants.kG, WristConstants.kV);
    private ProfiledPIDController controller = new ProfiledPIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD, 
    new TrapezoidProfile.Constraints(WristConstants.maxV, WristConstants.maxA));
    private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(WristConstants.maxV, WristConstants.maxA));
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    public WristIOReal() {
        config.smartCurrentLimit(WristConstants.stallLimit, WristConstants.freeLimit);
        wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.encoder.positionConversionFactor(2*Math.PI / 72.0);
        config.encoder.velocityConversionFactor(2*Math.PI / 60.0 / 72.0);

    }

    @Override
    public double getPosition(){
        return encoder.getPosition() + WristConstants.offset;
    }

    @Override
    public void setGoal(double angle){
        setpoint = new TrapezoidProfile.State(getPosition(), encoder.getVelocity());
        goal = new TrapezoidProfile.State(angle, 0);
    }

    @Override
    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }

    @Override
    public void updateMotionProfile(){
        setpoint = profile.calculate(0.02, setpoint, goal);
        double pidVolts = controller.calculate(getPosition(), setpoint.position);
        Logger.recordOutput("pid volts", pidVolts);
        double ffVolts = ff.calculate(getPosition(), setpoint.velocity);
        Logger.recordOutput("ff volts", ffVolts);
        setVoltage(ffVolts + pidVolts);
    }

    @Override
    public void updateInputs (WristIOInputs inputs){
        inputs.voltage = wristMotor.getBusVoltage();
        inputs.pos = getPosition();
        inputs.velocity = encoder.getVelocity();
    }
}
