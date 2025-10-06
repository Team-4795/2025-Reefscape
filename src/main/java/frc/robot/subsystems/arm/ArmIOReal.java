package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

public class ArmIOReal implements ArmIO {
    private final TalonFX armMotor = new TalonFX(ArmConstants.CAN_ID);
    private TalonFXConfiguration config = new TalonFXConfiguration();

    LoggedTunableNumber KP = new LoggedTunableNumber("Arm/KP", ArmConstants.kP);
    LoggedTunableNumber KI = new LoggedTunableNumber("Arm/KI", ArmConstants.kI);
    LoggedTunableNumber KD = new LoggedTunableNumber("Arm/KD", ArmConstants.kD); 
    
    LoggedTunableNumber KG = new LoggedTunableNumber("Arm/KG", ArmConstants.DEFAULTkG);
    LoggedTunableNumber KS = new LoggedTunableNumber("Arm/KS", ArmConstants.DEFAULTkS);
    LoggedTunableNumber KV = new LoggedTunableNumber("Arm/KV", ArmConstants.DEFAULTkV);
    LoggedTunableNumber KA = new LoggedTunableNumber("Arm/KA", ArmConstants.DEFAULTkA);  

    private ArmFeedforward ffmodel = new ArmFeedforward(ArmConstants.DEFAULTkS, ArmConstants.DEFAULTkG, ArmConstants.DEFAULTkV, ArmConstants.DEFAULTkA, 0.02);
    private PIDController controller = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(ArmConstants.MAX_VELOCITY, ArmConstants.MAX_ACCELERATION);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    public ArmIOReal(){
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ArmConstants.CURRENT_LIMIT;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.Sim.MAX_ANGLE;

        armMotor.clearStickyFault_BootDuringEnable();

        goal = new TrapezoidProfile.State(getOffsetAngle(), 0);
        setpoint = new TrapezoidProfile.State(getOffsetAngle(), 0);
        
        Logger.recordOutput("arm offset angle", getOffsetAngle());

        StatusCode response = armMotor.getConfigurator().apply(config);
        if (!response.isOK()) {
            System.out.println(
                    "Talon ID "
                            + armMotor.getDeviceID()
                            + " failed config with error "
                            + response.toString());
        }
    }

    // Write method that sets arm goal and sets the setpoint to the current position/velocity
    @Override
    public void setGoal(double angle) {

    }

    // Write method that sends voltage to the arm
    @Override
    public void setVoltage(double voltage) {
        
    }

    @Override
    public void hold() {
        double ffvolts = ffmodel.calculate(getOffsetAngle(), 0);
        double pidvolts = controller.calculate(getOffsetAngle(), goal.position);
        setVoltage(ffvolts + pidvolts);
    }
 
    @Override
    public double getGoal() {
        return goal.position;
    }
    
    @Override
    public void updateMotionProfile() {
        setpoint = profile.calculate(0.02, setpoint, goal);
        double ffvolts = ffmodel.calculate(getOffsetAngle(), setpoint.velocity);
        double pidvolts = controller.calculate(getOffsetAngle(), setpoint.position);
  
        setVoltage(ffvolts + pidvolts);

        Logger.recordOutput("Arm/ffvolts", ffvolts);
        Logger.recordOutput("Arm/pidvolts", pidvolts);
    }

    @Override
    public void setFFValues(double kS, double kG, double kV, double kA) {
        ffmodel = new ArmFeedforward(kS, kG, kV);
    }

    public double getOffsetAngle() {
        return armMotor.getPosition().getValueAsDouble() - ArmConstants.ARM_OFFSET;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        ffmodel = new ArmFeedforward(KS.get(), KG.get(), KV.get(), KA.get(), 0.02);
        controller = new PIDController(KP.get(), KI.get(), KD.get());

        inputs.angularPosition = getOffsetAngle();
        inputs.angularVelocity = armMotor.getVelocity().getValueAsDouble();
        inputs.current = armMotor.getStatorCurrent().getValueAsDouble();
        inputs.voltage = armMotor.getMotorVoltage().getValueAsDouble();
        inputs.setpointVelocity = setpoint.velocity;
        inputs.goalAngle = goal.position;
        inputs.setpointPosition = setpoint.position;
        inputs.angularPositionDegrees = Units.radiansToDegrees(getOffsetAngle());
    }
}
