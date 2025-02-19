package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorIOReal implements ElevatorIO {
    private SparkFlex rightElevatorMotor = new SparkFlex(ElevatorConstants.rightDeviceID, MotorType.kBrushless);
    private SparkFlex leftElevatorMotor = new SparkFlex(ElevatorConstants.leftDeviceID, MotorType.kBrushless); 

    private RelativeEncoder leftEncoder = leftElevatorMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightElevatorMotor.getEncoder();

    // private SparkClosedLoopController controller = rightElevatorMotor.getClosedLoopController();

 //   private AbsoluteEncoder leftAbsoluteEncoder = leftElevatorMotor.getAbsoluteEncoder();

    private final ElevatorFeedforward ffmodel = new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv);
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
    private final PIDController controller = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();

    private SparkFlexConfig config = new SparkFlexConfig();

    //zero stuff 
    public void zeroElevator(){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    
    
    public ElevatorIOReal(){
        leftElevatorMotor.clearFaults();
        rightElevatorMotor.clearFaults();
        config.encoder.positionConversionFactor(ElevatorConstants.conversionFactor);
        config.encoder.velocityConversionFactor(ElevatorConstants.conversionFactor / 60);    
        config.encoder.quadratureMeasurementPeriod(20);

        // config.softLimit.forwardSoftLimitEnabled(true);
        // config.softLimit.reverseSoftLimitEnabled(true);
        // config.softLimit.forwardSoftLimit(ElevatorConstants.maxDistance);
        // config.softLimit.reverseSoftLimit(ElevatorConstants.minDistance);

        // config.closedLoop.p(10);
        // config.closedLoop.i(0);
        // config.closedLoop.d(0);

        config.smartCurrentLimit(ElevatorConstants.elevatorCurrentLimits);
        config.voltageCompensation(12);
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);
        rightElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.follow(ElevatorConstants.rightDeviceID, true);

        leftElevatorMotor.setCANTimeout(200);
        leftElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVoltage(double voltage) {
        rightElevatorMotor.setVoltage(voltage);
    }

    @Override
    public void setGoal(double height) {
        if(height != goal.position) {
            setpoint = new TrapezoidProfile.State(rightEncoder.getPosition(), rightEncoder.getVelocity());
            goal = new TrapezoidProfile.State(height, 0);
        }
    }

    @Override
    public void updateMotionProfile() {
        // double prevVelocity = setpoint.velocity;
        setpoint = profile.calculate(0.02, setpoint, goal);
        // double acceleration = (setpoint.velocity - prevVelocity) / 0.02;
        // double ffvolts = ffmodel.calculate(setpoint.velocity, acceleration);
        double ffvolts = ffmodel.calculate(setpoint.velocity);
        double pidvolts = controller.calculate(rightEncoder.getPosition(), setpoint.position);
        setVoltage(ffvolts + pidvolts);
    }

    @Override
    public void hold() {
        double ffvolts = ffmodel.calculate(0);
        setVoltage(ffvolts);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorRightCurrent = rightElevatorMotor.getOutputCurrent();
        inputs.elevatorRightAppliedVolts = rightElevatorMotor.getAppliedOutput() * rightElevatorMotor.getBusVoltage();
        inputs.elevatorRightPositionMeters = rightEncoder.getPosition();
        inputs.elevatorRightVelocityMetersPerSecond = rightEncoder.getVelocity();

        inputs.elevatorLeftCurrent = leftElevatorMotor.getOutputCurrent();
        inputs.elevatorLeftAppliedVolts = leftElevatorMotor.getAppliedOutput() * leftElevatorMotor.getBusVoltage();
        inputs.elevatorLeftPositionMeters = leftEncoder.getPosition();
        inputs.elevatorLeftVelocityMetersPerSecond = leftEncoder.getVelocity();

        inputs.setpointVelocity = setpoint.velocity;
        inputs.goalHeight = goal.position;
        inputs.setpointPosition = setpoint.position;
    }

    @Override
    public void moveElevator(double speed) {
        rightElevatorMotor.set(speed);
    }
}
    
