package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
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

    private SparkClosedLoopController controller = rightElevatorMotor.getClosedLoopController();

 //   private AbsoluteEncoder leftAbsoluteEncoder = leftElevatorMotor.getAbsoluteEncoder();

    private final ElevatorFeedforward ffmodel = new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv);
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
    // private final PIDController controller = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
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

        // config.softLimit.forwardSoftLimitEnabled(true);
        // config.softLimit.reverseSoftLimitEnabled(true);
        // config.softLimit.forwardSoftLimit(ElevatorConstants.maxDistance);
        // config.softLimit.reverseSoftLimit(ElevatorConstants.minDistance);

        config.closedLoop.p(10);
        config.closedLoop.i(0);
        config.closedLoop.d(0);

        config.smartCurrentLimit(ElevatorConstants.elevatorCurrentLimits);
        config.idleMode(IdleMode.kBrake);
        config.inverted(true);
        rightElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.follow(ElevatorConstants.rightDeviceID, true);
    
        //   config.absoluteEncoder.positionConversionFactor(Units.inchesToMeters(11 / 18));             use these later
        //   config.absoluteEncoder.velocityConversionFactor(Units.inchesToMeters(11 / 18) / 60);        use these later
        leftElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVoltage(double voltage) {
        controller.setReference(voltage, ControlType.kVoltage);
    }

    @Override
    public void setGoal(double height) {
        setpoint = new TrapezoidProfile.State(leftEncoder.getPosition(), leftEncoder.getVelocity());
        goal = new TrapezoidProfile.State(height, 0);
    }

    @Override
    public void updateMotionProfile() {
        double prevVelocity = setpoint.velocity;
        setpoint = profile.calculate(0.02, setpoint, goal);
        double acceleration = (setpoint.velocity - prevVelocity) / 0.02;
        double ffvolts = ffmodel.calculate(setpoint.velocity, acceleration);
        controller.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffvolts);
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
       // inputs.elevatorRightMotorPositionMeters = leftAbsoluteEncoder.getPosition();
       // inputs.elevatorRightMotorVelocityMetersPerSecond = leftAbsoluteEncoder.getVelocity();     will use absolute encoder later


        inputs.elevatorLeftCurrent = leftElevatorMotor.getOutputCurrent();
        inputs.elevatorLeftAppliedVolts = leftElevatorMotor.getAppliedOutput() * leftElevatorMotor.getBusVoltage();
        inputs.elevatorLeftPositionMeters = leftEncoder.getPosition();
        inputs.elevatorLeftVelocityMetersPerSecond = leftEncoder.getVelocity();

        inputs.setpointVelocity = setpoint.velocity;
        inputs.goalHeight = goal.position;
    }

    @Override
    public void moveElevator(double speed) {
        rightElevatorMotor.set(speed);
    }
}
    
