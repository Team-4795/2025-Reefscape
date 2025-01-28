package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class ElevatorIOReal implements ElevatorIO {

    private SparkFlex rightElevatorMotor = new SparkFlex(ElevatorConstants.rightDeviceID, MotorType.kBrushless);
    private SparkFlex leftElevatorMotor = new SparkFlex(ElevatorConstants.leftDeviceID, MotorType.kBrushless); 

    private RelativeEncoder leftEncoder = leftElevatorMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightElevatorMotor.getEncoder();

    private AbsoluteEncoder leftAbsoluteEncoder = leftElevatorMotor.getAbsoluteEncoder();

    private final ElevatorFeedforward ffmodel = new ElevatorFeedforward(0, 0, 0);
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1, 1);
    private final PIDController controller = new PIDController(1, 0, 0);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();


    private double minPositionMeters = 0.0;
    private double maxPositionMeters = 0.7112;

    private SparkFlexConfig config = new SparkFlexConfig();
     
    private double inputVolts = 0.0;

    //final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private boolean isEnabled;
    private boolean hasPlayed = false;

    //zero stuff 
    public void zeroElevator(){
        double AbsolutePosition = leftAbsoluteEncoder.getPosition();
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    
    
    public ElevatorIOReal(){
    
    config.smartCurrentLimit(ElevatorConstants.elevatorCurrentLimits);
    rightElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    config.follow(ElevatorConstants.rightDeviceID);
    config.inverted(true);
    config.absoluteEncoder.positionConversionFactor(Units.inchesToMeters(11 / 8));
    config.absoluteEncoder.velocityConversionFactor(Units.inchesToMeters(11 / 8) / 60);
    leftElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    leftElevatorMotor.clearFaults();
    rightElevatorMotor.clearFaults();
}

    @Override
    public void setVoltage(double voltage) {
        double currentPosition = leftEncoder.getPosition(); // Relative encoder position
        if (currentPosition <= minPositionMeters && voltage < 0) {
            voltage = 0; // Prevent moving below the minimum position
        } else if (currentPosition >= maxPositionMeters && voltage > 0) {
            voltage = 0; // Prevent moving above the maximum position
        }
        inputVolts = voltage; 
        rightElevatorMotor.setVoltage(voltage);
    }


    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        //might want to separate by motor or you can average if they don't need to be ran in reverse

        if(!inputs.openLoop){
            setVoltage(ffmodel.calculate(setpoint.velocity, setpoint.position) + controller.calculate(setpoint.velocity));
            setpoint = profile.calculate(0.1, setpoint, setpoint);
        }
        inputs.elevatorCurrent = leftElevatorMotor.getOutputCurrent();
        inputs.elevatorAppliedVolts = leftElevatorMotor.getAppliedOutput() * leftElevatorMotor.getBusVoltage();

        //inputs.elevatorMotorPositionMeters = (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
        inputs.elevatorMotorPositionMeters = leftAbsoluteEncoder.getPosition();
        //inputs.elevatorMotorVelocityMetersPerSecond = (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
        inputs.elevatorMotorVelocityMetersPerSecond = leftAbsoluteEncoder.getVelocity();
        inputs.elevatorInputVolts = inputVolts;

        Logger.recordOutput("Elevator/Left Motor", leftEncoder.getPosition());
        Logger.recordOutput("Elevator/Right Motor", rightEncoder.getPosition());


    }

    @Override
    public void moveElevator(double speed) {

        double currentPosition = leftEncoder.getPosition();
        if ((currentPosition <= minPositionMeters && speed < 0) || 
            (currentPosition >= maxPositionMeters && speed > 0)) {
            speed = 0; 
        }
        rightElevatorMotor.set(speed);
    }

    public void setSoftLimits(double min, double max) {
        this.minPositionMeters = min;
        this.maxPositionMeters = max;
    }
    }
    
