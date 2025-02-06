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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class ElevatorIOReal implements ElevatorIO {

    private SparkFlex rightElevatorMotor = new SparkFlex(ElevatorConstants.rightDeviceID, MotorType.kBrushless);
    private SparkFlex leftElevatorMotor = new SparkFlex(ElevatorConstants.leftDeviceID, MotorType.kBrushless); 

    private RelativeEncoder leftEncoder = leftElevatorMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightElevatorMotor.getEncoder();

 //   private AbsoluteEncoder leftAbsoluteEncoder = leftElevatorMotor.getAbsoluteEncoder();

    private final ElevatorFeedforward ffmodel = new ElevatorFeedforward(0, 0, 0);
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
    private final PIDController controller = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kP);
    private final TrapezoidProfile profile = new TrapezoidProfile(constraints);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();


    private double minPositionMeters = 0.0;
    private double maxPositionMeters = 0.7112;

    private SparkFlexConfig config = new SparkFlexConfig();
     
    private double inputVolts = 0.0;

    //final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private boolean isEnabled;
    private boolean hasPlayed = false;

    //zero stuff 
    public void zeroElevator(){
    //    double AbsolutePosition = leftAbsoluteEncoder.getPosition();
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    
    
    public ElevatorIOReal(){
        leftElevatorMotor.clearFaults();
        rightElevatorMotor.clearFaults();
        config.encoder.positionConversionFactor(Units.inchesToMeters(11.0/18.0));
        config.encoder.velocityConversionFactor(Units.inchesToMeters(11.0/18.0)/60.0);    
        config.smartCurrentLimit(ElevatorConstants.elevatorCurrentLimits);
        config.idleMode(IdleMode.kBrake);

        rightElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.follow(ElevatorConstants.rightDeviceID, true);

  
 //   config.absoluteEncoder.positionConversionFactor(Units.inchesToMeters(11 / 18));             use these later
 //   config.absoluteEncoder.velocityConversionFactor(Units.inchesToMeters(11 / 18) / 60);        use these later
    leftElevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

}

    @Override
    public void setVoltage(double voltage) {
        // double currentPosition = leftEncoder.getPosition(); // Relative encoder position
        // if (currentPosition <= minPositionMeters && voltage < 0) {
        //     voltage = 0;
        // } else if (currentPosition >= maxPositionMeters && voltage > 0) {
        //     voltage = 0; 
        // }
        inputVolts = voltage; 
        rightElevatorMotor.setVoltage(voltage);
    }


    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        //might want to separate by motor or you can average if they don't need to be ran in reverse

        if(!inputs.openLoop){
            setVoltage(ffmodel.calculate(setpoint.velocity) + controller.calculate(setpoint.velocity));
            setpoint = profile.calculate(0.02, setpoint, goal);
        }
        inputs.elevatorRightCurrent = rightElevatorMotor.getOutputCurrent();
        inputs.elevatorRightAppliedVolts = rightElevatorMotor.getAppliedOutput() * leftElevatorMotor.getBusVoltage();
        inputs.elevatorRightPositionMeters = rightEncoder.getPosition();
        inputs.elevatorRightVelocityMetersPerSecond = rightEncoder.getVelocity();
       // inputs.elevatorRightMotorPositionMeters = leftAbsoluteEncoder.getPosition();
       // inputs.elevatorRightMotorVelocityMetersPerSecond = leftAbsoluteEncoder.getVelocity();     will use absolute encoder later
        inputs.elevatorRightInputVolts = inputVolts;


        inputs.elevatorLeftCurrent = leftElevatorMotor.getOutputCurrent();
        inputs.elevatorLeftAppliedVolts = leftElevatorMotor.getAppliedOutput() * leftElevatorMotor.getBusVoltage();
        inputs.elevatorLeftPositionMeters = leftEncoder.getPosition();
        inputs.elevatorLeftVelocityMetersPerSecond = leftEncoder.getVelocity();
        inputs.elevatorLeftInputVolts = inputVolts;

        Logger.recordOutput("Elevator/Setpoint/Position", setpoint.position);
        Logger.recordOutput("Elevator/Setpoint/Velocity", setpoint.velocity);
        
        // log goal after updating
        Logger.recordOutput("Elevator/Goal/Position", goal.position);
        Logger.recordOutput("Elevator/Goal/Velocity", goal.velocity);

        Logger.recordOutput("Elevator/Left Motor", leftEncoder.getPosition());
        Logger.recordOutput("Elevator/Right Motor", rightEncoder.getPosition());


    }

    @Override
    public void moveElevator(double speed) {
        rightElevatorMotor.set(speed);
    }

    public void setSoftLimits(double min, double max) {
        this.minPositionMeters = min;
        this.maxPositionMeters = max;
    }
    }
    
