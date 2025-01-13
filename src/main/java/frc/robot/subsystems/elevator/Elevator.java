package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.controller.PIDController;


public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputs inputs = new ElevatorIOInputs();

    @AutoLogOutput
    private double leftMovingSpeed = 0.0;

    @AutoLogOutput
    private double rightMovingSpeed = 0.0;

    private double maxDistance = ElevatorConstants.maxDistance;

    private static Elevator instance;
    
    public static Elevator getInstance(){
        return instance;
    }

    public static Elevator initialize(ElevatorIO io){
        if(instance == null){
            instance = new Elevator(io);
        }
        return instance;
    }

    private Elevator(ElevatorIO elevatorIO){
        io = elevatorIO;
        io.updateInputs(inputs);
    }

    public void setMovingSpeedRPM(double leftSpeed, double rightSpeed){
        leftMovingSpeed = leftSpeed;
        rightMovingSpeed = rightSpeed;
    }

    private ElevatorFeedforward feedFoward = new ElevatorFeedforward(rightMovingSpeed, maxDistance, leftMovingSpeed);

    private  PIDController controller = new PIDController(
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD,
        ElevatorConstants.constraints);

    public void setGoal(double goal){

    }

    private void setElevatorMode(Elevator mode){
        
    }

    public double getTruePosition() {
        return getPosition();
    }

    public double getVelocity() {
        return inputs.elevatorMotorVelocityRadPerSec;
    }

    public void reset() {
        controller.reset(getPosition());
        io.resetEncoders();
        this.setGoal(getPosition());
    }

    }






