package frc.robot.subsystems.elevator;


import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.ElevatorFeedforward;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;


public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    @AutoLogOutput
    private double leftMovingSpeed = 0.0;

    @AutoLogOutput
    private double rightMovingSpeed = 0.0;

    private double maxDistance = ElevatorConstants.maxDistance;

    private static Elevator instance;

    private  PIDController controller = new PIDController(
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD);

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.ks, ElevatorConstants.kg, ElevatorConstants.kv);

    
    public static Elevator getInstance(){
        return instance;
    }

    public void periodic () {

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
    

    // public void setMovingSpeedRPM(double leftSpeed, double rightSpeed){
    //     leftMovingSpeed = leftSpeed;
    //     rightMovingSpeed = rightSpeed;
    // }


    public void setGoal(double goal){

    }

    public  void moveElevator(double speed) {
        io.moveElevator(speed);
    }

    public double getTruePosition() {
        return getPosition();
    }

    public double getPosition(){
        return maxDistance; //change later
        
    }

    public double getVelocity() {
        return maxDistance; //change later
        
    }

    public void reset() {
        controller.reset();
        io.resetEncoders();
        this.setGoal(getPosition());
    }

    
    }







