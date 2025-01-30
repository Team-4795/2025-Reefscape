package frc.robot.subsystems.elevator;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


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
        Logger.processInputs(getName(), inputs);

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


    public Commands setGoal(double goal){
        
        return Commands.runOnce(() -> setOpenLoop(false)).andThen(() -> io.setGoal(goal), this);
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

    
    @Override
    public void setGoal(double angle) {
        goal = new TrapezoidProfile.State(goal, 0);
    }

}






