package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;


public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

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

        setDefaultCommand(Commands.run(() -> io.hold(), this));
    }
    
    public boolean atGoal(double goal) {
        return MathUtil.isNear(goal, getPosition(), ElevatorConstants.GOAL_TOLERANCE);
    }

    public Command setGoal(double goal){
        return Commands.runOnce(() -> io.setGoal(goal), this)
        .andThen(Commands.run(() -> io.updateMotionProfile(), this));
    }

    public  void moveElevator(double speed) {
        io.moveElevator(speed);
    }

    public double getGoalHeight() {
        return inputs.goalHeight;
    }

    public double getPosition(){
        return inputs.elevatorRightPositionMeters; //change later
    }

    public double getVelocity() {
        return inputs.elevatorRightVelocityMetersPerSecond; //change later
        
    }

    public boolean isNearGoal() {
        return MathUtil.isNear(inputs.goalHeight, inputs.elevatorRightPositionMeters, 0.01);
    }

    public void reset() {
        io.resetEncoders();
        this.setGoal(getPosition());
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }
}