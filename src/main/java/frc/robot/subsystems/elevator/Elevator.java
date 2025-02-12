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

    public Command setGoal(double goal){
        return Commands.runOnce(() -> io.setGoal(goal), this)
        .andThen(Commands.run(() -> io.updateMotionProfile(), this))
        .withTimeout(2.5);
    }

    public  void moveElevator(double speed) {
        io.moveElevator(speed);
    }

    public double getPosition(){
        return inputs.elevatorLeftPositionMeters; //change later
    }

    public double getVelocity() {
        return inputs.elevatorLeftVelocityMetersPerSecond; //change later
        
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