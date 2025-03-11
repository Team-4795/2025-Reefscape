package frc.robot.subsystems.statemanager;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.Util;

public class StateManagerV2 extends SubsystemBase {
    private Arm arm = Arm.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Intake intake = Intake.getInstance();

    private State currentState = new State();
    private State requestedState = new State();
    
    private static StateManagerV2 instance;

    public static void initalize() {
        instance = new StateManagerV2();
    }

    public StateManagerV2() {
        armTrigger().onFalse(Commands.runOnce(() -> arm.setGoal(currentState.armAngle)));
        elevatorTrigger().onFalse(Commands.runOnce(() -> elevator.setGoalHeight(currentState.elevatorHeight)));
    }

    public static StateManagerV2 getInstance() {
        return instance;
    }

    public static class State {
        public double armAngle = 0.0;
        public double elevatorHeight = 0.0;
        public double intakeSpeed;
        public String name = "Initial";
    }

    public static class StateRequest {
        public Double armAngle = null;
        public Double elevatorHeight = null;
        public double intakeSpeed;
        public String name = "Default";

        public StateRequest withArmAngle(double armAngle) {
            this.armAngle = armAngle;
            return this;
        }

        public StateRequest withElevatorHeight(double elevatorHeight) {
            this.elevatorHeight = elevatorHeight;
            return this;
        }

        public StateRequest withIntakeSpeed(double intakeSpeed) {
            this.intakeSpeed = intakeSpeed;
            return this;
        }

        public StateRequest and(State state) {
            this.elevatorHeight = state.elevatorHeight;
            this.armAngle = state.armAngle;
            return this;
        }
    }

    public boolean armCanMove() {
        return (elevator.getPosition() < .4 && requestedState.elevatorHeight <= currentState.elevatorHeight) || 
            (requestedState.elevatorHeight >= currentState.elevatorHeight);
    };

    public boolean elevatorCanMove() {
        return (arm.getAngle() > -Math.PI/4 && requestedState.elevatorHeight >= currentState.elevatorHeight)
            || (requestedState.elevatorHeight <= currentState.elevatorHeight);
    };

    public Trigger armTrigger() {
        return new Trigger(() -> armCanMove());
    }

    public Trigger elevatorTrigger() {
        return new Trigger(() -> elevatorCanMove());
    }
    
    public void requestState(StateRequest stateRequest) {
        Util.checkNull(stateRequest.armAngle, (value) -> requestedState.armAngle = value);
        Util.checkNull(stateRequest.elevatorHeight, (value) -> requestedState.elevatorHeight = value);
        Util.checkNull(stateRequest.intakeSpeed, (value) -> requestedState.intakeSpeed = value);
    }

    public State getRequestedState() {
        return requestedState;
    }

    public Command stateCommand(StateRequest stateRequest){
        Command command = 
            Commands.runOnce(() -> requestState(stateRequest))
                .andThen(Commands.waitUntil(() -> arm.atGoal(stateRequest.armAngle) && elevator.atGoal(stateRequest.elevatorHeight)));
        command.addRequirements(this);

        return command;
    }

    @Override
    public void periodic() {
        currentState.armAngle = arm.getAngle();
        currentState.elevatorHeight = elevator.getPosition();

        if(armCanMove()) {
            arm.setGoal(requestedState.armAngle);
        }
        if(elevatorCanMove()){
            elevator.setGoalHeight(requestedState.elevatorHeight);
        }

        intake.setIntakeSpeed(requestedState.intakeSpeed);

        Logger.recordOutput("StateManager/armCanMove", armCanMove());
        Logger.recordOutput("StateManager/elevatorCanMove", elevatorCanMove());
        Logger.recordOutput("StateManager/requestedStateName", requestedState.name);
    }
}
