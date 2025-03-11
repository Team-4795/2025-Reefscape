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
import frc.robot.util.Util;

public class StateManagerV2 extends SubsystemBase {
    private Arm arm = Arm.getInstance();
    private Elevator elevator = Elevator.getInstance();

    private State currentState = new State();
    private State requestedState = new State();

    private static StateManagerV2 instance;

    public static void initalize() {
        instance = new StateManagerV2();
    }

    public static StateManagerV2 getInstance() {
        return instance;
    }

    public static class State {
        public double armAngle = 0.0;
        public double elevatorHeight = 0.0;
    }

    public static class StateRequest {
        public Double armAngle = null;
        public Double elevatorHeight = null;

        public StateRequest withArmAngle(double armAngle) {
            this.armAngle = armAngle;
            return this;
        }

        public StateRequest withElevatorHeight(double elevatorHeight) {
            this.elevatorHeight = elevatorHeight;
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
    
    public void requestState(StateRequest stateRequest) {
        Util.checkNull(stateRequest.armAngle, (value) -> requestedState.armAngle = value);
        Util.checkNull(stateRequest.elevatorHeight, (value) -> requestedState.elevatorHeight = value);
    }

    public Command stateCommand(StateRequest stateRequest){
        requestState(stateRequest);
        return Commands.parallel(
            Commands.runOnce(() -> arm.setGoal(requestedState.armAngle)),
            Commands.runOnce(() -> elevator.setGoalHeight(requestedState.elevatorHeight))
        ).andThen(Commands.waitUntil(() -> arm.atGoal(requestedState.armAngle) && elevator.atGoal(requestedState.elevatorHeight)));
    }

    @Override
    public void periodic() {
        currentState.armAngle = arm.getAngle();
        currentState.elevatorHeight = elevator.getPosition();

        Logger.recordOutput("StateManager/armCanMove", armCanMove());
        Logger.recordOutput("StateManager/elevatorCanMove", elevatorCanMove());
    }
}
