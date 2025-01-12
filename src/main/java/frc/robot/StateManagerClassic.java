package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateManagerClassic extends SubsystemBase {
    public record State(String name, BooleanSupplier condition, Double armAngle, Double elevatorPos){};
    private State currentState;

    public State getCurrentState() {
        return currentState;
    }

    public String getStateName() {
        return currentState.name();
    }

    public void setState(State state) {
        if(state.condition.getAsBoolean()) {
            currentState = state;
        }

        checkNull(currentState.elevatorPos, System.out::println);
        checkNull(currentState.armAngle, System.out::println);
    }

    // passes value to consumer if value is not null;
    private void checkNull(Double value, DoubleConsumer consumer) {
        if(value != null) {
            consumer.accept(value);
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput("StateManager/Current State", getStateName());
    }
}
