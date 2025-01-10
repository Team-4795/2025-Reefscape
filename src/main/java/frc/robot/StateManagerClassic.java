package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateManagerClassic extends SubsystemBase {
    public record State(String name, double armAngle, double elevatorPos){};
    private State currentState;

    public State getCurrentState() {
        return currentState;
    }

    public String getStateName() {
        return currentState.name();
    }

    public void setState(State state) {
        currentState = state;

        System.out.println(currentState.armAngle);
        System.out.println(currentState.elevatorPos);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("StateManager/Current State", getStateName());
    }
}
