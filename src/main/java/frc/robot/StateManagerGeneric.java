package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.AutoLogOutput;

// This one just takes a list of generic runnables and calls them when the state is changed.
// Pros: Resuable, doesn't require inputs for every subsystem
// Cons: doesn't ensure that subsystem inputs are applied. States are not necessarily unique for every subsystem.
// defining states is also pretty ugly 

public class StateManagerGeneric {
    private Map<String, State> stateMap = new HashMap<String, State>(){};
    private String currentState;

    public static class State {
        private Runnable[] setters;

        public State(Runnable ...setters) {
            this.setters = setters;
        }

        public void applyState() {
            for(Runnable setter : setters) {
                setter.run();
            }
        }

        public Runnable[] getSetters() {
            return setters;
        }
    }

    public void addState(String name, State state) {
        stateMap.put(name, state);
    }

    public void setState(String name) {
        currentState = name;
        for(Runnable setter : stateMap.get(name).getSetters()) {
            setter.run();
        }
    }

    @AutoLogOutput
    public String getState() {
        return currentState;
    }
}