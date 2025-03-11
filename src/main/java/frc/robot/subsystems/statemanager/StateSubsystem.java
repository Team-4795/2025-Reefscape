package frc.robot.subsystems.statemanager;

public interface StateSubsystem {
    public boolean atGoalState();
    public void setGoalState(double... state);
    public double[] getState();
}
