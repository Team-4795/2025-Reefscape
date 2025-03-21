package frc.robot.subsystems.state;

public enum State {
    DYNAMIC(StateConstants.DYNAMIC),
    L4_DYNAMIC(StateConstants.DYNAMIC),
    L4(StateConstants.L4),
    L3(StateConstants.L3),
    L2(StateConstants.L2),
    STOW(StateConstants.STOW),
    VSTOW(StateConstants.VSTOW),
    LOW_ALGAE(StateConstants.LOW_ALGAE);

    public Setpoint setpoint;

    private State(Setpoint setpoint) {
        this.setpoint = setpoint;
    }

    public static class Setpoint {
        public Double armAngle;
        public Double elevatorHeight;
        public Double intakeSpeed;

        public Setpoint(Double armAngle, Double elevatorHeight, Double intakeSpeed) {
            this.armAngle = armAngle;
            this.elevatorHeight = elevatorHeight;
            this.intakeSpeed = intakeSpeed;
        }
    }
}
