package frc.robot.subsystems.state;

public enum State {
    DYNAMIC(StateConstants.DYNAMIC),
    L4(StateConstants.L4),
    L3(StateConstants.L3),
    L2(StateConstants.L2),
    STOW(StateConstants.STOW),
    VSTOW(StateConstants.VSTOW),
    LOW_ALGAE(StateConstants.LOW_ALGAE),
    PROCESSOR(StateConstants.PROCESSOR),
    FORWARD_NET(StateConstants.FORWARD_NET),
    BACKWARD_NET(StateConstants.BACKWARD_NET),
    HIGH_ALGAE(StateConstants.HIGH_ALGAE);

    public Setpoint setpoint;

    private State(Setpoint setpoint) {
        this.setpoint = setpoint;
    }

    public static class Setpoint {
        public Double armAngle;
        public Double elevatorHeight;
        public Double intakeSpeed;
        public Double wristAngle;

        public Setpoint(Double armAngle, Double elevatorHeight, Double intakeSpeed, Double wristAngle) {
            this.armAngle = armAngle;
            this.elevatorHeight = elevatorHeight;
            this.intakeSpeed = intakeSpeed;
            this.wristAngle = wristAngle;
        }
    }
}
