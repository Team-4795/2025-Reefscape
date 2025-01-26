package frc.robot;

public class Constants {
    public static Mode currentMode = Mode.REAL;
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }



    public static final class CurrentLimits {
        public static final int elevator = 30;
    }
}
