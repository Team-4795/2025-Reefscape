package frc.robot;

import frc.robot.subsystems.arm.CurrentLimits;

public class Constants {
    public static Mode currentMode = Mode.REAL;
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }
    public static class CurrentLimits {
        public static int armKraken = 40;
        //for now
    }
}
