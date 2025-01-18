package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Constants {
    public static Mode currentMode = Mode.SIM;
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static class OIConstants {
        public static final CommandXboxController driverController = new CommandXboxController(0);
    }
}
