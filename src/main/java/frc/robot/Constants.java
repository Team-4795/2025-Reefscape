package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
public class Constants {
    public static Mode currentMode = Mode.SIM;
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final class OIConstants{
        public static final double KAxisDeadband = 0.1;
        public static final  CommandXboxController driverController = new CommandXboxController(0);
        public static final  CommandXboxController operatorController = new CommandXboxController(1);
    }
}
