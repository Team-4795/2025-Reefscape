package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Constants {
    public static final class IntakeSetpoints{
      public static final double intake = -0.6;
      public static final double slow = -0.3;
      public static final double reverse = 0.7;
    }
    public static Mode currentMode = Mode.REAL;
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static class OIConstants {
        public static final CommandXboxController driverController = new CommandXboxController(0);
        public static final CommandXboxController operatorController = new CommandXboxController(1);
        public static final double KAxisDeadband = 0.1;  
    }

    public static enum Gamepiece {
        ALGAE,
        CORAL,
        NONE,
    }
}
