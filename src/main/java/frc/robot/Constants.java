package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class Constants {
    public static final class IntakeSetpoints{
        //placeholders
        public static final double intake = -0.6;
        public static final double slow = -0.3;
        public static final double reverse = 0.7;
    }
    
    public static final Mode currentMode = Mode.SIM;
    public static enum Mode {
        REAL,
    
        SIM,
    
        REPLAY;
    
        static Mode fromState() {
          if (Robot.isReal()) {
            return REAL;
          } else {
            return SIM;
          }
        }
      }
    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
    }

    public static final class OIConstants{
      public static final double KAxisDeadband = 0.1;
      public static final  CommandXboxController driverController = new CommandXboxController(0);
      public static final  CommandXboxController operatorController = new CommandXboxController(1);
  }
}

