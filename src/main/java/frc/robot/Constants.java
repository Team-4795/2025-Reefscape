package frc.robot;

public final class Constants {
    public static final class IntakeSetpoints{
        //placeholders
        public static final double intake = -0.85;
        public static final double reverse = 0.7;
    }
    
    public static final Mode currentMode = Mode.fromState();
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
}
