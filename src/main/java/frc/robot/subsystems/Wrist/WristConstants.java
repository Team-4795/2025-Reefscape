package frc.robot.subsystems.Wrist;

public class WristConstants {
    // all placeholders
    public final static int id = 0;
    
    public final static int stallLimit = 30;
    public final static int freeLimit = 30;
    
    public final static double kP = 0;
    public final static double kI = 0;
    public final static double kD = 0;

    public final static double kS = 0;
    public final static double kG = 0;
    public final static double kV = 0;

    public final static double maxV = 2;
    public final static double maxA = 1;

    public final static double offset = 0;
    public final static double gearing = (64.0 / 18.0) * (60.0 / 7.0);
    
    // sim constants
    public static class Sim {
        public static final double length = 1;
        public static final double minAngle = 0;
        public static final double maxAngle = 0;
        public static final boolean gravity = true;
        public static final double initAngle = 0;
    }
   
}
