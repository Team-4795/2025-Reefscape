package frc.robot.subsystems;

public class ClimberConstants {
    
    public static final int ClimberID = 9;

    public static final double gearing = 250;

    public static final int currentLimit = 80;
    public static final boolean isInverted = false;

    public static final double deployPosition = Math.PI/2;
    public static final double climbPositiont = 0;
    public static final double voltageCompensation = 12;

    public class SimConstants {
        public static final double length = 1;
        public static final double minAngle = 0;
        public static final double maxAngle = 0;
        public static final boolean gravity = true;
        public static final double initAngle = 0;
        
    }

}
