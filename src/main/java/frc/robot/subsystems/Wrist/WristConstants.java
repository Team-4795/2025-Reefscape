package frc.robot.subsystems.Wrist;

import edu.wpi.first.math.util.Units;

public class WristConstants {

    public final static int id = 20;
    
    public final static int currentLimit = 60;
    public final static boolean isInverted = false;
    
    public final static double gearing = 30;
    public final static double kP = 1;
    public final static double kI = 0;
    public final static double kD = 0;

    public final static double kS = 0;
    public final static double kG = 0;
    public final static double kV = 0;
    public final static double KA = 0;

    public final static double maxV = 2;
    public final static double maxA = 1;
    public final static double minPosition = Units.degreesToRadians(1);
    public final static double maxPosition = Units.degreesToRadians(43);

    public final static double moveUp = 0.2;
    public final static double moveDown = -0.2;
    public final static double voltageCompensation = 12.0;

    public final static double offset = 0;
    public final static double VFBAngle = 14;
    public final static double intakePosition = 1;
    public final static double stowPosition = 3;
    
    public static class Sim {
        public static final double length = 1;
        public static final double minAngle = 0;
        public static final double maxAngle = 0;
        public static final boolean gravity = true;
        public static final double initAngle = 0;
        public final static double SIMkP = 1;
        public final static double SIMkI = 0;
        public final static double SIMkD = 0;
        public final static double SIMkS = 0;
        public final static double SIMkG = 0;
        public final static double SIMkV = 0;
        public final static double SIMKA = 0;
        public final static double INIT_ANGLE = Units.degreesToRadians(1);
    
    }
   
}
