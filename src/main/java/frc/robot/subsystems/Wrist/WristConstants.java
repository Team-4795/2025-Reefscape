package frc.robot.subsystems.Wrist;

import edu.wpi.first.math.util.Units;

public class WristConstants {

    public final static int id = 19;
    
    public final static int currentLimit = 60;
    public final static boolean isInverted = false;
    
    public final static double gearing = 30;
    public final static double Coral_kP = 1;
    public final static double Coral_kI = 0;
    public final static double Coral_kD = 0;

    public final static double Coral_kS = 0;
    public final static double Coral_kG = 0;
    public final static double Coral_kV = 0;
    public final static double Coral_KA = 0;


    public final static double Algae_kP = 1;
    public final static double Algae_kI = 0;
    public final static double Algae_kD = 0;

    public final static double Algae_kS = 0;
    public final static double Algae_kG = 0;
    public final static double Algae_kV = 0;
    public final static double Algae_KA = 0;

    public final static double maxV = 2;
    public final static double maxA = 1;
    public final static double minPosition = Units.degreesToRadians(-60); //1
    public final static double maxPosition = Units.degreesToRadians(60); //43

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
