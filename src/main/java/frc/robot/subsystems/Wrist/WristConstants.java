package frc.robot.subsystems.Wrist;

import edu.wpi.first.math.util.Units;

public class WristConstants {

    public static final int id = 19;
    
    public static final int currentLimit = 60;
    public static final boolean isInverted = true;
    
    public static final double gearing = 30;
    public static final double Coral_kP = 1.3;
    public static final double Coral_kI = 0;
    public static final double Coral_kD = 0;

    public static final double Coral_kS = 0.5;
    public static final double Coral_kG = 0;
    public static final double Coral_kV = 0.5;
    public static final double Coral_KA = 0;


    public static final double Algae_kP = 1;
    public static final double Algae_kI = 0;
    public static final double Algae_kD = 0;

    public static final double Algae_kS = 1;
    public static final double Algae_kG = 0;
    public static final double Algae_kV = 0;
    public static final double Algae_KA = 0;

    public static final double maxV = 3;
    public static final double maxA = 5;
    public static final double minPosition = Units.degreesToRadians(-60); //1
    public static final double maxPosition = Units.degreesToRadians(180); //43

    public static final double voltageCompensation = 12.0;

    public static final double offset = 0;
    public static final double VFBAngle = Units.degreesToRadians(18);
    public static final double NET_SETPOINT = Units.degreesToRadians(90);
    public static final double intakePosition = 1;
    public static final double stowPosition = 3;

    public static double GOAL_TOLERANCE = 0.1; //change later
    
    public static class Sim {
        public static final double length = 1;
        public static final double minAngle = 0;
        public static final double maxAngle = 0;
        public static final boolean gravity = true;
        public static final double initAngle = 0;
        public static final double SIMkP = 1;
        public static final double SIMkI = 0;
        public static final double SIMkD = 0;
        public static final double SIMkS = 0;
        public static final double SIMkG = 0;
        public static final double SIMkV = 0;
        public static final double SIMKA = 0;
        public static final double INIT_ANGLE = Units.degreesToRadians(1);
    
    }
   
}
