package frc.robot.subsystems.Wrist;

import edu.wpi.first.math.util.Units;

public class WristConstants {

    public static final int id = 19;
    
    public static final int currentLimit = 60;
    public static final boolean isInverted = true;
    
    public static final double gearing = 30;
    public static final double Coral_kP = 5;
    public static final double Coral_kI = 0;
    public static final double Coral_kD = 0;

    public static final double Coral_kS = 1;
    public static final double Coral_kG = 0;
    public static final double Coral_kV = 0.25;
    public static final double Coral_KA = 0;


    public static final double Algae_kP = 1;
    public static final double Algae_kI = 0;
    public static final double Algae_kD = 0;

    public static final double Algae_kS = 1;
    public static final double Algae_kG = 0;
    public static final double Algae_kV = 0;
    public static final double Algae_KA = 0;

    public static final double maxV = 5;
    public static final double maxA = 8;
    public static final double minPosition = Units.degreesToRadians(-180); //1
    public static final double maxPosition = Units.degreesToRadians(360); //43

    public static final double voltageCompensation = 12.0;

    public static final double CORAL_L4_SETPOINT = 0.1096;
    public static final double CORAL_L3_SETPOINT = -0.1528117607831955;
    public static final double CORAL_L2_SETPOINT = 0;
    public static final double CORAL_L1_SETPOINT = 1.831;
    public static final double offset = 0;
    public static final double VFBAngle = 0.2468;
    public static final double FOWARD_NET_SETPOINT = -1.7;
    public static final double BACKWARD_NET_SETPOINT = -2.97340989112854 + Units.degreesToRadians(3);
    public static final double intakePosition = 0.24;
    public static final double stowPosition = 1;

    public static double GOAL_TOLERANCE = 0.1; //change later
    
    public static class Sim {
        public static final double length = 1;
        public static final double minAngle = -Math.PI;
        public static final double maxAngle = Math.PI;
        public static final boolean gravity = false;
        public static final double initAngle = 0;
        public static final double SIMkP = 1;
        public static final double SIMkI = 0;
        public static final double SIMkD = 0;
        public static final double SIMkS = 0.5;
        public static final double SIMkG = 0.5;
        public static final double SIMkV = 1;
        public static final double SIMKA = 0;
        public static final double INIT_ANGLE = Units.degreesToRadians(1);
    }
}
