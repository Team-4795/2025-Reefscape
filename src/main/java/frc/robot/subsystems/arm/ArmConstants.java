package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    public static final int CAN_ID = 14;
    public static final int CURRENT_LIMIT = 45;
    public static final double ARM_OFFSET = Math.PI;

    public static final double CORAL_L1 = -1.085;
    public static final double CORAL_L2 = -0.5812;  // (newer setpoint that didnt really work) -0.416;
    public static final double CORAL_L3 = 0.607; //0.4780269;  
    public static final double CORAL_L4 = 0.5091708580679217; 
    public static final double VSTOW = 1.145;//1.342;
    public static final double GOAL_TOLERANCE = 0.03;
    public static final double ALGAE_LOW = 0.3129383325576782 - Units.degreesToRadians(8);
    public static final double ALGAE_HIGH = 0.8301816582679749 - Units.degreesToRadians(6);
    public static final double NET_SETPOINT = Units.degreesToRadians(70);
    public static final double PROCESSOR = 0.0;

    public static final double STOW = -1.81;
    

    public static final double MAX_VELOCITY = 6;
    public static final double MAX_ACCELERATION = 8;

    // Simulation FF
    public static final double SIMkG = 1.3;
    public static final double SIMkV = 1.00;
    public static final double SIMkA = 0.05;
    public static final double SIMkS = 0.001;

    // Default FF
    public static final double DEFAULTkG = 0.35;
    public static final double DEFAULTkV = 1.21;
     public static final double DEFAULTkA = 0.03;
    public static final double DEFAULTkS = 0.25;

    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.12;

    // Algae FF
    public static final double ALGAEkG = 1.02;
    public static final double ALGAEkV = .73;
    public static final double ALGAEkA = 0.08;
    public static final double ALGAEkS = 0.75;

    // Default FF
    public static final double CORALkG = 1.09;
    public static final double CORALkV = 1.01;
    public static final double CORALkA = 0.05;
    public static final double CORALkS = 0.1;

    public static class Sim {
        public static double GEARING = 72;
        public static double MOI = 2.09670337984;
        public static double LENGTH = 0.6604;
        public static double MIN_ANGLE = STOW - Units.degreesToRadians(1);
        public static double MAX_ANGLE = Units.degreesToRadians(92);
        public static boolean GRAVITY = true;
        public static double INIT_ANGLE = -Math.PI / 2;
    }
}
