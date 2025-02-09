package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    public static final int CAN_ID = 14;
    public static final int CURRENT_LIMIT = 50;
    public static final double ARM_OFFSET = Math.PI/2;

    public static final double CORAL_L1 = Units.degreesToRadians(-34.74);
    public static final double CORAL_L2 = Units.degreesToRadians(-54.1); // with elevator up
    public static final double CORAL_L3 = Units.degreesToRadians(54.1);
    public static final double CORAL_L4 = Units.degreesToRadians(54.61);

    public static final double MAX_VELOCITY = 2;
    public static final double MAX_ACCELERATION = 4;

    // Simulation FF
    public static final double SIMkG = 1.3;
    public static final double SIMkV = 1.00;
    public static final double SIMkA = 0.05;
    public static final double SIMkS = 0.001;

    // Default FF
    public static final double DEFAULTkG = 1.02;
    public static final double DEFAULTkV = 1.01;
    public static final double DEFAULTkA = 0.05;
    public static final double DEFAULTkS = 0.1;

    // Algae FF
    public static final double ALGAEkG = 1.02;
    public static final double ALGAEkV = 1.01;
    public static final double ALGAEkA = 0.05;
    public static final double ALGAEkS = 0.1;

    // Default FF
    public static final double CORALkG = 1.09;
    public static final double CORALkV = 1.01;
    public static final double CORALkA = 0.05;
    public static final double CORALkS = 0.1;

    public static class Sim {
        public static double GEARING = 60;
        public static double MOI = 2.09670337984;
        public static double LENGTH = 0.6604;
        public static double MIN_ANGLE = Units.degreesToRadians(-95);
        public static double MAX_ANGLE = Units.degreesToRadians(55);
        public static boolean GRAVITY = true;
        public static double INIT_ANGLE = -Math.PI / 2;
    }
}
