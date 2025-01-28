package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    public static int CAN_ID = 14;
    public static int CURRENT_LIMIT = 40;
    public static double kG = 1.02;
    public static double kV = 1.01;
    public static double kA = 0.05;
    public static double kS = 0.1;

    public static class Sim {
        public static double GEARING = 60;
        public static double MOI = 2.09670337984;
        public static double LENGTH = 0.6604;
        public static double MIN_ANGLE = Units.degreesToRadians(-95);
        public static double MAX_ANGLE = Units.degreesToRadians(55);
        public static boolean GRAVITY = true;
        public static double INIT_ANGLE = -Math.PI / 6;
    }
}
