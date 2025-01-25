package frc.robot.subsystems.arm;

public class ArmConstants {
    public static int CAN_ID = 10;
    public static int CURRENT_LIMIT = 40;

    // need to be tuned
    public static class Sim {
        public static double GEARING = 40;
        public static double MOI = 2.09670337984;
        public static double LENGTH = 0.5;
        public static double MIN_ANGLE = -Math.PI / 6;
        public static double MAX_ANGLE = 2 * Math.PI / 3;
        public static boolean GRAVITY = false;
        public static double INIT_ANGLE = -Math.PI / 6;
    }
}
