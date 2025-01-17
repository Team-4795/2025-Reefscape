package frc.robot.Drive;

import edu.wpi.first.math.util.Units;

public class DriveConstants {
    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        REAL,

        SIM,

        REPLAY,
    }

    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(5);
    public static final double TRACK_LENGTH = Units.inchesToMeters(21.25);
    public static final double TRACK_WIDTH = Units.inchesToMeters(21.25);
    public static final double DRIVE_BASE_RADIUS = 
    Math.hypot(DriveConstants.TRACK_LENGTH / 2.0, DriveConstants.TRACK_WIDTH / 2.0);
    public static final double MAX_ANGULAR_SPEED = DriveConstants.MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    public static final double TranslationKP = 5.0;
    public static final double TranslationKI = 0.0;
    public static final double TranslationKD = 0.0;
    public static final double RotationKP = 5.0;
    public static final double RotationKI = 0.0;
    public static final double RotationKD = 0.0;
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    
    
}

    
