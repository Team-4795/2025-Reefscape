package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;

public class DriveConstants {

    public static final double MAX_LINEAR_SPEED = Units.metersToFeet(5);
    public static final double TRACK_LENGTH = Units.inchesToMeters(21.25);
    public static final double TRACK_WIDTH = Units.inchesToMeters(21.25);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(DriveConstants.TRACK_LENGTH / 2.0,
            DriveConstants.TRACK_WIDTH / 2.0);
    public static final double MAX_ANGULAR_SPEED = Units.feetToMeters(DriveConstants.MAX_LINEAR_SPEED)
            / DRIVE_BASE_RADIUS;
    public static final double TranslationKS = 0.014;
    public static final double TranslationKV = 0.134;
    public static final double TranslationKP = 0.1;
    public static final double TranslationKI = 0.0;
    public static final double TranslationKD = 0.0;
    public static final double RotationKP = 5.0;
    public static final double RotationKI = 0.0;
    public static final double RotationKD = 0.0;
    public static final double WHEEL_RADIUS = Units.inchesToMeters(1.5);
    public static final double TurnGearing = 12.155818;
    public static final double DriveGearing = 6.11;
    public static final int turnCurrentLimit = 60; // May need to change later
    public static final int driveCurrentLimit = 60;
    public static final int pigeonID = 20;
    public static final double turningFactor = 2*Math.PI;

}
