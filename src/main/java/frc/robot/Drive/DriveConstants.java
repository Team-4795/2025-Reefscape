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
    
}

    
