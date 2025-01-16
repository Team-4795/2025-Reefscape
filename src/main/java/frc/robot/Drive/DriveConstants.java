package frc.robot.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;

public class DriveConstants {
    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        REAL,

        SIM,

        REPLAY,
    }
}

    
