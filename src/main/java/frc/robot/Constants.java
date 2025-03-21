package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.AprilTag.VisionConstants;

public class Constants {
    public static final boolean tuningMode = false;
    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : Mode.SIM;
    public static final boolean visonSimEnabled = true;

    public static final class driveConstants {
        public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    }
    public static enum Mode {
        SIM,
        REAL,
        REPLAY
    }

    public static class OIConstants {
        public static final CommandXboxController driverController = new CommandXboxController(0);
        public static final CommandXboxController operatorController = new CommandXboxController(1);
        public static final double KAxisDeadband = 0.1;  
        public static final double OperatorLAxisDeadband = 0.3;
        public static boolean isScoringLeft = true;
        public static boolean aligned = false; 
        public static boolean inScoringDistance = false;
        public static boolean isReefTagOnly = true;
        public static int autoScoreMode = 4;
    }

    public static enum Gamepiece {
        ALGAE,
        CORAL,
        NONE,
        SIM,
    }

    public static final class FieldConstants {
        public static final double fieldLength = VisionConstants.aprilTagFieldLayout.getFieldLength();
        public static final double fieldWidth = VisionConstants.aprilTagFieldLayout.getFieldWidth();
    }
}
