package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.vision.AprilTag.VisionConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
public class Constants {
    public static Mode currentMode = Mode.SIM;
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final class OIConstants{
        public static final double kAxisDeadband = 0.1;
        public static final CommandXboxController driverController = new CommandXboxController(0);
        public static final CommandXboxController operatorController = new CommandXboxController(1);
  }

    public static final class FieldConstants {
        public static final double fieldLength = VisionConstants.aprilTagFieldLayout.getFieldLength();
        public static final double fieldWidth = VisionConstants.aprilTagFieldLayout.getFieldWidth();
  }
}
