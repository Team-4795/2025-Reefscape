package frc.robot.subsystems.vision.AprilTag;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // CHANGE THESE?
    public static final double fieldBorderMargin = 0.5;
    public static final double zMargin = 0.5;

    public static final double xyStdDevSingleTag = 0.03;
    public static final double xyStdDevMultiTag = 0.018;

    public static final int aprilTagPipelineID = 0;
    public static final int reefDetectionPipelineID = 1;

    public static final int[] redReefIds = {6, 7, 8, 9, 10, 11};
    public static final int[] blueReefIds = {17, 18, 19, 20, 21, 22};

    public static final String[] cameraIds =
    new String[] {
        "Jermaine Coral",
        "Chief Reef",
        "Kendrick LaBarge",
        "Cod Wave"
      };

    // Based on sim. Change once actually mounted
    public static final Transform3d[] cameraPoses =
    new Transform3d[] {
        // Front Camera
        new Transform3d(
            new Translation3d(
                0.3,
                0,
                Units.inchesToMeters(7)), 
            new Rotation3d(
                Units.degreesToRadians(90), 
                Units.degreesToRadians(-30), 
                0)),

        // Backwards Camera
        new Transform3d(
            new Translation3d(
                -Units.inchesToMeters(0),
                0,
                Units.inchesToMeters(39.5)), 
            new Rotation3d(
                Units.degreesToRadians(90), 
                Units.degreesToRadians(-20), 
                Units.degreesToRadians(180))),

        // Left Camera
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(12),
                Units.inchesToMeters(39.5)), 
            new Rotation3d(
                0, 
                Units.degreesToRadians(0), 
                Units.degreesToRadians(135))),

        // Right Camera
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0),
                -Units.inchesToMeters(12),
                Units.inchesToMeters(39.5)), 
            new Rotation3d(
                0, 
                Units.degreesToRadians(0), 
                Units.degreesToRadians(-135)))};

    public static AprilTagFieldLayout aprilTagFieldLayout;

    static {
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
            aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static final double areaCutoff = 11;
    public static final double timeDelay = 0.5;
    public static final double intakeCamOffset = 0.3;
    public static final double degreeTolerance = 6.5;
}