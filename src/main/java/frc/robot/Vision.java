package frc.robot;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    AprilTagFieldLayout tagLayout;
    VisionSystemSim visionSim;
    
    public Vision() {
        visionSim = new VisionSystemSim("main");

        try {
            //tagLayout = AprilTagFieldLayout.loadFromResource("apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape.json");
            tagLayout = AprilTagFieldLayout.loadFromResource("/frc/robot/2025AprilTag.json");
            tagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        }
        catch(IOException e) {
            e.printStackTrace();
        }
        
        visionSim.addAprilTags(tagLayout);

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(78));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.38, 0.2);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        PhotonCamera camera = new PhotonCamera("Jermaine Coral");

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);

        visionSim.addCamera(
        cameraSim, 
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0), 
                Units.inchesToMeters(5.75)), 
            new Rotation3d(
                0, 
                Units.degreesToRadians(0), 
                0)));

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);

        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public void periodic() {
        visionSim.update(
            new Pose3d(
                new Pose2d(13.8,2.9, Rotation2d.fromDegrees(130))));

        visionSim.getDebugField();
    }
}
