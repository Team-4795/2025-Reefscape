package frc.robot.subsystems.vision.AprilTag;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;
public class VisionIOSim implements VisionIO {
    VisionSystemSim visionSim;
    TargetModel  targetModel;
    SimCameraProperties cameraProperties;
    PhotonCamera camera;
    PhotonCameraSim cameraSim;
    int cameraId;

    public VisionIOSim() {
        cameraId = 1;
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(VisionConstants.aprilTagFieldLayout);

        cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(78));
        cameraProperties.setCalibError(0.38, 0.2);
        cameraProperties.setFPS(30);
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);

        camera = new PhotonCamera(VisionConstants.cameraIds[cameraId]);
        // camera.setPipelineIndex(1);

        cameraSim = new PhotonCameraSim(camera, cameraProperties);

        visionSim.addCamera(
            cameraSim, 
            VisionConstants.cameraPoses[cameraId]);
            

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // if (Drive.getInstance() != null) {
            // Pose2d drivePose = Drive.getInstance().getPose();
            // inputs.barbaryFigPose = Optional.of(new EstimatedPose(drivePose, Timer.getFPGATimestamp()));
        // }

        // Uncomment once drive is done
        visionSim.update(Swerve.getInstance().getState().Pose);
        
        // visionSim.update(new Pose2d(12, 12, new Rotation2d(45)));
        visionSim.getDebugField();
    }
}