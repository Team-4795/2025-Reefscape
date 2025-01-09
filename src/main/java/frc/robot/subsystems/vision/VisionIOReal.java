package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionIOReal implements VisionIO {
    List<PhotonPipelineResult> result;

    PhotonCamera camera;

    PhotonPoseEstimator poseEstimator;

    Pose2d speakerPosition;
    double distanceToTarget;

    public VisionIOReal(int camIndex) {
        camera = new PhotonCamera(VisionConstants.cameraIds[camIndex]); 

        poseEstimator = new PhotonPoseEstimator(VisionConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.cameraPoses[camIndex]);
        
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } 

    @Override
    public void setReferencePose(Pose2d reference) {
        poseEstimator.setReferencePose(reference);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        result = camera.getAllUnreadResults();

        for(int i = 0; i < result.size(); i++) {
            poseEstimator.update(result.get(i), null, null).ifPresentOrElse((pose) -> {
                inputs.pose = new Pose3d[] {pose.estimatedPose};
                inputs.timestamp = new double[] {pose.timestampSeconds};
                inputs.tags = pose.targetsUsed.stream().mapToInt((target) -> target.getFiducialId()).toArray();
            }, () -> {
                inputs.pose = new Pose3d[] {};
                inputs.timestamp = new double[] {};
                inputs.tags = new int[] {};
            });
        }
    }
}
