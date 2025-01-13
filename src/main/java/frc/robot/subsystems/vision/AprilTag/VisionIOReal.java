package frc.robot.subsystems.vision.AprilTag;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class VisionIOReal implements VisionIO {
    List<PhotonPipelineResult> result;

    PhotonCamera camera;

    PhotonPoseEstimator poseEstimator;
    PhotonTargetSortMode sortMode;
    PhotonTrackedTarget target;

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
    public void switchPipeline() {
        // Change later
        if(camera.getPipelineIndex() == 0)
            camera.setPipelineIndex(1);

        else if(camera.getPipelineIndex() == 1)
            camera.setPipelineIndex(0);
    }

    @Override
    public void targetLeftReef() {
        sortMode = PhotonTargetSortMode.Leftmost;
    }

    @Override
    public void targetRightReef() {
        sortMode = PhotonTargetSortMode.Rightmost;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        if(camera.getPipelineIndex() == 0) { // April tag pipeline index? Change later
            result = camera.getAllUnreadResults();

            for(int i = 0; i < result.size(); i++) {
                poseEstimator.update(result.get(i), camera.getCameraMatrix(), camera.getDistCoeffs()).ifPresentOrElse((pose) -> {
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

        else if(camera.getPipelineIndex() == 1) { // Reef detection pippeline index? Change later
            result = camera.getAllUnreadResults();

            for(int i = 0; i < result.size(); i++)
            {
                result.get(i).targets.sort(sortMode.getComparator());
                target = result.get(i).getBestTarget();

                inputs.roll = target.getSkew();
                inputs.pitch = target.getPitch();
                inputs.yaw = target.getYaw();
                inputs.area = target.getArea();
            }
        }
    }
}
