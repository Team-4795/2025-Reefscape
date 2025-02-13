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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

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
        sortMode = PhotonTargetSortMode.Largest;
        
    } 

    @Override
    public void setReferencePose(Pose2d reference) {
        poseEstimator.setReferencePose(reference);
    }

    @Override
    public void switchPipeline() {
        if(camera.getPipelineIndex() == VisionConstants.aprilTagPipelineID && camera.getName().equals(VisionConstants.cameraIds[0]))
            camera.setPipelineIndex(VisionConstants.reefDetectionPipelineID);

        else if(camera.getPipelineIndex() == VisionConstants.reefDetectionPipelineID && camera.getName().equals(VisionConstants.cameraIds[0]))
            camera.setPipelineIndex(VisionConstants.aprilTagPipelineID);
    }

    public Pose2d getBestReefPos() {
        Translation2d odometry = Drive.getInstance().getPose().getTranslation();
        Pose2d bestPose = new Pose2d();

        if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red))
        {

            double distance = VisionConstants.redReefScoringPoses[0].getTranslation().getDistance(odometry);

            for(int i = 1; i < VisionConstants.redReefScoringPoses.length; i++)
            {
                if(VisionConstants.redReefScoringPoses[i].getTranslation().getDistance(odometry) < distance)
                {
                    distance = VisionConstants.redReefScoringPoses[i].getTranslation().getDistance(odometry);
                    bestPose = VisionConstants.redReefScoringPoses[i];
                }
            }
            
        }

        if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue))
        {
            double distance = VisionConstants.blueReefScoringPoses[0].getTranslation().getDistance(odometry);

            for(int i = 1; i < VisionConstants.blueReefScoringPoses.length; i++)
            {
                if(VisionConstants.blueReefScoringPoses[i].getTranslation().getDistance(odometry) < distance)
                {
                    distance = VisionConstants.blueReefScoringPoses[i].getTranslation().getDistance(odometry);
                    bestPose = VisionConstants.blueReefScoringPoses[i];
                }
            }
        }

        return bestPose;
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
    public void targetAprilTag() {
        sortMode = PhotonTargetSortMode.Largest;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.pipelineIndex = camera.getPipelineIndex();
        inputs.sortMode = sortMode.toString();

        result = camera.getAllUnreadResults();

        if(camera.getPipelineIndex() == VisionConstants.aprilTagPipelineID) { // April tag pipeline index? Change later
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
            
            inputs.reefPose = getBestReefPos();
        }

        else if(camera.getPipelineIndex() == VisionConstants.reefDetectionPipelineID) { // Reef detection pippeline index? Change later
         
            for(int i = 0; i < result.size(); i++)
            {
                result.get(i).targets.sort(sortMode.getComparator());
                target = result.get(i).getBestTarget();

                if(target != null) {
                    inputs.roll = target.getSkew();
                    inputs.pitch = target.getPitch();
                    inputs.yaw = target.getYaw();
                    inputs.area = target.getArea();
                }
            }
        }
    }
}
