package frc.robot.subsystems.vision.AprilTag;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
public class VisionIOSim implements VisionIO {
    VisionSystemSim visionSim;
    TargetModel  targetModel;
    SimCameraProperties cameraProperties;
    PhotonCamera camera;
    PhotonCameraSim cameraSim;
    int cameraId;

    public VisionIOSim() {
        if(Constants.photonVisonSimEnabled)
        {
            cameraId = 0;
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

    }

    public Pose2d getBestReefPos() {
        Translation2d odometry = Swerve.getInstance().getState().Pose.getTranslation();
        Pose2d bestPose = new Pose2d();
        Alliance alliance = DriverStation.getAlliance().orElse(null);

        if(alliance != null)
        {
            if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red))
            {
                double distance = VisionConstants.redReefScoringPoses[0].getTranslation().getDistance(odometry);

                for(int i = 0; i < VisionConstants.redReefScoringPoses.length; i++)
                {
                    if(VisionConstants.redReefScoringPoses[i].getTranslation().getDistance(odometry) <= distance)
                    {
                        distance = VisionConstants.redReefScoringPoses[i].getTranslation().getDistance(odometry);
                        bestPose = VisionConstants.redReefScoringPoses[i];
                    }
                }
                
            }

            if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue))
            {
                double distance = VisionConstants.blueReefScoringPoses[0].getTranslation().getDistance(odometry);

                for(int i = 0; i < VisionConstants.blueReefScoringPoses.length; i++)
                {
                    if(VisionConstants.blueReefScoringPoses[i].getTranslation().getDistance(odometry) <= distance)
                    {
                        distance = VisionConstants.blueReefScoringPoses[i].getTranslation().getDistance(odometry);
                        bestPose = VisionConstants.blueReefScoringPoses[i];
                    }
                }
            }
        }

        return bestPose;
    }

    public int getReefTag() {
        Pose2d bestReefPose = getBestReefPos();
        Alliance alliance = DriverStation.getAlliance().orElse(null);

        if(alliance != null)
        {
            if(alliance.equals(DriverStation.Alliance.Red))
            {
                for(int i = 0; i < VisionConstants.redReefScoringPoses.length; i++)
                {
                    if(VisionConstants.redReefScoringPoses[i] == bestReefPose)
                    {
                        return i + 6;
                    }
                }
            }
            
            if(alliance.equals(DriverStation.Alliance.Blue))
            {
                for(int i = 0; i < VisionConstants.blueReefScoringPoses.length; i++)
                {
                    if(VisionConstants.blueReefScoringPoses[i] == bestReefPose)
                    {
                        return i + 17;
                    }
                }
            }
        }

        return 0;
    }

    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        if(Constants.photonVisonSimEnabled)
        {
            visionSim.update(Swerve.getInstance().getState().Pose);
            visionSim.getDebugField();
        }

        inputs.reefPose = getBestReefPos();
        inputs.reefTag = getReefTag();
    }
}