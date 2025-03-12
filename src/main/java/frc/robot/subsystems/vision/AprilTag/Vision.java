package frc.robot.subsystems.vision.AprilTag;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.exc.ValueInstantiationException;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.Constants;

import static frc.robot.subsystems.vision.AprilTag.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase{
    private VisionIO io[];
    private VisionIOInputsAutoLogged inputs[];
    private boolean[] shouldUpdate = new boolean[] {true, true};

    public static Vision instance;

    public static Vision getInstance() {
        return instance;
    }

    public static Vision initialize(VisionIO... io) {
        if (instance == null) {
            instance = new Vision(io);
        }
        return instance;
    }

    public Vision(VisionIO visionIO[]) {
        io = visionIO;
        inputs = new VisionIOInputsAutoLogged[io.length];

        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }
    }

    public void setReferencePose(Pose2d reference) {
        for(int i = 0; i < io.length; i++) {
            io[i].setReferencePose(reference);
        }
    }

    public void toggleShouldUpdate() {
        for(int i = 0; i < io.length; i++) {
            shouldUpdate[i] = !shouldUpdate[i];
        }
    }

    public boolean isVisionUpdating() {
        return shouldUpdate[0];
    }

    public void toggleReefTag() {
        OIConstants.isReefTagOnly = !OIConstants.isReefTagOnly;
    }


    public Pose2d getBestReefPose() {
        if(Constants.currentMode == Constants.Mode.SIM)
        {
            return VisionConstants.redReefScoringPoses[0];
        }
        
        for(int i = 0; i < io.length; i++) {
            if(inputs[i].reefPose != new Pose2d())
            {
                return inputs[i].reefPose;
            }
        }
        return new Pose2d();
    }

    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + VisionConstants.cameraIds[i], inputs[i]);
            Logger.recordOutput("Vision/" + VisionConstants.cameraIds[i] + "/is updating", isVisionUpdating());
        }

        for (int i = 0; i < io.length; i++) {
            for (int p = 0; p < inputs[i].pose.length; p++) {
                Pose3d robotPose = inputs[i].pose[p];

                if (robotPose.getX() < -fieldBorderMargin
                    || robotPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
                    || robotPose.getY() < -fieldBorderMargin
                    || robotPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin
                    || robotPose.getZ() < -zMargin
                    || robotPose.getZ() > zMargin
                ) continue;

                List<Pose3d> tagPoses = new ArrayList<>();
                
                if(OIConstants.isReefTagOnly) {
                    for (int tag : inputs[i].tags) {
                        if(tag != 1 && tag != 2 && tag != 3 && tag != 4 && tag != 5 && tag != 12 && tag != 13 && tag != 14 && tag != 15 && tag != 16) {
                            VisionConstants.aprilTagFieldLayout.getTagPose(tag).ifPresent(tagPoses::add);
                        }
                        else {
                            continue;
                        }
                    }
                }

                if (tagPoses.isEmpty()) continue;

                double distance = 0.0;
                for (var tag : tagPoses) {
                    distance += tag.getTranslation().getDistance(robotPose.getTranslation());
                }

                distance /= tagPoses.size();
                double xyStdDev = (tagPoses.size() == 1 ? xyStdDevSingleTag : xyStdDevMultiTag) * Math.pow(distance, 2);

                // if(distance > Units.feetToMeters(6)) {
                //     xyStdDev *= 2;
                // }

                var stddevs = VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(100));

                Logger.recordOutput("Vision/" + VisionConstants.cameraIds[i] + "/Avg distance", distance);
                Logger.recordOutput("Vision/" + VisionConstants.cameraIds[i] + "/xy std dev", xyStdDev);
                Logger.recordOutput("Vision/" + VisionConstants.cameraIds[i] + "/Robot Pose", robotPose);
                
                if(inputs[i].poseAmbiguity < 0.1 && shouldUpdate[i]) {
                    Swerve.getInstance().addVisionMeasurement(robotPose.toPose2d(), inputs[i].timestamp[p], stddevs);
                }
            }
        }
    }
}
