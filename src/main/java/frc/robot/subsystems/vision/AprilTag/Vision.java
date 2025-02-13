package frc.robot.subsystems.vision.AprilTag;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.vision.AprilTag.VisionIOInputsAutoLogged;

import static frc.robot.subsystems.vision.AprilTag.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase{
    private VisionIO io[];
    private VisionIOInputsAutoLogged inputs[];
    private boolean[] shouldUpdate = new boolean[] {true, true, true, true};

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

    public void toggleShouldUpdate(int id) {
        shouldUpdate[id] = !shouldUpdate[id];
    }

    public Pose2d getBestReefPose() {
        for(int i = 0; i < io.length; i++) {
            if(inputs[i].reefPose != new Pose2d())
            {
                return inputs[i].reefPose;
            }
        }
        return new Pose2d();
    }

    // public void toggleReefMode() {
    //     io[0].switchPipeline();
    //     toggleShouldUpdate(0);
    // }

    // public void targetLeftReef() {
    //     io[0].targetLeftReef();
    // }

    // public void targetRightReef() {
    //     io[0].targetRightReef();
    // }

    public Command toggleReefMode() {
        return Commands.parallel(
            new RunCommand(() -> io[0].switchPipeline()), 
            new RunCommand(() -> toggleShouldUpdate(0))
        );
    }

    public Command targetLeftReef() { 
        return Commands.run(() -> io[0].targetLeftReef());
    }

    public Command targetRightReef() { 
        return Commands.run(() -> io[0].targetRightReef());
    }

    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + VisionConstants.cameraIds[i], inputs[i]);
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
                for (int tag : inputs[i].tags) {
                    VisionConstants.aprilTagFieldLayout.getTagPose(tag).ifPresent(tagPoses::add);
                }

                if (tagPoses.isEmpty()) continue;

                double distance = 0.0;
                for (var tag : tagPoses) {
                    distance += tag.getTranslation().getDistance(robotPose.getTranslation());
                }

                distance /= tagPoses.size();
                double xyStdDev = (tagPoses.size() == 1 ? xyStdDevSingleTag : xyStdDevMultiTag) * Math.pow(distance, 2);
                var stddevs = VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(40));

                Logger.recordOutput("Vision/" + VisionConstants.cameraIds[i] + "/Avg distance", distance);
                Logger.recordOutput("Vision/" + VisionConstants.cameraIds[i] + "/xy std dev", xyStdDev);
                
                if(shouldUpdate[i]) {
                    RobotContainer.drivetrain.addVisionMeasurement(robotPose.toPose2d(), inputs[i].timestamp[p], stddevs);
                    // Drive.getInstance().addVisionMeasurement(robotPose.toPose2d(), inputs[i].timestamp[p], stddevs);
                }
            }
        }
    }
}
