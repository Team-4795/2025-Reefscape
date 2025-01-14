package frc.robot.subsystems.vision.AprilTag;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        Pose3d[] pose = new Pose3d[] {};
        double[] timestamp = new double[] {};
        int[] tags = new int[] {};

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        double area = 0.0;

        int pipelineIndex = 10;
        String sortMode = "";
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void setReferencePose(Pose2d reference) {}

    public default void switchPipeline() {}

    public default void targetLeftReef() {}

    public default void targetRightReef() {}
}
