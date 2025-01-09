package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        Pose3d[] pose = new Pose3d[] {};
        double[] timestamp = new double[] {};
        int[] tags = new int[] {};
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void setReferencePose(Pose2d reference) {}
}
