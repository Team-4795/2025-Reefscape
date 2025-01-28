package frc.robot.subsystems.arm;

import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private ArmIO io;
    private static Arm instance;

    private Arm(ArmIO io) {
        this.io = io;
    }

    public static void initialize(ArmIO io) {
        instance = new Arm(io);
    }

    public static Arm getInstance() {
        return instance;
    }

    public Command setGoal(double angle) {
        return Commands.runOnce(() -> setOpenLoop(false)).andThen(() -> io.setGoal(angle), this);
    }

    public void resetAbsoluteEncoder() {
        io.resetAbsoluteEncoder();
    }

    public Command manualVoltage(double voltage) {
        return  Commands.runOnce(() -> setOpenLoop(true)).andThen(Commands.startEnd(() -> io.setVoltage(voltage), () -> io.setVoltage(voltage), this));
    }

    public void setOpenLoop(boolean openLoop) {
        inputs.openLoop = openLoop;
    }

    public Pose3d getArmPose(){
        return new Pose3d(
            new Translation3d(10 + -(0.45) * Math.cos(inputs.angularPosition), 1, 1 + (0.45) * Math.sin(inputs.angularPosition)),
            new Rotation3d(0, inputs.angularPosition, 0)
        );
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/Pose", getArmPose());
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }
}
