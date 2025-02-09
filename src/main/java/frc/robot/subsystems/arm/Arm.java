package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Gamepiece;

public class Arm extends SubsystemBase {
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private ArmIO io;
    private static Arm instance;

    private Arm(ArmIO io) {
        this.io = io;
        setDefaultCommand(Commands.run(() -> io.hold(), this));
    }

    public double getAngle() {
        return inputs.angularPosition;
    }

    public static void initialize(ArmIO io) {
        instance = new Arm(io);
    }

    public static Arm getInstance() {
        return instance;
    }

    public Command setGoal(double angle) {
        return Commands.startEnd(
            () -> {
                setOpenLoop(false);
                io.setGoal(angle);
            },
            () -> {
                setOpenLoop(true);
            },
            this
        ).until(() -> MathUtil.isNear(angle, inputs.angularPosition, 0.008));
    }

    public void resetAbsoluteEncoder() {
        io.resetAbsoluteEncoder();
    }

    public void manualVoltage(double voltage) {
        Logger.recordOutput("isHolding", false);
        setOpenLoop(true);
        io.setVoltage(voltage);
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

    public void setFFMode(Gamepiece gamepieceType) {
        switch (gamepieceType) {
            case CORAL:
                io.setFFValues(ArmConstants.CORALkS, ArmConstants.CORALkG, ArmConstants.CORALkV, ArmConstants.CORALkA);
                break;
            case ALGAE:
                io.setFFValues(ArmConstants.ALGAEkS, ArmConstants.ALGAEkG, ArmConstants.CORALkV, ArmConstants.CORALkA);
                break;
            case NONE:
                io.setFFValues(ArmConstants.DEFAULTkS, ArmConstants.DEFAULTkG, ArmConstants.DEFAULTkV, ArmConstants.DEFAULTkA);
                break;
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/Pose", getArmPose());
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }
}
