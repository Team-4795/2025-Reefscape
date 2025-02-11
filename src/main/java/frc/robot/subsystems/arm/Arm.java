package frc.robot.subsystems.arm;

import java.util.regex.MatchResult;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Gamepiece;
import frc.robot.Constants.Mode;
import frc.robot.Constants.OIConstants;

public class Arm extends SubsystemBase {
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private ArmIO io;
    private static Arm instance;

    private Arm(ArmIO io) {
        this.io = io;
        setDefaultCommand(
            Commands.run(() -> {
                double up = MathUtil.applyDeadband(OIConstants.driverController.getLeftTriggerAxis(), 0.1);
                double down = MathUtil.applyDeadband(OIConstants.driverController.getRightTriggerAxis(), 0.1);
                double change = (up - down) * 0.1;

                // io.setGoal(inputs.goalAngle + change);
            }, this)
        );
        setFFMode(Constants.currentMode == Mode.SIM ? Gamepiece.SIM: Gamepiece.NONE);
    }

    public double getAngle() {
        return inputs.angularPosition;
    }

    public double getGoalAngle() {
        return inputs.goalAngle;
    }

    public static void initialize(ArmIO io) {
        instance = new Arm(io);
    }

    public static Arm getInstance() {
        return instance;
    }

    public void setGoal(double angle) {
        io.setGoal(angle);
    }

    public void resetAbsoluteEncoder() {
        io.resetEncoder();
    }

    public void manualVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public Pose3d getArmPose(){
        return new Pose3d(
            new Translation3d(10 + -(0.45) * Math.cos(inputs.angularPosition), 1, 1 + (0.45) * Math.sin(inputs.angularPosition)),
            new Rotation3d(0, inputs.angularPosition, 0)
        );
    }

    public void setFFMode(Gamepiece gamepieceType) {
        Logger.recordOutput(getName() + "/FFMode", gamepieceType);
        switch (gamepieceType) {
            case CORAL:
                io.setFFValues(ArmConstants.CORALkS, ArmConstants.CORALkG, ArmConstants.CORALkV, ArmConstants.CORALkA);
                break;
            case ALGAE:
                io.setFFValues(ArmConstants.ALGAEkS, ArmConstants.ALGAEkG, ArmConstants.ALGAEkV, ArmConstants.ALGAEkA);
                break;
            case SIM:
                io.setFFValues(ArmConstants.SIMkS, ArmConstants.SIMkG, ArmConstants.SIMkV, ArmConstants.SIMkA);
                break;
            case NONE:
                io.setFFValues(ArmConstants.DEFAULTkS, ArmConstants.DEFAULTkG, ArmConstants.DEFAULTkV, ArmConstants.DEFAULTkA);
                break;
        }
    }

    @Override
    public void periodic() {
        io.updateMotionProfile();
        Logger.recordOutput(getName() + "/Pose", getArmPose());
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }
}
