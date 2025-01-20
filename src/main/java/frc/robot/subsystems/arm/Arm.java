package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private boolean openLoop = true;
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

    public void setGoal(double angle) {
        openLoop = false;
        io.setGoal(angle);
    }

    public void setOpenLoop(boolean openLoop) {
        this.openLoop = openLoop;
    }

    @Override
    public void periodic() {
        this.inputs.openLoop = openLoop;
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }
}
