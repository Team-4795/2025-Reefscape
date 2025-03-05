package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private double intakeSpeed = 0.0;
    public boolean isStoring = false; 

    private static Intake instance;

    public static Intake getInstance(){
        return instance;
    }

    public static Intake initialize(IntakeIO io){
        if(instance == null){
            instance = new Intake(io);
        }
        return instance;
    }

    private Intake(IntakeIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    public double getPosition() {
        return inputs.angularPositionRot;
    }

    public void setIntakeSpeed(double speed){
        intakeSpeed = speed;
        io.setMotorSpeed(intakeSpeed);
    }

    /*
    public Command intakeS() {
        return startEnd(() -> setIntakeSpeed(IntakeConstants.intake), () -> setIntakeSpeed(0)).until(() -> GamePieceInitial()).andThen(() -> intakeSlow().until(() -> GamePeiceFinal()));
        return startEnd(() -> setIntakeSpeed(IntakeSetpoints.intake), () -> setIntakeSpeed(0)).until(() -> GamePieceInitial());
    }
    */

    public Command intake() {
        return startEnd(() -> setIntakeSpeed(IntakeConstants.intake), () -> setIntakeSpeed(0));
    }

    public Command intakeSlow() {
        return startEnd(() -> setIntakeSpeed(IntakeConstants.slow), () -> setIntakeSpeed(0));
    }

    public Command reverse() {
        return startEnd(() -> setIntakeSpeed(IntakeConstants.reverse), () -> setIntakeSpeed(0));
    }

    public Command intakeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> setIntakeSpeed(IntakeConstants.intake)), 
            Commands.waitSeconds(0.3),
            Commands.waitUntil(() -> GamePieceFinal()),
            reverse().withTimeout(0.12),
            Commands.runOnce(() -> isStoring()));
    }
    
    public boolean GamePieceInitial() {
        return IntakeConstants.initialThreshold <= inputs.currentAmps;
    }

    public boolean GamePieceFinal() {
        return IntakeConstants.currentThreshold <= inputs.currentAmps; 
    }

    public boolean hasGamepiece() {
        return io.hasGamepiece();
    }

    public void isStoring() {
        isStoring = true; 
    }

    public void outtake() {
        isStoring = false; 
    }
    
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Intake/Intake speed", intakeSpeed);
        Logger.recordOutput("Intake/Gamepiece detected", hasGamepiece());
        Logger.recordOutput("Intake/Curent Above", GamePieceFinal());
    }
}



