package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSetpoints;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private double intakeSpeed = 0.0;

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

    public void setIntakeSpeed(double speed){
        intakeSpeed = speed;
    }

    /*
    public Command intakeS() {
        return startEnd(() -> setIntakeSpeed(IntakeSetpoints.intake), () -> setIntakeSpeed(0)).until(() -> GamePieceInitial()).andThen(() -> intakeSlow().until(() -> GamePeiceFinal()));
        return startEnd(() -> setIntakeSpeed(IntakeSetpoints.intake), () -> setIntakeSpeed(0)).until(() -> GamePieceInitial());
    }
    */

    public Command intake() {
        return startEnd(() -> setIntakeSpeed(IntakeSetpoints.intake), () -> setIntakeSpeed(0));
    }

    public Command intakeSlow() {
        return startEnd(() -> setIntakeSpeed(IntakeSetpoints.slow), () -> setIntakeSpeed(0));
    }

    public Command reverse() {
        return startEnd(() -> setIntakeSpeed(IntakeSetpoints.reverse), () -> setIntakeSpeed(0));
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
    
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        io.setMotorSpeed(intakeSpeed);
        Logger.recordOutput("Intake/Intake speed", intakeSpeed);
        Logger.recordOutput("Intake/Gamepiece detected", hasGamepiece());
    }
}



