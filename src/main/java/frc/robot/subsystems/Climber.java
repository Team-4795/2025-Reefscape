package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private CimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private ClimberIO io;
    private static Climber instance;
    private Climber(ClimberIO io){
        this.io = io;
    }

    public double getAngle(){
        return inputs.relativeEncoderPosition;
    }
    public double getGoalAngle(){
        return inputs.goalAngle;
    }
    public static void initialize(ClimberIO io){
        instance = newClimber(io);
    }
    public static Climber getInstance(){
        return instance;
    }
    public void setGoal(double angle){
        io.setGoal(angle);
    }
}