package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private ClimberIO io;
    private static Climber instance;
    private Climber(ClimberIO io){
        this.io = io;
    }

    public static Climber initialize(ClimberIO init){
        if (instance == null){
            instance = new Climber(init);
        }
        return instance;
    }
  
    public static Climber getInstance(){
        return instance;
    }
    public void setGoal(double angle){
        io.setGoal(angle);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }
}
