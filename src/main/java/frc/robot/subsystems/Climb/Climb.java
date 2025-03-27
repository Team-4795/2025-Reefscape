package frc.robot.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase{
    private ClimbAutoLogged inputs = new ClimbAutologged();

    private ClimbIO io;
    private static Climb instance;

    public Climb(ClimbIO io){
        this.io = io;
        io.updateInputs();
    }

    
    public static Climb getInstance(){
        return instance;
    }

    public static Climb intialize(ClimbIO io){
        if (instance == null) {
            instance = new Climb(io);
        }
        return instance;
    }
}
