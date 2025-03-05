package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private WristIO io;
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private static Wrist instance;


    public Wrist(WristIO type) {
        this.io = type;
        io.updateInputs(inputs);
    }

    public static Wrist initialize(WristIO init){
        if (instance == null) {
            instance = new Wrist(init);
        }
        return instance;
    }

    public static Wrist getInstance(){
        return instance;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
    }
}
