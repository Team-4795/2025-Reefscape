package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs{
        public double current = 0.0;
        public double voltage = 0.0;
        public double position = 0.0;
        public double goalPosition = 0.0;
        public double velocity = 0.0;
    }
public default void updateInputs(ClimberIOInputs inputs) {

}
public default void setClimberVoltage(double volts){

}
public default void resetEncoder(){

}

public default double getGoal(){
    return 0.0;
}

public default double getPosition(){
    return 0.0;
}
public default void setGoal(double angle){

}
public default void hold(){
    
}

public default void deploy(double angle){

}

public default void climb(double angle){

}

}
