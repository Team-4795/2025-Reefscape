package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;

public class Wrist extends SubsystemBase {
    private WristIO io;
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private static Wrist instance;
    


    public Wrist(WristIO io) {
        this.io = io;
        setDefaultCommand(
          Commands.run(() -> {
               double change = MathUtil.applyDeadband(OIConstants.operatorController.getLeftY(), OIConstants.OperatorLAxisDeadband);
               change = .05 * Math.pow(change, 3);
               if(DriverStation.isTeleopEnabled() && change != 0) {
                   io.setGoal(inputs.goalPosition + change);
               }
               io.updateMotionProfile();
           }, this)
       );   
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
    
    public void setGoal(double angle){
        io.setGoal(angle);
    }

    public double getPosition() {
        return inputs.position;
    }
    
    public boolean atGoal(double goal) {
        return MathUtil.isNear(goal, getPosition(), WristConstants.GOAL_TOLERANCE);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
    } 

    public SparkAbsoluteEncoder getArmAbsoluteEncoder(){
       return io.getArmAbsoluteEncoder();
    }


}
