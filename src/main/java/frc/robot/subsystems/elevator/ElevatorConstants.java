package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public final class ElevatorConstants{
    public static final double maxDistance = 0.80;
    // public static final double constraints = 0.2; //figure this out later
    public static final Constraints constraints = new Constraints(4, 5); //change later

    public static final double kD = 0; //figure this out later
    public static final double kI = 0; //figure this out later
    public static final double kP = 1; //figure this out later
    public static final int rightDeviceID = 12;
    public static final int leftDeviceID = 13;
    public static final double ks = 0;
    public static final double kg = 0.3;
    public static final double kv = 15.96; //IS THIS RIGHT?!?!?
    public static final double MAX_ACCELERATION = 1;
    public static final double MAX_VELOCITY = 1; 
    
    //change this later
    public static final double conversionFactor = 0.1; 


    public static final int elevatorCurrentLimits = 70; //might need to adjust later



    

}
