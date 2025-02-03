package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public final class ElevatorConstants{
    public static final double maxDistance = 0.80;
    // public static double constraints = 0.2; //figure this out later
    public static final Constraints constraints = new Constraints(4, 5); //change later

    public static double kD = 0; //figure this out later
    public static double kI = 0; //figure this out later
    public static double kP = 1; //figure this out later
    public static int rightDeviceID = 12;
    public static int leftDeviceID = 13;
    public static double ks = 0;
    public static double kg = 0.16;
    public static double kv = 15.96; //IS THIS RIGHT?!?!?
    
    //change this later
    public static double conversionFactor = 0.1; 


    public static final int elevatorCurrentLimits = 45; //might need to adjust later



    

}
