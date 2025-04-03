package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class ElevatorConstants{
    public static final double maxDistance = 0.7112;
    public static final double minDistance = 0.0;
    // public static final double constraints = 0.2; //figure this out later
    public static final Constraints constraints = new Constraints(4, 5); //change later

    public static final double kD = .3; //figure this out later
    public static final double kI = 0; //figure this out later
    public static final double kP = 16; //figure this out later
    public static final int rightDeviceID = 12;
    public static final int leftDeviceID = 13;
    public static final double ks = 1.4;
    public static final double kg = 0.25;
    public static final double kv = 6.5;
    public static final double MAX_ACCELERATION = 7;
    public static final double MAX_VELOCITY = 1.75;
    public static final double CORAL_L4_SETPOINT = 0.7097808122634888 + Units.inchesToMeters(1); 
    public static final double CORAL_L3_SETPOINT = (0.098505221 + Units.inchesToMeters(2)) / 2.0;
    public static final double CORAL_L2_SETPOINT = 0.207 + Units.inchesToMeters(2);
    public static final double CORAL_L1_SETPOINT = 0;
    public static final double ALGEA_SETPOINT = 0.19420458376407623 - Units.inchesToMeters(1);
    public static final double HIGH_ALGAE_SETPOINT = 0.12916289269924164;
    public static final double PROCESSOR_SETPOINT = 0.19420458376407623 - Units.inchesToMeters(2);
    public static final double NET_SETPOINT = 0.60;
    public static final double GOAL_TOLERANCE = 0.02;
    public static final double STOW = 0.01;
    
    public static final double conversionFactor = Units.inchesToMeters(1.0/9.0 * Math.PI * 1.751); // gearing * pi * sprocket diameter


    public static final int elevatorCurrentLimits = 50; //might need to adjust later



    

}
