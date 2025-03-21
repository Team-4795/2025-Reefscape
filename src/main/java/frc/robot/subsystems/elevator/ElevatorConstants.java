package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class ElevatorConstants{
    public static final double maxDistance = 0.7112;
    public static final double minDistance = 0.0;
    // public static final double constraints = 0.2; //figure this out later
    public static final Constraints constraints = new Constraints(4, 5); //change later

    public static final double kD = 0.05; //figure this out later
    public static final double kI = 0; //figure this out later
    public static final double kP = 28; //figure this out later
    public static final int rightDeviceID = 12;
    public static final int leftDeviceID = 13;
    public static final double ks = 0.1;
    public static final double kg = 0.25;
    // public static final double ka = 0.03;
    public static final double kv = 8.5;
    public static final double MAX_ACCELERATION = 10;
    public static final double MAX_VELOCITY = 1.75; 
    public static final double CORAL_L4_SETPOINT = 0.57 + Units.inchesToMeters(1); 
    public static final double ONE_CORAL_AWAY = 0.6761819124221802;
    public static final double CORAL_L2_SETPOINT = 0.39241382479667664;
    public static final double ALGEA_SETPOINT = 0.19420458376407623 - Units.inchesToMeters(1);
    public static final double HIGH_ALGAE_SETPOINT = 0.12916289269924164;
    public static final double PROCESSOR_SETPOINT = 0.19420458376407623 - Units.inchesToMeters(2);
    public static final double GOAL_TOLERANCE = 0.02;
    public static final double STOW = 0.01;
    
    public static final double conversionFactor = Units.inchesToMeters(1.0/9.0 * Math.PI * 1.751); // gearing * pi * sprocket diameter


    public static final int elevatorCurrentLimits = 60; //might need to adjust later



    

}
