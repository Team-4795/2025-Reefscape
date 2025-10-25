package frc.robot.subsystems.intake;

public final class IntakeConstants {
    //PLACEHOLDERS
    public static final int canID = 18;

    public static final double intake = -0.7;
    public static final double slow = -0.3;
    public static final double reverse = 0.7;
    public static final double coralReverse = 0.3; 
    public static final int sensorChannel = 9;

    public static final double simGearRatio = 0;
    public static final double overrideSpeed = -0.0;

    public static final double intakeCurrent = 0;
    public static final int currentLimit = 70; 
    public static final double currentThreshold = 40; //change later based on akit numbers for gamepiece

    public static final double initialThreshold = 40;

    public static double velocityThreshold = 0.0; //change this to intake drop in speed
    public static double intakeVelocity = 0.0; //change this to speed we want on intake to actually be able to intake a coral
    public static double reverseThreshold = 0.0;  //when you reverse the coral 

}

