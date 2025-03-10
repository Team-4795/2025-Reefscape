package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GenericRequirement extends SubsystemBase {
    /// our bad guys
    private static GenericRequirement instance;
    public static void initialize() {
        instance = new GenericRequirement();
    }
    public static GenericRequirement getInstance() {
        return instance;
    }
}
