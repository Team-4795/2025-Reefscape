package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public class NamedCommandManager {
    public static void registerNamedCommands() {
        NamedCommands.registerCommand("Say Hello", Commands.print("Hello World!"));
        NamedCommands.registerCommand("raiseL4", AutoCommands.raiseL4());
    }
}
