package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public class NamedCommandManager {
    public static void registerNamedCommands() {
        NamedCommands.registerCommand("Say Hello", Commands.print("Hello World!"));
        NamedCommands.registerCommand("Raise to L4", AutoCommands.raiseL4());
        NamedCommands.registerCommand("Stow", AutoCommands.stow());
        NamedCommands.registerCommand("Drive to Reef", AutoCommands.alignReef());
        NamedCommands.registerCommand("Drive to Feeder", AutoCommands.alignFeeder());
        NamedCommands.registerCommand("score", AutoCommands.score());
    }
}
