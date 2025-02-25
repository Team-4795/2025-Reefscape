package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public class NamedCommandManager {
    public static void registerNamedCommands() {
        NamedCommands.registerCommand("Say Hello", Commands.print("Hello World!"));
        NamedCommands.registerCommand("Raise to L4", AutoCommands.raiseL4());
        NamedCommands.registerCommand("Stow", AutoCommands.stow());
        NamedCommands.registerCommand("Raise to L3", AutoCommands.raiseL3());
        NamedCommands.registerCommand("Raise to L2", AutoCommands.raiseL2());
        NamedCommands.registerCommand("Score", AutoCommands.score());

    }
}
