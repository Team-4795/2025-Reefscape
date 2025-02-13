package frc.robot.Util;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;

public class NamedCommandManager {
    public void registerNamedCommands() {
        NamedCommands.registerCommand("Say Hello", Commands.print("Hello World!"));
    }
}
