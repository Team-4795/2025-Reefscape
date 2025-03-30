package frc.robot.util;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoCommands;

public class NamedCommandManager {
    public static void registerNamedCommands() {
        NamedCommands.registerCommand("Say Hello", Commands.print("Hello World!"));
        NamedCommands.registerCommand("Raise to L4", AutoCommands.raiseL4());
        NamedCommands.registerCommand("Raise to new L4", AutoCommands.raiseL4());
        NamedCommands.registerCommand("Stow", AutoCommands.autoStow());
        NamedCommands.registerCommand("Stow no intake", AutoCommands.stow());
        NamedCommands.registerCommand("Raise to L3", AutoCommands.noElevatorRaiseL3());
        NamedCommands.registerCommand("Raise to L2", AutoCommands.raiseL2());
        NamedCommands.registerCommand("Intake", AutoCommands.intake());
        NamedCommands.registerCommand("Align Reef", AutoCommands.alignReefUntil());
        NamedCommands.registerCommand("Auto Score", AutoCommands.autoScore());
        NamedCommands.registerCommand("Score", AutoCommands.score());
        NamedCommands.registerCommand("VStow", AutoCommands.vstow());
        NamedCommands.registerCommand("SetIntakeSpeed", AutoCommands.setIntakeSpeed());
        NamedCommands.registerCommand("Score left reef", AutoCommands.scoreLeftReef());
        NamedCommands.registerCommand("Score right reef", AutoCommands.scoreRightReef());
        NamedCommands.registerCommand("Zero Arm", AutoCommands.zeroArm());
        NamedCommands.registerCommand("Set Score L4", AutoCommands.setScoringState());
    }
}
