package org.team1540.robot2024.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CommandUtils {
    public static Command startStopTimed(Runnable start, Runnable end, double duration, Subsystem... requirements) {
        return Commands.race(
            Commands.waitSeconds(duration),
            Commands.startEnd(start, end, requirements)
        );
    }

    public static Command rumbleCommandTimed(XboxController controller, double amount, double duration) {
        return startStopTimed(
                () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, amount),
                () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0),
                duration
        );
    }
    public static Command rumbleCommand(XboxController controller, double amount) {
        return Commands.startEnd(
                () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, amount),
                () -> controller.setRumble(GenericHID.RumbleType.kBothRumble, 0)
        );
    }
    public static Command rumbleCommand(CommandXboxController controller, double amount) {
        return rumbleCommand(controller.getHID(), amount);
    }
}
