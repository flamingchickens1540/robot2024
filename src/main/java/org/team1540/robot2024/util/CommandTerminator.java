package org.team1540.robot2024.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class CommandTerminator extends ParallelDeadlineGroup {
    public CommandTerminator(Command deadline, Runnable... onEnd) {
        super(
                deadline,
                Commands.startEnd(()->{}, () -> {
                    for (Runnable run : onEnd) {
                        run.run();
                    }
                })
        );
    }
}
