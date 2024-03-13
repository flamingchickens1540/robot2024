package org.team1540.robot2024.commands.tramp;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class TrampShoot extends ParallelRaceGroup {
    public TrampShoot(Tramp tramp) {
        addCommands(
                Commands.sequence(
                        Commands.waitUntil(() -> !tramp.isNoteStaged()).withTimeout(10),
                        Commands.waitSeconds(0.5)
                ),
                tramp.commandRun(1)
        );
    }
}

