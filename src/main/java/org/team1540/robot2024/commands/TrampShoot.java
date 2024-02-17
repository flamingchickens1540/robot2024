package org.team1540.robot2024.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class TrampShoot extends ParallelDeadlineGroup {
    public TrampShoot(Tramp tramp) {
        super(
                Commands.sequence(
                        Commands.waitUntil(() -> !tramp.isNoteStaged()),
                        Commands.waitSeconds(2) //TODO: tune this
                ),
                Commands.startEnd(
                        () -> tramp.setPercent(0.5), //TODO: tune this
                        tramp::stop,
                        tramp
            )
        );
    }
}

