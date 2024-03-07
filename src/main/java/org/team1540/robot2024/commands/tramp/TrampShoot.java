package org.team1540.robot2024.commands.tramp;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class TrampShoot extends SequentialCommandGroup {
    public TrampShoot(Tramp tramp) {
        addCommands(
                tramp.commandRun(1).onlyWhile(tramp::isNoteStaged),
                Commands.runOnce(()->tramp.setDistanceToGo(1))
        );
    }
}

