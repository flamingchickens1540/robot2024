package org.team1540.robot2024.commands.tramp;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class TrampShoot extends SequentialCommandGroup {
    public TrampShoot(Tramp tramp) {
        addRequirements(tramp);
//        addCommands(
//                new InstantCommand(() -> tramp.setPercent(1))
//        );
        //TODO Make sure this works
        addCommands(
                new InstantCommand(() -> tramp.setPercent(1)),
                Commands.waitUntil(() -> !tramp.isNoteStaged()).withTimeout(10),
                Commands.waitSeconds(0.5),
                new InstantCommand(tramp::stop)
        );
    }
}

