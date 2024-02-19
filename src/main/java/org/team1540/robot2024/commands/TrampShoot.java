package org.team1540.robot2024.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.subsystems.tramp.Tramp;

public class TrampShoot extends SequentialCommandGroup {
    public TrampShoot(Tramp tramp) {
        addCommands(
                new InstantCommand(() -> tramp.setPercent(1))
        );
    }
}

