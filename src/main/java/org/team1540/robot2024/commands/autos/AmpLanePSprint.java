package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.PathHelper;

public class AmpLanePSprint extends AutoCommand {
    public AmpLanePSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("AmpLanePSprint");

        addPath(
                PathHelper.fromChoreoPath("AmpLanePSprint.1", true, true),
                PathHelper.fromChoreoPath("AmpLanePSprint.2"));

        addCommands(
                getPath(0).getCommand(drivetrain),
                new ShootSequence(shooter, indexer),
                getPath(1).getCommand(drivetrain)
        );
    }
}
