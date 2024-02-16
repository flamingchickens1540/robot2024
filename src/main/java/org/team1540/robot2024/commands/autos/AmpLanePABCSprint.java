package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class AmpLanePABCSprint extends AutoCommand {
    public AmpLanePABCSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("AmpLanePABCSprint");

        addPath(
                PathHelper.fromChoreoPath("AmpLanePABCSprint.1", true, true),
                PathHelper.fromChoreoPath("AmpLanePABCSprint.2"),
                PathHelper.fromChoreoPath("AmpLanePABCSprint.3"),
                PathHelper.fromChoreoPath("AmpLanePABCSprint.4")
        );

        addCommands(
//                new ShootSequence(shooter, indexer),
                getPath(0).getCommand(drivetrain),
//                new ShootSequence(shooter, indexer),
                getPath(1).getCommand(drivetrain),
//                new ShootSequence(shooter, indexer),
                getPath(2).getCommand(drivetrain),
//                new ShootSequence(shooter, indexer)
                getPath(3).getCommand(drivetrain)
        );
    }
}
