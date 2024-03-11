package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePBDA extends AutoCommand {

    public CenterLanePBDA(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!CenterLanePBDA");
        addPath(
                PathHelper.fromChoreoPath("CenterLanePBDA.1", true, true),
                PathHelper.fromChoreoPath("CenterLanePBDA.2"),
                PathHelper.fromChoreoPath("CenterLanePBDA.3")
        );

        addCommands(
                ShootSequence.forAutoSubwoofer(shooter, indexer),
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createSegmentSequence(drivetrain, indexer, 0),
                                createSegmentSequence(drivetrain, indexer, 1),
                                createSegmentSequence(drivetrain, indexer, 2)
                        )
                )
        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
