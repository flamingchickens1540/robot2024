package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.AutoShooterPrepare;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePCBAD extends AutoCommand {

    public CenterLanePCBAD(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!CenterLanePCBAD");
        addPath(
                PathHelper.fromChoreoPath("CenterLanePCBAD.1", true, true),
                PathHelper.fromChoreoPath("CenterLanePCBAD.2"),
                PathHelper.fromChoreoPath("CenterLanePCBAD.3"),
                PathHelper.fromChoreoPath("CenterLanePCBAD.4"),
                PathHelper.fromChoreoPath("CenterLanePCBAD.5")
        );

        addCommands(
                new ShootSequence(shooter, indexer),
                Commands.parallel(
                        new AutoShooterPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createSegmentSequence(drivetrain, indexer, 0),
                                createSegmentSequence(drivetrain, indexer, 1),
                                createSegmentSequence(drivetrain, indexer, 2),
                                createSegmentSequence(drivetrain, indexer, 3),
                                getPath(4).getCommand(drivetrain)
                        )
                )
        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
