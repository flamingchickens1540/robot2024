package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePCBA extends AutoCommand {

    public CenterLanePCBA(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!CenterLanePCBA");
        addPath(
                PathHelper.fromChoreoPath("CenterLanePCBA.1", true, true),
                PathHelper.fromChoreoPath("CenterLanePCBA.2"),
                PathHelper.fromChoreoPath("CenterLanePCBA.3")
        );

        addCommands(
                ShootSequence.forAutoSubwoofer(shooter, indexer),
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createSegmentSequence(drivetrain, indexer, 0),
                                createSegmentSequence(drivetrain, indexer, 1),
                                createSegmentSequence(drivetrain, indexer, 1)
                        )
                )
        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
