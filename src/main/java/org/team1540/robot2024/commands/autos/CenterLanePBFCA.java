package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.util.auto.PathHelper;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.LeadingShootPrepare;

public class CenterLanePBFCA extends AutoCommand {
    public CenterLanePBFCA(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("CenterLanePBFCA");

        addPath(
                PathHelper.fromChoreoPath("CenterLanePBFCA.1", true, true),
                PathHelper.fromChoreoPath("CenterLanePBFCA.2"),
                PathHelper.fromChoreoPath("CenterLanePBFCA.3"),
                PathHelper.fromChoreoPath("CenterLanePBFCA.4")
        );
        addCommands(
                ShootSequence.forAutoSubwoofer(shooter, indexer),
                Commands.parallel(
                        new LeadingShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createCancoderSegmentSequence(drivetrain, shooter, indexer, 0),
                                createSegmentSequence(drivetrain, shooter, indexer, 1),
                                createSegmentSequence(drivetrain, shooter, indexer, 2),
                                createSegmentSequence(drivetrain, shooter, indexer, 3)
                        )
                )
        );
    }
}