package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePCBAFE extends AutoCommand {

    public CenterLanePCBAFE(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!CenterLanePCBAFE");
        addPath(
                PathHelper.fromChoreoPath("CenterLanePCBAFE.1", false, true),
                PathHelper.fromChoreoPath("CenterLanePCBAFE.2"),
                PathHelper.fromChoreoPath("CenterLanePCBAFE.3"),
                PathHelper.fromChoreoPath("CenterLanePCBAFE.4"),
                PathHelper.fromChoreoPath("CenterLanePCBAFE.5")

        );

        addCommands(
                ShootSequence.forAutoSubwoofer(shooter, indexer),
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                drivetrain.commandCopyVisionPose(),
                                createSegmentSequence(drivetrain, shooter, indexer, 0, false, false, false),
                                createSegmentSequence(drivetrain, shooter, indexer, 1, false, false, true),
                                createSegmentSequence(drivetrain, shooter, indexer, 2, false, false, true),
                                createSegmentSequence(drivetrain, shooter, indexer, 3, false, false, true),
                                createSegmentSequence(drivetrain, shooter, indexer, 4, false, false, true)
                        )
                )
        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
