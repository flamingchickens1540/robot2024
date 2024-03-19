package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePCBADSprint extends AutoCommand {

    public CenterLanePCBADSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!CenterLanePCBADSprint");
        addPath(
                PathHelper.fromChoreoPath("CenterLanePCBADSprint.1", true, true),
                PathHelper.fromChoreoPath("CenterLanePCBADSprint.2"),
                PathHelper.fromChoreoPath("CenterLanePCBADSprint.3"),
                PathHelper.fromChoreoPath("CenterLanePCBADSprint.4"),
                PathHelper.fromChoreoPath("CenterLanePCBADSprint.5")
        );

        addCommands(
                ShootSequence.forAutoSubwoofer(shooter, indexer),
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createSegmentSequence(drivetrain, shooter, indexer, 0),
                                createSegmentSequence(drivetrain, shooter, indexer, 1),
                                createSegmentSequence(drivetrain, shooter, indexer, 2),
                                createSegmentSequence(drivetrain, shooter, indexer, 3)
                        )
                ),
                getPath(4).getCommand(drivetrain)

        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
