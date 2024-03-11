package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePCBAFSprint extends AutoCommand {

    public CenterLanePCBAFSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!CenterLanePCBAFSprint");
        addPath(
                PathHelper.fromChoreoPath("CenterLanePCBAFSprint.1", true, true),
                PathHelper.fromChoreoPath("CenterLanePCBAFSprint.2"),
                PathHelper.fromChoreoPath("CenterLanePCBAFSprint.3"),
                PathHelper.fromChoreoPath("CenterLanePCBAFSprint.4"),
                PathHelper.fromChoreoPath("CenterLanePCBAFSprint.5")

        );

        addCommands(
                ShootSequence.forAutoSubwoofer(shooter, indexer),
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createSegmentSequence(drivetrain, indexer, 0),
                                drivetrain.commandCopyVisionPose(),
                                createSegmentSequence(drivetrain, indexer, 1),
                                drivetrain.commandCopyVisionPose(),
                                createSegmentSequence(drivetrain, indexer, 2),
                                drivetrain.commandCopyVisionPose(),
                                createSegmentSequence(drivetrain, indexer, 3),
                                drivetrain.commandCopyVisionPose()
                        )
                ),
                getPath(4).getCommand(drivetrain)
        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
