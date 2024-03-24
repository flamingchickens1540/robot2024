package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePCBFSprint extends AutoCommand {

    public CenterLanePCBFSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!CenterLanePCBFSprint");
        addPath(
                PathHelper.fromChoreoPath("CenterLanePCBAFSprint.1", false, true),
                PathHelper.fromChoreoPath("CenterLanePCBFSprint.2"),
                PathHelper.fromChoreoPath("CenterLanePCBFSprint.3"),
                PathHelper.fromChoreoPath("CenterLanePCBFSprint.4")

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
                                getPath(3).getCommand(drivetrain)
                        )
                )
        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
