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
    public CenterLanePCBFSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("!!CenterLanePCBFSprint");

        addPath(
                PathHelper.fromChoreoPath("CenterLanePCBFSprint.1", true, true),
                PathHelper.fromChoreoPath("CenterLanePCBFSprint.2"),
                PathHelper.fromChoreoPath("CenterLanePCBFSprint.3"),
                PathHelper.fromChoreoPath("CenterLanePCBFSprint.4"),
                PathHelper.fromChoreoPath("CenterLanePCBFSprint.5")
        );

        addCommands(
                ShootSequence.forAutoSubwoofer(shooter, indexer),
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createSegmentSequence(drivetrain, indexer, 0),
                                createSegmentSequence(drivetrain, indexer, 1),
                                getPath(2).getCommand(drivetrain),
                                Commands.runOnce(drivetrain::copyVisionPose),
                                createSegmentSequence(drivetrain, indexer, 3)
                        )
                ),
                getPath(4).getCommand(drivetrain)
        );
    }
}
