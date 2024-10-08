package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class AmpLanePAEDSprint extends AutoCommand {

    public AmpLanePAEDSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("AmpLanePAEDSprint");

        addPath(
                PathHelper.fromChoreoPath("AmpLanePAEDSprint.1", true, true),
                PathHelper.fromChoreoPath("AmpLanePAEDSprint.2"),
                PathHelper.fromChoreoPath("AmpLanePAEDSprint.3"),
                PathHelper.fromChoreoPath("AmpLanePAEDSprint.4"),
                PathHelper.fromChoreoPath("AmpLanePAEDSprint.5")
        );

        addCommands(
//                Commands.runOnce(drivetrain::unblockTags),
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
//                        Commands.sequence(
//                                Commands.deadline(
//                                        getPath(0).getCommand(drivetrain),
//                                        Commands.sequence(
//                                                Commands.waitSeconds(0.25),
//                                                new InstantCommand(shooter::zeroPivotToCancoder),
//                                                Commands.waitSeconds(1),
//                                                IntakeAndFeed.withDefaults(indexer).withTimeout(2),
//                                                new IntakeCommand(indexer, () -> false, 1)
//                                        )
//                                ),
//                                Commands.sequence(
//                                        Commands.waitSeconds(0.25),
//                                        new InstantCommand(shooter::zeroPivotToCancoder)
//                                ),
//                                drivetrain.commandCopyVisionPose(),
//                                Commands.parallel(
//                                        new DriveWithTargetingCommand(drivetrain, null).withTimeout(0.4),
//                                        Commands.sequence(
//                                                new ParallelDeadlineGroup(
//                                                        Commands.sequence(
//                                                                Commands.waitUntil(()->!indexer.isNoteStaged()),
//                                                                Commands.waitSeconds(0.2)
//                                                        ).withTimeout(1),
//                                                        IntakeAndFeed.withDefaults(indexer)
//                                                )
//                                        )
//                                ),
                        Commands.sequence(
                                createCancoderSegmentSequence(drivetrain, shooter, indexer, 0),
                                createSegmentSequence(drivetrain, shooter, indexer, 1),
                                createSegmentSequence(drivetrain, shooter, indexer, 2),
                                createSegmentSequence(drivetrain, shooter, indexer, 3),
                                createSegmentSequence(drivetrain, shooter, indexer, 4)
                        )
//                        )
                )
        );
    }
}
