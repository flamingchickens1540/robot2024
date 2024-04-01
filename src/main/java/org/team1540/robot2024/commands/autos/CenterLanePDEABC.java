package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.commands.drivetrain.DriveWithTargetingCommand;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.commands.shooter.AutoShootPrepareWhileMoving;
import org.team1540.robot2024.commands.shooter.LeadingShootPrepare;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePDEABC extends AutoCommand {

    public CenterLanePDEABC(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!CenterLanePDEABC");

        addPath(
                PathHelper.fromChoreoPath("CenterLanePDEABC.1", true, true),
                PathHelper.fromChoreoPath("CenterLanePDEABC.2"),
                PathHelper.fromChoreoPath("CenterLanePDEABC.3"),
                PathHelper.fromChoreoPath("CenterLanePDEABC.4"),
                PathHelper.fromChoreoPath("CenterLanePDEABC.5")
        );

        addCommands(
                Commands.parallel(
                        new LeadingShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                Commands.runOnce(drivetrain::unblockTags),
//                                drivetrain.commandCopyVisionPose(),
//                                Commands.deadline(
//                                        Commands.waitSeconds(0.2).andThen(getPath(0).getCommand(drivetrain)),
                                        Commands.sequence(
//                                                Commands.waitSeconds(0.25),
//                                                Commands.waitSeconds(0.05),
                                                IntakeAndFeed.withDefaults(indexer).withTimeout(0.2)
//                                                new IntakeCommand(indexer, () -> false, 1)
                                        ),
//                                )
//                                ,
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
                                createCancoderSegmentSequence(drivetrain, shooter, indexer, 0),
                                createSegmentSequence(drivetrain, shooter, indexer, 1),
                                Commands.runOnce(drivetrain::blockTags),
                                createSegmentSequence(drivetrain, shooter, indexer, 2),
                                createSegmentSequence(drivetrain, shooter, indexer, 3),
                                createSegmentSequence(drivetrain, shooter, indexer, 4)
                        )
                )
        );
    }
}
