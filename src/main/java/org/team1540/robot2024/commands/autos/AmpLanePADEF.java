package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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

public class AmpLanePADEF extends AutoCommand {

    public AmpLanePADEF(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!AmpLanePADEF");

        addPath(
                PathHelper.fromChoreoPath("AmpLanePADEF.1", true, true),
                PathHelper.fromChoreoPath("AmpLanePADEF.2"),
                PathHelper.fromChoreoPath("AmpLanePADEF.3"),
                PathHelper.fromChoreoPath("AmpLanePADEF.4"),
                PathHelper.fromChoreoPath("AmpLanePADEF.5")
        );

        addCommands(
//                Commands.runOnce(drivetrain::unblockTags),
                Commands.parallel(
                        new LeadingShootPrepare(drivetrain, shooter),
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
