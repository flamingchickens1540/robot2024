package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class AmpLanePADESprint extends AutoCommand {

    public AmpLanePADESprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!AmpLanePADESprint");

        addPath(
                PathHelper.fromChoreoPath("AmpLanePADESprint.1", true, true),
                PathHelper.fromChoreoPath("AmpLanePADESprint.2"),
                PathHelper.fromChoreoPath("AmpLanePADESprint.3"),
                PathHelper.fromChoreoPath("AmpLanePADESprint.4"),
                PathHelper.fromChoreoPath("AmpLanePADESprint.5")
        );

        addCommands(
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createCancoderSegmentSequence(drivetrain, shooter, indexer, 0),
                                createSegmentSequence(drivetrain, indexer, 1),
//                                new WaitCommand(5),
//                                drivetrain.commandCopyVisionPose(),
                                createSegmentSequence(drivetrain, indexer, 2),
//                                new WaitCommand(5),
//                                drivetrain.commandCopyVisionPose(),
                                createSegmentSequence(drivetrain, indexer, 3),
                                getPath(4).getCommand(drivetrain)
//                                drivetrain.commandCopyVisionPose()
                        )
                ),
                getPath(4).getCommand(drivetrain)
        );
    }
}
