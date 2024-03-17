package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class SourceLanePGHSprint extends AutoCommand {
    public SourceLanePGHSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("!SourceLanePGHSprint");
        addPath(
                PathHelper.fromChoreoPath("SourceLanePGHSprint.1", true, true),
                PathHelper.fromChoreoPath("SourceLanePGHSprint.2"),
                PathHelper.fromChoreoPath("SourceLanePGHSprint.3"),
                PathHelper.fromChoreoPath("SourceLanePGHSprint.4")
        );

        addCommands(
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createCancoderSegmentSequence(drivetrain, shooter, indexer, 0),
                                drivetrain.commandCopyVisionPose(),
                                createSegmentSequence(drivetrain, indexer, 1),
                                drivetrain.commandCopyVisionPose(),
                                createSegmentSequence(drivetrain, indexer, 2),
                                drivetrain.commandCopyVisionPose(),
                                getPath(3).getCommand(drivetrain)
                        )
                ),
                getPath(3).getCommand(drivetrain)
        );
    }

}
