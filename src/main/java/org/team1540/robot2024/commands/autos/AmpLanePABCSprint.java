package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.shooter.AutoShooterPrepare;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class AmpLanePABCSprint extends AutoCommand {
    public AmpLanePABCSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("!AmpLanePABCSprint");

        addPath(
                PathHelper.fromChoreoPath("AmpLanePABCSprint.1", true, true),
                PathHelper.fromChoreoPath("AmpLanePABCSprint.2"),
                PathHelper.fromChoreoPath("AmpLanePABCSprint.3"),
                PathHelper.fromChoreoPath("AmpLanePABCSprint.4")
//                PathHelper.fromChoreoPath("AmpLanePABCSprint.4")
        );

        addCommands(
                Commands.parallel(
                        new AutoShooterPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createSegmentSequence(drivetrain, indexer, 0),
                                IntakeAndFeed.withDefaults(indexer).withTimeout(2),
                                createSegmentSequence(drivetrain, indexer, 1),
                                createSegmentSequence(drivetrain, indexer, 2)
                        )
                ),
                getPath(3).getCommand(drivetrain)
        );
    }
}
