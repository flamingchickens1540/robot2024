package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class AmpLanepdChaos extends AutoCommand {

    public AmpLanepdChaos(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!AmpLanepdChaos");

        addPath(
                PathHelper.fromChoreoPath("AmpLanepdChaos.1", true, true)
        );

        addCommands(
                Commands.parallel(
                        new AutoShootPrepare(drivetrain, shooter),
                        getPath(0).getCommand(drivetrain),
                        Commands.sequence(
                                Commands.waitSeconds(0.25),
                                Commands.runOnce(shooter::zeroPivotToCancoder),
                                IntakeAndFeed.withDefaults(indexer).withTimeout(0.5),
                                new IntakeCommand(indexer, ()->false, 1)
                        )
                )
        );
    }
}
