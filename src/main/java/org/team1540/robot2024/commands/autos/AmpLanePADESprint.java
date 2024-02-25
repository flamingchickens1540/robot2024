package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class AmpLanePADESprint extends AutoCommand {
    public AmpLanePADESprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("AmpLanePADESprint");

        addPath(
                PathHelper.fromChoreoPath("AmpLanePADESprint.1", true, true),
                PathHelper.fromChoreoPath("AmpLanePADESprint.2"),
                PathHelper.fromChoreoPath("AmpLanePADESprint.3"),
                PathHelper.fromChoreoPath("AmpLanePADESprint.4")
        );

        addCommands(
                getPath(0).getCommand(drivetrain),
                new ShootSequence(shooter, indexer),
                new IntakeCommand(indexer, ()->false, 1).withTimeout(1),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(3),
                        getPath(1).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(3),
                        getPath(2).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                getPath(3).getCommand(drivetrain)
        );
    }
}
