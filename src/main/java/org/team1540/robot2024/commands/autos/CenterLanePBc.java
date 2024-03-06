package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePBc extends AutoCommand {
    public CenterLanePBc(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("CenterLanePBc");

        addPath(
                PathHelper.fromChoreoPath("CenterLanePBc.1.traj", true, true),
                PathHelper.fromChoreoPath("CenterLanePBc.2.traj"),
                PathHelper.fromChoreoPath("CenterLanePBc.3.traj"),
                PathHelper.fromChoreoPath("CenterLanePBc.4.traj")
        );

        addCommands(
                getPath(0).getCommand(drivetrain),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(3),
                        getPath(1).getCommand(drivetrain)
                ),
                getPath(2).getCommand(drivetrain),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(3),
                        getPath(3).getCommand(drivetrain)
                )
        );
    }
}
