package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePCBFSprint extends AutoCommand {
    public CenterLanePCBFSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("CenterLanePCBFSprint");

        addPath(
                PathHelper.fromChoreoPath("CenterLanePCBFSprint.1", true, true),
                PathHelper.fromChoreoPath("CenterLanePCBFSprint.2"),
                PathHelper.fromChoreoPath("CenterLanePCBFSprint.3"),
                PathHelper.fromChoreoPath("CenterLanePCBFSprint.4")
        );

        addCommands(
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(3),
                        getPath(0).getCommand(drivetrain)
                ),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(3),
                        getPath(1).getCommand(drivetrain)
                ), //TODO Could be made into an intake and shoot
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
