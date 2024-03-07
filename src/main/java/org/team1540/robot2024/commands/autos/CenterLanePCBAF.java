package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePCBAF extends AutoCommand {

    public CenterLanePCBAF(Drivetrain drivetrain, Shooter shooter, Indexer indexer ) {
        super("CenterLanePCBAF");
        addPath(
                PathHelper.fromChoreoPath("CenterLanePCBAF.1", true, true),
                PathHelper.fromChoreoPath("CenterLanePCBAF.2"),
                PathHelper.fromChoreoPath("CenterLanePCBAF.3"),
                PathHelper.fromChoreoPath("CenterLanePCBAF.4")
        );

        addCommands(
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(2),
                        getPath(0).getCommand(drivetrain)
//                        new PrepareShooterCommand(shooter, HUB_SHOOT)
                ),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(1.5),
                        getPath(1).getCommand(drivetrain)
//                        new PrepareShooterCommand(shooter, HUB_SHOOT)
                ),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(2),
                        getPath(2).getCommand(drivetrain)
//                        new PrepareShooterCommand(shooter, HUB_SHOOT)
                ),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(5),
                        getPath(3).getCommand(drivetrain)
//                        new PrepareShooterCommand(shooter, HUB_SHOOT)
                ),
                new ShootSequence(shooter, indexer)
        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
