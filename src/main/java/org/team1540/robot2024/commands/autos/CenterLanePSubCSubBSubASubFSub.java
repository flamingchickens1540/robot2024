package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.PrepareShooterCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLanePSubCSubBSubASubFSub extends AutoCommand {

    public CenterLanePSubCSubBSubASubFSub(Drivetrain drivetrain, Shooter shooter, Indexer indexer ) {
        super("CenterLanePSubCSubBSubASubFSub");
        addPath(
                PathHelper.fromChoreoPath("CenterLanePSubCSubBSubASubFSub.1", true, true),
                PathHelper.fromChoreoPath("CenterLanePSubCSubBSubASubFSub.2"),
                PathHelper.fromChoreoPath("CenterLanePSubCSubBSubASubFSub.3"),
                PathHelper.fromChoreoPath("CenterLanePSubCSubBSubASubFSub.4")
        );

        addCommands(
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(2),
                        getPath(0).getCommand(drivetrain),
                        new PrepareShooterCommand(shooter)
                ),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(1.5),
                        getPath(1).getCommand(drivetrain),
                        new PrepareShooterCommand(shooter)
                ),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(2),
                        getPath(2).getCommand(drivetrain),
                        new PrepareShooterCommand(shooter)
                ),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(5),
                        getPath(3).getCommand(drivetrain),
                        new PrepareShooterCommand(shooter)
                ),
                new ShootSequence(shooter, indexer)
        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
