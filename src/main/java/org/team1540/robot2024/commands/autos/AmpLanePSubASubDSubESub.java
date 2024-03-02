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

import static org.team1540.robot2024.Constants.Shooter.Pivot.HUB_SHOOT;

public class AmpLanePSubASubDSubESub extends AutoCommand {

    public AmpLanePSubASubDSubESub(Drivetrain drivetrain, Shooter shooter, Indexer indexer ) {
        super("AmpLanePSubASubDSubESub");
        addPath(
                PathHelper.fromChoreoPath("AmpLanePSubASubDSubESub.1", true, true),
                PathHelper.fromChoreoPath("AmpLanePSubASubDSubESub.2"),
                PathHelper.fromChoreoPath("AmpLanePSubASubDSubESub.3")
        );

        addCommands(
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(2.5),
                        getPath(0).getCommand(drivetrain)
//                        new PrepareShooterCommand(shooter, HUB_SHOOT)
                ),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(4),
                        getPath(1).getCommand(drivetrain)
//                        new PrepareShooterCommand(shooter, HUB_SHOOT)
                ),
                new ShootSequence(shooter, indexer),
                new ParallelCommandGroup(
                        new IntakeCommand(indexer, ()->false, 1).withTimeout(5),
                        getPath(2).getCommand(drivetrain)
//                        new PrepareShooterCommand(shooter, HUB_SHOOT)
                ),
                new ShootSequence(shooter, indexer)
        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
