package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

import java.util.function.BooleanSupplier;

public class AmpLanePA extends AutoCommand {
    public AmpLanePA (Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("AmpLanePA");
        addPath(
                PathHelper.fromChoreoPath("AmpLanePA.1")
        );
        addCommands(
                getPath(0).getCommand(drivetrain),
                new ShootSequence(shooter, indexer),
                new IntakeCommand(indexer, () -> false, 100), //TODO: Tune this
                new ShootSequence(shooter, indexer)
        );
    }
}
