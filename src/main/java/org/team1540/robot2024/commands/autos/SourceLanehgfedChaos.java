package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.shooter.PrepareShooterCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;

public class SourceLanehgfedChaos extends AutoCommand {
    public  SourceLanehgfedChaos (Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("SourceLanephgfedChaos");
        addPath(
                PathHelper.fromChoreoPath("SourceLanehgfedChaos.1", true, true)
        );
        addCommands(
                    Commands.parallel(
                            getPath(0).getCommand(drivetrain),
                            IntakeAndFeed.withDefaults(indexer),
                            new PrepareShooterCommand(shooter, ()->new ShooterSetpoint(Constants.Shooter.Pivot.HUB_SHOOT.pivot, 1000, 1000))
                    )
        );
    }
}
