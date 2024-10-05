package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.Constants;
import org.team1540.robot2024.commands.shooter.DefaultShooterCommand;
import org.team1540.robot2024.commands.shooter.SpitShoot;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.util.auto.PathHelper;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.commands.shooter.LeadingShootPrepare;

public class WeirdSourceLanePHGF extends AutoCommand {
    public WeirdSourceLanePHGF(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("WeirdSourceLanePHGF");

        addPath(
                PathHelper.fromChoreoPath("WeirdSourceLanePHGF.1", true, true),
                PathHelper.fromChoreoPath("WeirdSourceLanePHGF.2"),
                PathHelper.fromChoreoPath("WeirdSourceLanePHGF.3")
        );
        addCommands(
                new SpitShoot(shooter, indexer, Constants.Shooter.Flywheels.LEFT_RPM, Constants.Shooter.Flywheels.RIGHT_RPM, 1, 1, 0.25),
                Commands.parallel(
                        new LeadingShootPrepare(drivetrain, shooter),
                        Commands.sequence(
                                createCancoderSegmentSequence(drivetrain, shooter, indexer, 0),
                                createSegmentSequence(drivetrain, shooter, indexer, 1),
                                createSegmentSequence(drivetrain, shooter, indexer, 2)
                        )
                )
        );
    }
}
