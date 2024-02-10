package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.AutoCommand;
import org.team1540.robot2024.util.PathPlannerHelper;

public class AmpLanePABCSprint extends AutoCommand {

    public AmpLanePABCSprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer){
        super("AmpLanePABCSprint");

        addPath(
                new PathPlannerHelper(drivetrain, "AmpLanePABCSprint.1", true, true, true),
                new PathPlannerHelper(drivetrain, "AmpLanePABCSprint.2", true),
                new PathPlannerHelper(drivetrain, "AmpLanePABCSprint.3", true),
                new PathPlannerHelper(drivetrain, "AmpLanePABCSprint.4", true)
                );
        addCommands(
//                new ShootSequence(shooter, indexer),
                getPath(0).getCommand(),
//                new ShootSequence(shooter, indexer),
                getPath(1).getCommand(),
//                new ShootSequence(shooter, indexer),
                getPath(2).getCommand(),
//                new ShootSequence(shooter, indexer)
                getPath(3).getCommand()
        );
    }
}
