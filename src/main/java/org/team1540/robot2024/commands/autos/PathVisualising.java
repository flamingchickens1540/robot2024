package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class PathVisualising extends AutoCommand {

    public PathVisualising(Drivetrain drivetrain){
        super("PathVisualising");

        addPath(
                PathHelper.fromChoreoPath("AmpLanePADESprint", true, true)
        );
        addCommands(
                getPath(0).getCommand(drivetrain)
        );
    }
}
