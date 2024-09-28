package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class PathTesting extends AutoCommand {

    public PathTesting(Drivetrain drivetrain) {
        super("PathTesting");
        addPath(
                PathHelper.fromChoreoPath("TestPath.1", true, true),
                PathHelper.fromChoreoPath("TestPath.3", true, true),
                PathHelper.fromChoreoPath("TestPath.6", true, true)
        );
        addCommands(
            getPath(2).getCommand(drivetrain)
        );
    }
}
