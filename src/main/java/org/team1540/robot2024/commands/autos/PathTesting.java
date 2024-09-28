package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class PathTesting extends AutoCommand {

    public PathTesting(Drivetrain drivetrain) {
        super("PathTesting");
        addPath(
                PathHelper.fromChoreoPath("TestPath.1"),
                PathHelper.fromChoreoPath("TestPath.3"),
                PathHelper.fromChoreoPath("TestPath.6")
        );
        addCommands(
            getPath(1).getCommand(drivetrain)
        );
    }
}
