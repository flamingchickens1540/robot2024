package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.AutoCommand;
import org.team1540.robot2024.util.PathPlannerHelper;

public class SourceLanePHGFSprint extends AutoCommand {
    public SourceLanePHGFSprint(Drivetrain drivetrain){
        super("SourceLanePHGFSprint");
        addPath(
                new PathPlannerHelper(drivetrain, "SourceLanePHGFSprint.1", true, true, true),
                new PathPlannerHelper(drivetrain, "SourceLanePHGFSprint.2", true),
                new PathPlannerHelper(drivetrain, "SourceLanePHGFSprint.3", true),
                new PathPlannerHelper(drivetrain, "SourceLanePHGFSprint.4", true)
        );
        addCommands(
                getPath(0).getCommand(),
                getPath(1).getCommand(),
                getPath(2).getCommand(),
                getPath(3).getCommand()
        );
    }
}
