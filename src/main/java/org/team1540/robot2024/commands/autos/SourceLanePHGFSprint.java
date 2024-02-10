package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.PrintCommand;
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
                new PrintCommand("Past 1"),
                getPath(1).getCommand(),
                new PrintCommand("Past 2"),
                getPath(2).getCommand(),
                new PrintCommand("Past 3"),
                getPath(3).getCommand(),
                new PrintCommand("Past 4")
        );
    }
}
