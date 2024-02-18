package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class SourceLanePHGFSprint extends AutoCommand {
    public SourceLanePHGFSprint(Drivetrain drivetrain){
        super("SourceLanePHGFSprint");

        addPath(
                PathHelper.fromChoreoPath("SourceLanePHGFSprint.1", true, true),
                PathHelper.fromChoreoPath("SourceLanePHGFSprint.2"),
                PathHelper.fromChoreoPath("SourceLanePHGFSprint.3"),
                PathHelper.fromChoreoPath("SourceLanePHGFSprint.4")
        );

        addCommands(
                getPath(0).getCommand(drivetrain, true),
                getPath(1).getCommand(drivetrain),
                getPath(2).getCommand(drivetrain),
                getPath(3).getCommand(drivetrain)
        );
    }
}
