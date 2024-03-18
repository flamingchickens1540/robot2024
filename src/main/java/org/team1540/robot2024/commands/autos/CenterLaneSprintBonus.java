package org.team1540.robot2024.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

public class CenterLaneSprintBonus extends AutoCommand {
    public CenterLaneSprintBonus(Drivetrain drivetrain){
        super("CenterLaneSprintBonus");

        addPath(
                PathHelper.fromChoreoPath("CenterLaneSprintBonus.1", true, true),
                PathHelper.fromChoreoPath("CenterLaneSprintBonus.2")
        );

        addCommands(
                getPath(0).getCommand(drivetrain),
                Commands.waitSeconds(2),
                drivetrain.commandCopyVisionPose(),
                Commands.waitSeconds(2),
                getPath(1).getCommand(drivetrain),
                Commands.runOnce(drivetrain::stop)
        );
    }
}
