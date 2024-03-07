package org.team1540.robot2024.commands.autos;

import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;

import java.nio.file.Path;

public class CenterLanePCBASprint extends AutoCommand {
    public CenterLanePCBASprint(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("CenterLanePCBASprint");

        addPath(
                PathHelper.fromChoreoPath("CenterLanePCBASprint.1"),
                PathHelper.fromChoreoPath("CenterLanePCBASprint.2"),
                PathHelper.fromChoreoPath("CenterLanePCBASprint.3"),
                PathHelper.fromChoreoPath("CenterLanePCBASprint.4")
        );



    }
}
