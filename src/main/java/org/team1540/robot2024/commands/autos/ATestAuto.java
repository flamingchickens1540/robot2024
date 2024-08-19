package org.team1540.robot2024.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2024.commands.shooter.AutoShootPrepare;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.PathHelper;
import org.team1540.robot2024.util.math.Triplet;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ATestAuto extends AutoCommand {

    public ATestAuto(Drivetrain drivetrain, Shooter shooter, Indexer indexer) {
        super("!ATestAuto");
        addPath(
                PathHelper.fromChoreoPath("TestPath.1", true, true),
                PathHelper.fromChoreoPath("TestPath.2", false, true),
                PathHelper.fromChoreoPath("TestPath.3", false, true)
        );
        addCommands(
                getPath(0).withInterrupt(drivetrain, false,
                        new Triplet<>(0, ()->true, getPath(1).getCommand(drivetrain, false).andThen(Commands.print(("message 1")))),
                        new Triplet<>(1, ()->true, getPath(2).getCommand(drivetrain,false).andThen(Commands.print("message 2")))
                ),

                Commands.print("HALLELUJAH")
        );
        addRequirements(drivetrain, shooter, indexer);
    }
}
