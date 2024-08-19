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
        Command cmd = getPath(0).getCommand(drivetrain);
        addCommands(
//                new ProxyCommand(Commands.none()),
//                new ProxyCommand(Commands.waitSeconds(1).andThen(Commands.print("Waited"))),
//                getPath(0).getCommand(drivetrain),
//                new ProxyCommand(Commands.waitSeconds(2)),
//                new DeferredCommand((Supplier<Command>) () -> getPath(0).getCommand(drivetrain).andThen(Commands.print("THEWORD")),null)

//                Commands.race(
//                        getPath(0).getCommand(drivetrain).finallyDo(()->System.out.println("HI")),
//                        Commands.waitSeconds(1.5).andThen(Commands.print("TIMEOUT")),
//                        Commands.sequence(
//                                Commands.waitSeconds(1),
////                                getPath(1).getCommand(drivetrain).finallyDo(()->System.out.println("H2I")).asProxy().finallyDo(()->System.out.println("H#I"))
//                                Commands.runOnce(()->getPath(1).getCommand(drivetrain).schedule())
//                        )
//                ).andThen(Commands.print("TIMESUP")),

//                  Commands.runOnce(()->cmd.schedule()),
//                  Commands.waitUntil(()->!cmd.isScheduled()),
//                new ProxyCommand(cmd.alongWith(Commands.waitSeconds(1).andThen(new ProxyCommand(cmd)))),
//                  new ProxyCommand(cmd.andThen(Commands.runOnce(()->{
//                      CommandScheduler.getInstance().cancel(cmd);
//                  }).andThen(Commands.print("CANCELING"))
//                  )),
//                new ProxyCommand(
//                        Commands.deadline(
//                                Commands.waitSeconds(5).andThen(Commands.print("ENDOF5")),
//                                getPath(0).getCommand(drivetrain).andThen(Commands.print("THEWORD"))
//                        )
//                ),

                getPath(0).withInterrupt(drivetrain, false,
//                        new Triplet<>(1, ()->true,Commands.waitSeconds(1).andThen(Commands.print("TERMINATED 1")))
//                )
//                        ,
                        new Triplet<>(0, ()->true, getPath(1).getCommand(drivetrain, false).andThen(Commands.print(("message 1")))),
                        new Triplet<>(1, ()->true, getPath(2).getCommand(drivetrain,false).andThen(Commands.print("message 2")))
                ),


                Commands.print("HALLELUJAH")
//                getPath(1).withInterrupt(drivetrain, false, null)
//                new StartEndCommand()
//                new StartEndCommand()


        );

//        addCommands(
//                ShootSequence.forAutoSubwoofer(shooter, indexer),
//                Commands.parallel(
//                        new AutoShootPrepare(drivetrain, shooter),
//                        Commands.sequence(
//                                drivetrain.commandCopyVisionPose(),
////                                Commands.deadline(
//                                Commands.race(
//                                        getPath(0).getCommand(drivetrain),
//                                        Commands.sequence(
//                                                Commands.waitSeconds(1),
//                                                Commands.either(
////                                                        Commands.waitSeconds(0),
//                                                        Commands.runOnce(()->{}),
//                                                        Commands.waitSeconds(100),
//                                                        ()->false
//                                                )
//                                        )
//                                ),
////                                Commands.print("YO YO DIGGITY DAWG"),
////                                        Commands.runOnce(drivetrain::stop)
////                                        Commands.sequence(
////                                                Commands.waitSeconds(1)
////                                                Commands.runOnce(()->CommandScheduler.getInstance().cancelAll())
////                                        )
////                                ),
////                                Commands.runOnce(drivetrain::stop)
//                                drivetrain.commandStop(),
//                                getPath(1).getCommand(drivetrain,true)
//
//                        )
//                )
//        );

        addRequirements(drivetrain, shooter, indexer);
    }
}
