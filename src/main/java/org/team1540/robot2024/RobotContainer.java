package org.team1540.robot2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1540.robot2024.commands.FeedForwardCharacterization;
import org.team1540.robot2024.commands.autos.*;
import org.team1540.robot2024.commands.climb.ClimbAlignment;
import org.team1540.robot2024.commands.drivetrain.SwerveDriveCommand;
import org.team1540.robot2024.commands.drivetrain.WheelRadiusCharacterization;
import org.team1540.robot2024.commands.elevator.ElevatorManualCommand;
import org.team1540.robot2024.commands.indexer.ContinuousIntakeCommand;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.indexer.StageTrampCommand;
import org.team1540.robot2024.commands.shooter.*;
import org.team1540.robot2024.commands.tramp.AmpScoreSequence;
import org.team1540.robot2024.commands.tramp.AmpScoreStageSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.led.Leds;
import org.team1540.robot2024.subsystems.led.patterns.LedPatternRSLState;
import org.team1540.robot2024.subsystems.led.patterns.LedPatternWave;
import org.team1540.robot2024.subsystems.led.patterns.SimpleLedPattern;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.subsystems.tramp.Tramp;
import org.team1540.robot2024.subsystems.vision.AprilTagVision;
import org.team1540.robot2024.util.CommandUtils;
import org.team1540.robot2024.util.PhoenixTimeSyncSignalRefresher;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.AutoManager;

import static org.team1540.robot2024.Constants.SwerveConfig;
import static org.team1540.robot2024.Constants.isTuningMode;

public class RobotContainer {
    // Subsystems
    public final Drivetrain drivetrain;
    public final Tramp tramp;
    public final Shooter shooter;
    public final Elevator elevator;
    public final Indexer indexer;
    public final AprilTagVision aprilTagVision;
    public final Leds leds = new Leds();

    // Controller
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController copilot = new CommandXboxController(1);

    public final PhoenixTimeSyncSignalRefresher odometrySignalRefresher = new PhoenixTimeSyncSignalRefresher(SwerveConfig.CAN_BUS);

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL -> {
                // Real robot, instantiate hardware IO implementations
                if (Constants.IS_COMPETITION_ROBOT) {
                    elevator = Elevator.createReal();
                    drivetrain = Drivetrain.createReal(odometrySignalRefresher, elevator::getVelocity);
                    tramp = Tramp.createReal();
                    shooter = Shooter.createReal();
                    indexer = Indexer.createReal();
                    aprilTagVision = AprilTagVision.createReal(
                            drivetrain::addVisionMeasurement,
                            elevator::getPosition);
                } else {
                    elevator = Elevator.createDummy();
                    drivetrain = Drivetrain.createReal(odometrySignalRefresher, () -> 0.0);
                    tramp = Tramp.createDummy();
                    shooter = Shooter.createDummy();
                    indexer = Indexer.createDummy();
                    aprilTagVision = AprilTagVision.createDummy();
                }
            }
            case SIM -> {
                // Sim robot, instantiate physics sim IO implementations
                elevator = Elevator.createSim();
                drivetrain = Drivetrain.createSim(elevator::getVelocity);
                tramp = Tramp.createSim();
                shooter = Shooter.createSim();
                indexer = Indexer.createSim();
                aprilTagVision = AprilTagVision.createSim(
                        drivetrain::addVisionMeasurement,
                        drivetrain::getPose,
                        elevator::getPosition);
            }
            default -> {
                // Replayed robot, disable IO implementations
                elevator = Elevator.createDummy();
                drivetrain = Drivetrain.createDummy();
                tramp = Tramp.createDummy();
                shooter = Shooter.createDummy();
                indexer = Indexer.createDummy();
                aprilTagVision = AprilTagVision.createDummy();
            }
        }

        // Set up FF characterization routines
        AutoManager.getInstance().add(
                new AutoCommand(
                        "Drive FF Characterization",
                        new FeedForwardCharacterization(
                                drivetrain, drivetrain::runCharacterizationVolts, drivetrain::getCharacterizationVelocity
                        )
                )
        );

        AutoManager.getInstance().add(
                new AutoCommand(
                        "Flywheels FF Characterization",
                        new FeedForwardCharacterization(
                                shooter, volts -> shooter.setFlywheelVolts(volts, volts), () -> shooter.getLeftFlywheelSpeed() / 60
                        )
                )
        );

        AutoManager.getInstance().addDefault(new AmpLanePABCSprint(drivetrain, shooter, indexer));
        AutoManager.getInstance().add(new SourceLanePHGFSprint(drivetrain));

        configureAutoRoutines();
        // Configure the button bindings
        configureButtonBindings();
        configureLedBindings();
    }

    private void configureLedBindings() {
//        Runnable onDisconnect = () -> leds.setPatternAll(LedPatternFlame::new, Leds.PatternCriticality.HIGH);
//        onDisconnect.run();
//        new Trigger(DriverStation::isDSAttached)
//                .onTrue(Commands.runOnce(() -> leds.clearPatternAll(Leds.PatternCriticality.HIGH))
//                            .ignoringDisable(true))
//                .onFalse(Commands.runOnce(onDisconnect).ignoringDisable(true));
    }

    private void configureButtonBindings() {
        Command manualPivotCommand = new ManualPivotCommand(shooter, copilot);
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver));
//        drivetrain.setDefaultCommand(new OverStageShootPrepareWithTargeting(driver.getHID(), drivetrain, shooter));
//        drivetrain.setDefaultCommand(new AutoShootPrepareWhileMoving(driver.getHID(), drivetrain, shooter));
        elevator.setDefaultCommand(new ElevatorManualCommand(elevator, copilot));
        shooter.setDefaultCommand(manualPivotCommand);
        driver.b().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
//        driver.y().toggleOnTrue(new DriveWithSpeakerTargetingCommand(drivetrain, driver));
        driver.y().onTrue(Commands.runOnce(() -> {
            drivetrain.zeroFieldOrientationManual();
            drivetrain.setBrakeMode(true);
        }).ignoringDisable(true));


        Command targetDrive = new AutoShootPrepareWithTargeting(driver.getHID(), drivetrain, shooter)
                .alongWith(leds.commandShowPattern(new LedPatternWave("#00a9ff"), Leds.PatternLevel.DRIVER_LOCK));
        Command overstageTargetDrive = new OverStageShootPrepareWithTargeting(driver.getHID(), drivetrain, shooter)
                .alongWith(leds.commandShowPattern(new LedPatternWave("#f700ff"), Leds.PatternLevel.DRIVER_LOCK));
        Command autoShooterCommand = new AutoShootPrepare(drivetrain, shooter)
                .alongWith(leds.commandShowPattern(new LedPatternWave("#00ffbc"), Leds.PatternLevel.DRIVER_LOCK));
        Command cancelAlignment = Commands.runOnce(() -> {
            targetDrive.cancel();
            overstageTargetDrive.cancel();
        });
        driver.x()
                .whileTrue(IntakeAndFeed.withDefaults(indexer))
                .onFalse(cancelAlignment);

        driver.rightBumper().toggleOnTrue(targetDrive);
        driver.leftBumper().toggleOnTrue(overstageTargetDrive);

        // TODO remove this
        if (isTuningMode()) {
            driver.leftTrigger().whileTrue(new AutoShootPrepareWhileMoving(driver.getHID(), drivetrain, shooter).alongWith(leds.commandShowPattern(new LedPatternWave("#00ff00"), Leds.PatternLevel.DRIVER_LOCK)));
            driver.a().whileTrue(new TuneShooterCommand(shooter, indexer, drivetrain::getPose));

        }
        driver.povDown().and(() -> !DriverStation.isFMSAttached()).onTrue(Commands.runOnce(() -> drivetrain.setPose(new Pose2d(Units.inchesToMeters(260), Units.inchesToMeters(161.62), Rotation2d.fromRadians(0)))).ignoringDisable(true));

        driver.rightTrigger(0.95).toggleOnTrue(autoShooterCommand);

        driver.rightStick().onTrue(cancelAlignment);

        copilot.back().onTrue(Commands.runOnce(shooter::zeroPivotToCancoder).andThen(Commands.print("BACK IS PRESSED")));

        copilot.leftBumper().whileTrue(new AmpScoreSequence(tramp, indexer, elevator));
        Command intakeCommand = new ContinuousIntakeCommand(indexer, 1)
                .deadlineWith(CommandUtils.rumbleCommand(driver, 0.5), CommandUtils.rumbleCommand(copilot, 0.5));
        copilot.rightBumper().whileTrue(intakeCommand);

        copilot.povDown().whileTrue(indexer.commandRunIntake(-1));
        copilot.povUp().whileTrue(indexer.commandRunIntake(1));
        copilot.povRight().whileTrue(tramp.commandRun(1));


        copilot.rightTrigger(0.95).whileTrue(tramp.commandRun(1));
        copilot.leftTrigger(0.95).whileTrue(new ClimbAlignment(drivetrain, elevator, tramp, indexer, shooter));


        copilot.x().whileTrue(new ShootSequence(shooter, indexer));
        copilot.a().whileTrue(new AmpScoreStageSequence(indexer, tramp, elevator));
        copilot.b().whileTrue(IntakeAndFeed.withDefaults(indexer))
                    .onFalse(cancelAlignment);
        copilot.y().whileTrue(new StageTrampCommand(tramp, indexer));

//        copilot.leftTrigger(0.5).whileTrue(new ElevatorSetpointCommand(elevator, ElevatorState.CLIMB));


        new Trigger(() -> elevator.getPosition() > 0.1).debounce(0.1)
                .whileTrue(PrepareShooterCommand.lowerPivot(shooter));

        new Trigger(indexer::isNoteStaged).debounce(0.1)
                .onTrue(CommandUtils.rumbleCommandTimed(driver.getHID(), 1, 1))
                .whileTrue(leds.commandShowIntakePattern(SimpleLedPattern.solid("#ff0000")));

        new Trigger(tramp::isNoteStaged).debounce(0.1)
                .whileTrue(leds.commandShowTrampPattern(SimpleLedPattern.solid("#ff9900")));

        new Trigger(indexer::isNoteStaged).and(intakeCommand::isScheduled).onTrue(CommandUtils.rumbleCommandTimed(driver.getHID(), 0.8, 0.4));

        new Trigger(RobotController::getUserButton).toggleOnTrue(Commands.startEnd(
                () -> {
                    elevator.setBrakeMode(false);
                    leds.setPatternAll(() -> new LedPatternRSLState(Color.kMagenta), Leds.PatternLevel.ELEVATOR_STATE);
                },
                () -> {
                    elevator.setBrakeMode(true);
                    leds.clearPatternAll(Leds.PatternLevel.ELEVATOR_STATE);
                }
        ).ignoringDisable(true));


    }

    private void configureAutoRoutines() {
        AutoManager autos = AutoManager.getInstance();
        // Set up FF characterization routines
        if (isTuningMode()) {
            AutoManager.getInstance().add(
                    new AutoCommand(
                            "Drive FF Characterization",
                            new FeedForwardCharacterization(
                                    drivetrain, drivetrain::runCharacterizationVolts, drivetrain::getCharacterizationVelocity
                            )
                    )
            );

            AutoManager.getInstance().add(
                    new AutoCommand(
                            "Flywheels FF Characterization",
                            new FeedForwardCharacterization(
                                    shooter, volts -> shooter.setFlywheelVolts(volts, volts), () -> shooter.getLeftFlywheelSpeed() / 60
                            )
                    )
            );

            autos.add(new AutoCommand("WheelRadiusChar", new WheelRadiusCharacterization(drivetrain, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE)));
        }
        autos.addDefault(new AutoCommand("Dwayne :skull:"));
        autos.add(new AutoCommand("SubwooferShot", new ShootSequence(shooter, indexer)));
        autos.add(new DriveSinglePath("AmpLaneTaxi", drivetrain));
        autos.add(new DriveSinglePath("AmpLaneSprint", drivetrain));
        autos.add(new AmpLanePADESprint(drivetrain, shooter, indexer));
        autos.add(new DriveSinglePath("CenterLaneTaxi", drivetrain));
        autos.add(new DriveSinglePath("CenterLaneSprint", drivetrain, true, true));
        autos.add(new CenterLanePSubSprint(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBADSprint(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBAFSprint(drivetrain, shooter, indexer));
//        autos.add(new CenterLanePCBAFE(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBA(drivetrain, shooter, indexer));
        autos.add(new CenterLanePBDA(drivetrain, shooter, indexer));
        autos.add(new CenterLanePSubCSubBSubASubFSub(drivetrain, shooter, indexer));
//        autos.add(new CenterLanePSubCSubBSubFSub(drivetrain, shooter, indexer));
        autos.add(new CenterLanePSubCSubBSubASub(drivetrain, shooter, indexer));
        autos.add(new DriveSinglePath("SourceLaneTaxi", drivetrain));
        autos.add(new DriveSinglePath("SourceLaneSprint", drivetrain));
        autos.add(new SourceLanePGHSprint(drivetrain, shooter, indexer));
        autos.add(new SourceLanePHG(drivetrain, shooter, indexer));
    }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoManager.getInstance().getSelected();
    }
}
