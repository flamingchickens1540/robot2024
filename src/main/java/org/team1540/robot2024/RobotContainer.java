package org.team1540.robot2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.commands.FeedForwardCharacterization;
import org.team1540.robot2024.commands.autos.*;
import org.team1540.robot2024.commands.climb.ClimbAlignment;
import org.team1540.robot2024.commands.drivetrain.DriveWithAmpSideLock;
import org.team1540.robot2024.commands.drivetrain.SwerveDriveCommand;
import org.team1540.robot2024.commands.drivetrain.WheelRadiusCharacterization;
import org.team1540.robot2024.commands.elevator.ElevatorManualCommand;
import org.team1540.robot2024.commands.indexer.ContinuousIntakeCommand;
import org.team1540.robot2024.commands.indexer.IntakeAndFeed;
import org.team1540.robot2024.commands.indexer.StageTrampCommand;
import org.team1540.robot2024.commands.shooter.*;
import org.team1540.robot2024.commands.tramp.AmpScoreSequence;
import org.team1540.robot2024.commands.tramp.AmpScoreStageSequence;
import org.team1540.robot2024.subsystems.drive.Drivetrain;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.led.Leds;
import org.team1540.robot2024.subsystems.led.patterns.*;
import org.team1540.robot2024.subsystems.shooter.Shooter;
import org.team1540.robot2024.subsystems.tramp.Tramp;
import org.team1540.robot2024.subsystems.vision.AprilTagVision;
import org.team1540.robot2024.util.CommandUtils;
import org.team1540.robot2024.util.PhoenixTimeSyncSignalRefresher;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.AutoManager;

import java.util.function.BooleanSupplier;

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
    public final CommandXboxController kidPilot = new CommandXboxController(2);

    public final PhoenixTimeSyncSignalRefresher odometrySignalRefresher = new PhoenixTimeSyncSignalRefresher(SwerveConfig.CAN_BUS);



    public boolean isBrakeMode;
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
                            elevator::getPosition,
                            drivetrain::getRotation);
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
        elevator.setDefaultCommand(new ElevatorManualCommand(elevator, copilot));
        shooter.setDefaultCommand(manualPivotCommand);
        driver.b().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driver.y().onTrue(Commands.runOnce(() -> {
            drivetrain.zeroFieldOrientationManual();
            enableBrakeMode(false);
        }).ignoringDisable(true));

        Command targetDrive = new AutoShootPrepareWithTargeting(driver.getHID(), drivetrain, shooter)
                .alongWith(leds.commandShowPattern(
                        new LedPatternProgressBar(shooter::getSpinUpPercent, "#00a9ff", 33),
                        Leds.PatternLevel.DRIVER_LOCK));
        Command overstageTargetDrive = new OverStageShootPrepareWithTargeting(driver.getHID(), drivetrain, shooter)
                .alongWith(leds.commandShowPattern(
                        new LedPatternProgressBar(shooter::getSpinUpPercent, "#f700ff", 33),
                        Leds.PatternLevel.DRIVER_LOCK));
        Command counterShuffleDrive = new CounterShufflePrepareWithTargeting(driver.getHID(), drivetrain, shooter)
                .alongWith(leds.commandShowPattern(
                        new LedPatternProgressBar(shooter::getSpinUpPercent, "#00ffbc", 33),
                        Leds.PatternLevel.DRIVER_LOCK));
        Command ampLock = new DriveWithAmpSideLock(drivetrain, driver.getHID())
                .alongWith(leds.commandShowPattern(new LedPatternWave("#ffffff"), Leds.PatternLevel.DRIVER_LOCK));
        Command cancelAlignment = Commands.runOnce(() -> {
            targetDrive.cancel();
            overstageTargetDrive.cancel();
            ampLock.cancel();
            counterShuffleDrive.cancel();
        });
        driver.x()
                .whileTrue(IntakeAndFeed.withDefaults(indexer))
                .onFalse(cancelAlignment);

        driver.rightBumper().toggleOnTrue(targetDrive);
        driver.leftBumper().toggleOnTrue(overstageTargetDrive);

        kidPilot.leftBumper().or(kidPilot.leftTrigger()).whileTrue(new ContinuousIntakeCommand(indexer, leds, 1));
        kidPilot.rightBumper().or(kidPilot.rightTrigger()).whileTrue(new ShootSequence(shooter, indexer));

        // TODO remove this
        if (isTuningMode()) {
            driver.leftTrigger().whileTrue(new AutoShootPrepareWhileMoving(driver.getHID(), drivetrain, shooter).alongWith(leds.commandShowPattern(new LedPatternWave("#00ff00"), Leds.PatternLevel.DRIVER_LOCK)));
            driver.a().whileTrue(new TuneShooterCommand(shooter, indexer, drivetrain::getPose));
        }

        driver.povDown().and(() -> !DriverStation.isFMSAttached()).onTrue(Commands.runOnce(() -> drivetrain.setPose(new Pose2d(Units.inchesToMeters(260), Units.inchesToMeters(161.62), Rotation2d.fromRadians(0)))).ignoringDisable(true));

        driver.rightTrigger(0.95).toggleOnTrue(counterShuffleDrive);

        driver.rightStick().onTrue(cancelAlignment);

        copilot.back().onTrue(Commands.runOnce(shooter::zeroPivotToCancoder).andThen(Commands.print("BACK IS PRESSED")));

        copilot.leftBumper().whileTrue(new AmpScoreSequence(tramp, indexer, elevator));
        Command intakeCommand = new ContinuousIntakeCommand(indexer, leds, 1)
                .deadlineWith(CommandUtils.rumbleCommand(driver, 0.5), CommandUtils.rumbleCommand(copilot, 0.5));
        copilot.rightBumper().whileTrue(intakeCommand);

        copilot.povDown().whileTrue(indexer.commandRunIntake(-1));
        copilot.povUp().whileTrue(indexer.commandRunIntake(1));
        copilot.povRight().whileTrue(IntakeAndFeed.withDefaults(indexer)).onFalse(cancelAlignment);
        copilot.povLeft().onTrue(Commands.runOnce(()->elevator.setFlipper(true))).onFalse(Commands.runOnce(()->elevator.setFlipper(false)));


        copilot.rightTrigger(0.95).whileTrue(tramp.commandRun(1));
        copilot.leftTrigger(0.95).whileTrue(new ClimbAlignment(drivetrain, elevator, tramp, indexer));


        copilot.x().whileTrue(new ShootSequence(shooter, indexer));
        copilot.a().whileTrue(new AmpScoreStageSequence(indexer, tramp, elevator).alongWith(ampLock));
        copilot.b()
                .and(shooter::areFlywheelsSpunUp)
                .and(() -> targetDrive.isScheduled()
                        || overstageTargetDrive.isScheduled()
                        || counterShuffleDrive.isScheduled())
                .whileTrue(IntakeAndFeed.withDefaults(indexer))
                .onFalse(cancelAlignment);
        copilot.y().whileTrue(new StageTrampCommand(tramp, indexer));

//        copilot.leftTrigger(0.5).whileTrue(new ElevatorSetpointCommand(elevator, ElevatorState.CLIMB));


        new Trigger(() -> elevator.getPosition() > 0.1).debounce(0.1)
                .whileTrue(PrepareShooterCommand.lowerPivot(shooter));

        new Trigger(indexer::isNoteStaged).debounce(0.05)
                .onTrue(CommandUtils.rumbleCommandTimed(driver.getHID(), 1, 1))
                .whileTrue(leds.commandShowIntakePattern(SimpleLedPattern.solid("#ff0000")));

        new Trigger(tramp::isNoteStaged).debounce(0.1)
                .whileTrue(leds.commandShowTrampPattern(SimpleLedPattern.solid("#ff9900")));

        new Trigger(indexer::isNoteStaged).and(intakeCommand::isScheduled).onTrue(CommandUtils.rumbleCommandTimed(driver.getHID(), 0.8, 0.4));


        BooleanSupplier isPreMatch = () -> (!DriverStation.isDSAttached() || !DriverStation.isFMSAttached() || DriverStation.isAutonomous()) && DriverStation.isDisabled();
        Command brakeModeCommand = Commands.runOnce(
                () -> {
                    if (!isPreMatch.getAsBoolean()) {return;}
                    if (!isBrakeMode) {
                        enableBrakeMode(true);
                    } else {
                        disableBrakeMode();
                    }
                }
        ).ignoringDisable(true);

        new Trigger(tramp::isNoteStaged).debounce(0.1)
                .onTrue(brakeModeCommand);

        driver.start().onTrue(brakeModeCommand);
        copilot.start().onTrue(brakeModeCommand);

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

        }
        autos.add(new AutoCommand("WheelRadiusChar", new WheelRadiusCharacterization(drivetrain, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE)));
        autos.addDefault(new AutoCommand("Dwayne :skull:"));
        autos.add(new AmpLanePADEF(drivetrain, shooter, indexer));
        autos.add(new AmpLanePAEDF(drivetrain, shooter, indexer));
//        autos.add(new AutoCommand("SubwooferShot", new ShootSequence(shooter, indexer)));
//        autos.add(new DriveSinglePath("Taxi", drivetrain));
//        autos.add(new DriveSinglePath("Sprint", drivetrain));
//        autos.add(new DriveSinglePath("CenterLaneSprint", drivetrain, true, true));
//        autos.add(new AmpLanePADESprint(drivetrain, shooter, indexer));
//        autos.add(new CenterLanePSubSprint(drivetrain, shooter, indexer));
//        autos.add(new CenterLanePCBADSprint(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBAFSprint(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBAFE(drivetrain, shooter, indexer));
        autos.add(new CenterLanePDEABC(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBAFG(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBAEF(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBAGF(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBADE(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBAED(drivetrain, shooter, indexer));
        autos.add(new CenterLanePCBA(drivetrain, shooter, indexer));
//        autos.add(new CenterLanePCBA(drivetrain, shooter, indexer));
//        autos.add(new CenterLanePBDA(drivetrain, shooter, indexer));
//        autos.add(new CenterLanePSubCSubBSubASubFSub(drivetrain, shooter, indexer));
////        autos.add(new CenterLanePSubCSubBSubFSub(drivetrain, shooter, indexer));
//        autos.add(new CenterLanePSubCSubBSubASub(drivetrain, shooter, indexer));
        autos.add(new SourceLanePHGF(drivetrain, shooter, indexer));
        autos.add(new SourceLanePGHSprint(drivetrain, shooter, indexer));
        autos.add(new SourceLanePGFE(drivetrain, shooter, indexer));
        autos.add(new SourceLanePGFH(drivetrain, shooter, indexer));
//        autos.addDefault(new ATestAuto(drivetrain, shooter, indexer));
        autos.add(new AutoCommand("Subwoofer Shot", ShootSequence.forAutoSubwoofer(shooter, indexer)));
    }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoManager.getInstance().getSelected();
    }

    public void enableBrakeMode(boolean requireRobotDisabled) {
        if (!isBrakeMode && (!requireRobotDisabled || !DriverStation.isEnabled())) {
            shooter.setPivotBrakeMode(true);
            indexer.setIntakeBrakeMode(true);
            elevator.setBrakeMode(true);
            drivetrain.setBrakeMode(true);
            leds.clearPattern(Leds.Zone.MAIN, Leds.PatternLevel.COAST_STATE);
            isBrakeMode = true;
            Logger.recordOutput("brakeMode", true);
        }
    }

    public void disableBrakeMode() {
        if (isBrakeMode && !DriverStation.isEnabled()) {
            shooter.setPivotBrakeMode(false);
            indexer.setIntakeBrakeMode(false);
            elevator.setBrakeMode(false);
            drivetrain.setBrakeMode(false);
            leds.setPattern(Leds.Zone.MAIN, new LedPatternBreathing("#f000c7"), Leds.PatternLevel.COAST_STATE);
            isBrakeMode = false;
            Logger.recordOutput("brakeMode", false);
        }
    }
}
