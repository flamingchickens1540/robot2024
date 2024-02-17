package org.team1540.robot2024;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.team1540.robot2024.Constants.Elevator.ElevatorState;
import org.team1540.robot2024.commands.FeedForwardCharacterization;
import org.team1540.robot2024.commands.DriveWithSpeakerTargetingCommand;
import org.team1540.robot2024.commands.SwerveDriveCommand;
import org.team1540.robot2024.commands.elevator.ElevatorManualCommand;
import org.team1540.robot2024.commands.elevator.ElevatorSetpointCommand;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.commands.shooter.ShootSequence;
import org.team1540.robot2024.commands.autos.*;
import org.team1540.robot2024.subsystems.drive.*;
import org.team1540.robot2024.subsystems.elevator.Elevator;
import org.team1540.robot2024.subsystems.elevator.ElevatorIO;
import org.team1540.robot2024.subsystems.elevator.ElevatorIOSim;
import org.team1540.robot2024.subsystems.elevator.ElevatorIOTalonFX;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.indexer.IndexerIO;
import org.team1540.robot2024.subsystems.indexer.IndexerIOSim;
import org.team1540.robot2024.subsystems.indexer.IndexerIOSparkMax;
import org.team1540.robot2024.subsystems.led.Leds;
import org.team1540.robot2024.subsystems.led.patterns.LedPatternFlame;
import org.team1540.robot2024.subsystems.shooter.*;
import org.team1540.robot2024.subsystems.tramp.Tramp;
import org.team1540.robot2024.subsystems.tramp.TrampIO;
import org.team1540.robot2024.subsystems.tramp.TrampIOSim;
import org.team1540.robot2024.subsystems.tramp.TrampIOSparkMax;
import org.team1540.robot2024.subsystems.vision.AprilTagVision;
import org.team1540.robot2024.subsystems.vision.AprilTagVisionIO;
import org.team1540.robot2024.subsystems.vision.AprilTagVisionIOLimelight;
import org.team1540.robot2024.subsystems.vision.AprilTagVisionIOSim;
import org.team1540.robot2024.util.auto.AutoCommand;
import org.team1540.robot2024.util.auto.AutoManager;
import org.team1540.robot2024.util.PhoenixTimeSyncSignalRefresher;
import org.team1540.robot2024.util.swerve.SwerveFactory;
import org.team1540.robot2024.util.vision.VisionPoseAcceptor;

import static org.team1540.robot2024.Constants.SwerveConfig;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
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
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drivetrain =
                        new Drivetrain(
                                new GyroIOPigeon2(odometrySignalRefresher),
                                new ModuleIOTalonFX(SwerveFactory.getModuleMotors(SwerveConfig.FRONT_LEFT, SwerveFactory.SwerveCorner.FRONT_LEFT), odometrySignalRefresher),
                                new ModuleIOTalonFX(SwerveFactory.getModuleMotors(SwerveConfig.FRONT_RIGHT, SwerveFactory.SwerveCorner.FRONT_RIGHT), odometrySignalRefresher),
                                new ModuleIOTalonFX(SwerveFactory.getModuleMotors(SwerveConfig.BACK_LEFT, SwerveFactory.SwerveCorner.BACK_LEFT), odometrySignalRefresher),
                                new ModuleIOTalonFX(SwerveFactory.getModuleMotors(SwerveConfig.BACK_RIGHT, SwerveFactory.SwerveCorner.BACK_RIGHT), odometrySignalRefresher));
                tramp = new Tramp(new TrampIOSparkMax());
                shooter = new Shooter(new ShooterPivotIOTalonFX(), new FlywheelsIOTalonFX());
                elevator = new Elevator(new ElevatorIOTalonFX());
                indexer =
                        new Indexer(
                                new IndexerIOSparkMax()
                        );
                aprilTagVision = new AprilTagVision(
                        new AprilTagVisionIOLimelight(Constants.Vision.FRONT_CAMERA_NAME, Constants.Vision.FRONT_CAMERA_POSE),
                        new AprilTagVisionIOLimelight(Constants.Vision.REAR_CAMERA_NAME, Constants.Vision.REAR_CAMERA_POSE),
                        drivetrain::addVisionMeasurement,
                        () -> 0.0, // TODO: ACTUALLY GET ELEVATOR HEIGHT HERE
                        new VisionPoseAcceptor(drivetrain::getChassisSpeeds, () -> 0.0)); // TODO: ACTUALLY GET ELEVATOR VELOCITY HERE
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drivetrain =
                        new Drivetrain(
                                new GyroIO() {},
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim(),
                                new ModuleIOSim());
                tramp = new Tramp(new TrampIOSim());
                shooter = new Shooter(new ShooterPivotIOSim(), new FlywheelsIOSim());
                elevator = new Elevator(new ElevatorIOSim());
                aprilTagVision =
                        new AprilTagVision(
                                new AprilTagVisionIOSim(Constants.Vision.FRONT_CAMERA_NAME, Constants.Vision.FRONT_CAMERA_POSE, drivetrain::getPose),
                                new AprilTagVisionIOSim(Constants.Vision.REAR_CAMERA_NAME, Constants.Vision.REAR_CAMERA_POSE, drivetrain::getPose),
                                ignored -> {},
                                () -> 0.0, // TODO: ACTUALLY GET ELEVATOR HEIGHT HERE
                                new VisionPoseAcceptor(drivetrain::getChassisSpeeds, () -> 0.0)); // TODO: ACTUALLY GET ELEVATOR VELOCITY HERE
                indexer =
                        new Indexer(
                                new IndexerIOSim()
                        );
                break;
            default:
                // Replayed robot, disable IO implementations
                drivetrain =
                        new Drivetrain(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {});
                shooter = new Shooter(new ShooterPivotIO() {}, new FlywheelsIO() {});
                elevator = new Elevator(new ElevatorIO() {});
                aprilTagVision =
                        new AprilTagVision(
                                new AprilTagVisionIO() {},
                                new AprilTagVisionIO() {},
                                (ignored) -> {},
                                () -> 0.0,
                                new VisionPoseAcceptor(drivetrain::getChassisSpeeds, () -> 0.0)
                        );

                indexer = new Indexer(new IndexerIO() {});
                tramp = new Tramp(new TrampIO() {});

                break;
        }



        // Set up FF characterization routines
        AutoManager.getInstance().addAuto(
                new AutoCommand(
                        "Drive FF Characterization",
                        new FeedForwardCharacterization(
                                drivetrain, drivetrain::runCharacterizationVolts, drivetrain::getCharacterizationVelocity
                        )
                )
        );

        AutoManager.getInstance().addAuto(
                new AutoCommand(
                        "Flywheels FF Characterization",
                        new FeedForwardCharacterization(
                                shooter, volts -> shooter.setFlywheelVolts(volts, volts), () -> shooter.getLeftFlywheelSpeed() / 60
                        )
                )
        );

        AutoManager.getInstance().addDefaultAuto(new AmpLanePABCSprint(drivetrain, shooter, indexer));
        AutoManager.getInstance().addAuto(new SourceLanePHGFSprint(drivetrain));
        AutoManager.getInstance().addAuto(new PathVisualising(drivetrain));

        // Configure the button bindings
        configureButtonBindings();
        configureLedBindings();
    }

    private void configureLedBindings() {
        leds.setFatalPattern(LedPatternFlame::new);
        new Trigger(DriverStation::isDSAttached)
                .onTrue(Commands.runOnce(leds::clearFatalPattern)
                            .ignoringDisable(true))
                .onFalse(Commands.runOnce(() -> leds.setFatalPattern(LedPatternFlame::new))
                            .ignoringDisable(true));
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver));
        elevator.setDefaultCommand(new ElevatorManualCommand(elevator, copilot));
        indexer.setDefaultCommand(new IntakeCommand(indexer, tramp));

        driver.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driver.y().toggleOnTrue(new DriveWithSpeakerTargetingCommand(drivetrain, driver));
        driver.b().onTrue(
                Commands.runOnce(
                        () -> drivetrain.setPose(new Pose2d(drivetrain.getPose().getTranslation(), new Rotation2d())),
                        drivetrain
                ).ignoringDisable(true)
        );

        copilot.rightBumper().onTrue(new ElevatorSetpointCommand(elevator, ElevatorState.TOP));
        copilot.leftBumper().onTrue(new ElevatorSetpointCommand(elevator, ElevatorState.BOTTOM));
        copilot.a().onTrue(new ShootSequence(shooter, indexer))
                .onFalse(Commands.runOnce(shooter::stopFlywheels, shooter));
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
