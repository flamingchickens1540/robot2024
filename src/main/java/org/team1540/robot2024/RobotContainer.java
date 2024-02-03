package org.team1540.robot2024;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.team1540.robot2024.commands.FeedForwardCharacterization;
import org.team1540.robot2024.commands.SwerveDriveCommand;
import org.team1540.robot2024.commands.indexer.IntakeCommand;
import org.team1540.robot2024.subsystems.drive.*;
import org.team1540.robot2024.subsystems.tramp.Tramp;
import org.team1540.robot2024.subsystems.tramp.TrampIO;
import org.team1540.robot2024.subsystems.tramp.TrampIOSim;
import org.team1540.robot2024.subsystems.tramp.TrampIOSparkMax;
import org.team1540.robot2024.subsystems.shooter.*;
import org.team1540.robot2024.subsystems.vision.AprilTagVision;
import org.team1540.robot2024.subsystems.vision.AprilTagVisionIO;
import org.team1540.robot2024.subsystems.vision.AprilTagVisionIOLimelight;
import org.team1540.robot2024.subsystems.vision.AprilTagVisionIOSim;
import org.team1540.robot2024.util.PhoenixTimeSyncSignalRefresher;
import org.team1540.robot2024.subsystems.indexer.Indexer;
import org.team1540.robot2024.subsystems.indexer.IndexerIO;
import org.team1540.robot2024.subsystems.indexer.IndexerIOSim;
import org.team1540.robot2024.subsystems.indexer.IndexerIOSparkMax;
import org.team1540.robot2024.util.swerve.SwerveFactory;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
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
    public final Indexer indexer;
    public final AprilTagVision aprilTagVision;

    // Controller
    public final CommandXboxController driver = new CommandXboxController(0);
    public final CommandXboxController copilot = new CommandXboxController(1);


    // Dashboard inputs
    public final LoggedDashboardChooser<Command> autoChooser;

    public final PhoenixTimeSyncSignalRefresher odometrySignalRefresher = new PhoenixTimeSyncSignalRefresher(SwerveConfig.CAN_BUS);

    // TODO: testing dashboard inputs, remove for comp
    public final LoggedDashboardNumber leftFlywheelSetpoint = new LoggedDashboardNumber("Shooter/Flywheels/leftSetpoint", 6000);
    public final LoggedDashboardNumber rightFlywheelSetpoint = new LoggedDashboardNumber("Shooter/Flywheels/rightSetpoint", 6000);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
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
                aprilTagVision =
                        new AprilTagVision(
                                new AprilTagVisionIO() {},
                                new AprilTagVisionIO() {},
                                (ignored) -> {},
                                () -> 0.0,
                                new VisionPoseAcceptor(drivetrain::getChassisSpeeds, () -> 0.0));

                indexer =
                        new Indexer(
                                new IndexerIO() {}
                        );

                tramp = new Tramp(new TrampIO() {});

                break;
        }


        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up FF characterization routines
        autoChooser.addOption(
                "Drive FF Characterization",
                new FeedForwardCharacterization(
                        drivetrain, drivetrain::runCharacterizationVolts, drivetrain::getCharacterizationVelocity));
        autoChooser.addOption(
                "Flywheels FF Characterization",
                new FeedForwardCharacterization(
                        shooter, volts -> shooter.setFlywheelVolts(volts, volts), () -> shooter.getLeftFlywheelSpeed() / 60));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, driver));
        driver.x().onTrue(Commands.runOnce(drivetrain::stopWithX, drivetrain));
        driver.b().onTrue(
                Commands.runOnce(
                        () -> drivetrain.setPose(new Pose2d(drivetrain.getPose().getTranslation(), new Rotation2d())),
                        drivetrain
                ).ignoringDisable(true)
        );

        copilot.a().onTrue(shooter.spinUpCommand(leftFlywheelSetpoint.get(), rightFlywheelSetpoint.get()))
                .onFalse(Commands.runOnce(shooter::stopFlywheels, shooter));

        driver.a().onTrue(new IntakeCommand(indexer));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
