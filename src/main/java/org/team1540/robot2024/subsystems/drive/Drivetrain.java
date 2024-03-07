package org.team1540.robot2024.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.util.auto.LocalADStarAK;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.util.PhoenixTimeSyncSignalRefresher;
import org.team1540.robot2024.util.swerve.SwerveFactory;
import org.team1540.robot2024.util.vision.TimestampedVisionPose;

import static org.team1540.robot2024.Constants.Drivetrain.*;

public class Drivetrain extends SubsystemBase {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private Rotation2d fieldOrientationOffset = new Rotation2d();
    private boolean forceModuleAngleChange = false;

    private final SwerveDrivePoseEstimator poseEstimator;

    private static boolean hasInstance = false;

    private Drivetrain(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        if (hasInstance) throw new IllegalStateException("Instance of drivetrain already exists");
        hasInstance = true;

        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                rawGyroRotation,
                getModulePositions(),
                new Pose2d(),
                // TODO: tune std devs (scale vision by distance?)
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, 5.0)); // Trust the gyro more than the AprilTags

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                () -> kinematics.toChassisSpeeds(getModuleStates()),
                this::runVelocity,
                new HolonomicPathFollowerConfig(MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
                () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> Logger.recordOutput("Pathplanner/ActivePath", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> Logger.recordOutput("Pathplanner/TargetPosition", targetPose));
    }

    public static Drivetrain createReal(PhoenixTimeSyncSignalRefresher odometrySignalRefresher) {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real drivetrain on simulated robot", false);
        }
        return new Drivetrain(
                new GyroIOPigeon2(odometrySignalRefresher),
                new ModuleIOTalonFX(SwerveFactory.getModuleMotors(Constants.SwerveConfig.FRONT_LEFT, SwerveFactory.SwerveCorner.FRONT_LEFT), odometrySignalRefresher),
                new ModuleIOTalonFX(SwerveFactory.getModuleMotors(Constants.SwerveConfig.FRONT_RIGHT, SwerveFactory.SwerveCorner.FRONT_RIGHT), odometrySignalRefresher),
                new ModuleIOTalonFX(SwerveFactory.getModuleMotors(Constants.SwerveConfig.BACK_LEFT, SwerveFactory.SwerveCorner.BACK_LEFT), odometrySignalRefresher),
                new ModuleIOTalonFX(SwerveFactory.getModuleMotors(Constants.SwerveConfig.BACK_RIGHT, SwerveFactory.SwerveCorner.BACK_RIGHT), odometrySignalRefresher));
    }

    public static Drivetrain createSim() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated drivetrain on real robot", false);
        }
        return new Drivetrain(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
    }

    public static Drivetrain createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy drivetrain on real robot", false);
        }
        return new Drivetrain(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drivetrain/Gyro", gyroInputs);
        for (Module module : modules) {
            module.periodic();
        }

        if (DriverStation.isDisabled()) {
            // Stop moving when disabled
            for (Module module : modules) module.stop();

            // Log empty setpoint states when disabled
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]{});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
        }

        // Calculate module deltas
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = modules[i].getPositionDelta();
        }
        // Use gyro rotation if gyro is connected, otherwise use module deltas to calculate rotation delta
        rawGyroRotation =
                gyroInputs.connected ?
                        gyroInputs.yawPosition
                        : rawGyroRotation.plus(Rotation2d.fromRadians(kinematics.toTwist2d(wheelDeltas).dtheta));
        // Update odometry
        poseEstimator.update(rawGyroRotation, getModulePositions());
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i], forceModuleAngleChange);
        }
        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    public void drivePercent(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {
        Rotation2d linearDirection = new Rotation2d(xPercent, yPercent);
        double linearMagnitude = Math.hypot(xPercent, yPercent);

        Translation2d linearVelocity =
                new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();

        // Convert to chassis speeds
        runVelocity(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
                                linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
                                rotPercent * getMaxAngularSpeedRadPerSec(),
                                rawGyroRotation.minus(fieldOrientationOffset))
                        : new ChassisSpeeds(
                                linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
                                linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
                                rotPercent * getMaxAngularSpeedRadPerSec())
        );
    }

    /**
     * Stops the drive.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        forceModuleAngleChange = true;
        kinematics.resetHeadings(headings);
        stop();
        forceModuleAngleChange = false;
    }

    /**
     * Runs forwards at the commanded voltage.
     */
    public void runCharacterizationVolts(double volts) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(volts);
        }
    }

    /**
     * Returns the average drive velocity in radians/sec.
     */
    public double getCharacterizationVelocity() {
        double driveVelocityAverage = 0.0;
        for (Module module : modules) {
            driveVelocityAverage += module.getCharacterizationVelocity();
        }
        return driveVelocityAverage / 4.0;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current odometry rotation.
     */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void zeroFieldOrientationManual() {
        fieldOrientationOffset = rawGyroRotation;
    }

    public void zeroFieldOrientation() {
        boolean isFlipped = DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
        fieldOrientationOffset =
                rawGyroRotation.minus(isFlipped ? getRotation().plus(Rotation2d.fromDegrees(180)) : getRotation());
    }

    /**
     * Resets the current odometry pose.
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    public void addVisionMeasurement(TimestampedVisionPose visionPose) {
        poseEstimator.addVisionMeasurement(visionPose.poseMeters, visionPose.timestampSecs);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                modules[0].getPosition(),
                modules[1].getPosition(),
                modules[2].getPosition(),
                modules[3].getPosition(),
        };
    }

    /**
     * Returns the maximum linear speed in meters per sec.
     */
    public double getMaxLinearSpeedMetersPerSec() {
        return MAX_LINEAR_SPEED;
    }

    /**
     * Returns the maximum angular speed in radians per sec.
     */
    public double getMaxAngularSpeedRadPerSec() {
        return MAX_ANGULAR_SPEED;
    }

    /**
     * Returns an array of module translations.
     */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[]{
                new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
                new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
                new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
        };
    }
}
