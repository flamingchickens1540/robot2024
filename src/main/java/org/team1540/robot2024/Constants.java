package org.team1540.robot2024;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.team1540.robot2024.util.shooter.ShooterSetpoint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DIO {
        public static final int INTAKE_BEAM_BREAK = 6;
        public static final int SHOOTER_BEAM_BREAK = 4;
        public static final int INDEXER_BEAM_BREAK = 5;
        public static final int TRAMP_BEAM_BREAK = 9;
    }
    public static final boolean IS_COMPETITION_ROBOT = true;
    // Whether to pull PID constants from SmartDashboard
    private static final boolean tuningMode = true; // TODO: DO NOT SET TO TRUE FOR COMP
    private static final Mode simMode = Mode.SIM; // Can also be Mode.REPLAY

    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : simMode;
    public static final class Leds {
        public static final int LED_STRIP_PORT_PWM = 9;
        public static final int LED_STRIP_LENGTH= 41;
    }
    public enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    public static final double LOOP_PERIOD_SECS = 0.02;

    public static class SwerveConfig {
        public static final String CAN_BUS  = "swerve";
        public static final double CAN_UPDATE_FREQUENCY_HZ = 200.0;

        public static final int FRONT_LEFT  = 4;
        public static final int FRONT_RIGHT = 3;
        public static final int BACK_LEFT   = 1;
        public static final int BACK_RIGHT  = 7;

        public static final int PIGEON_ID = 9;
    }

    public static class Drivetrain {
        public static final boolean IS_L3 = true;
        public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (IS_L3 ? 16.0 / 28.0 : 17.0 / 27.0) * (45.0 / 15.0);
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
        public static final boolean IS_TURN_MOTOR_INVERTED = true;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(1.9639138572591197);

        public static final double MAX_LINEAR_SPEED = Units.feetToMeters(IS_L3 ? 16.0 : 15.7);
        public static final double TRACK_WIDTH_X = Units.inchesToMeters(18.75);
        public static final double TRACK_WIDTH_Y = Units.inchesToMeters(19.75);
        public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    }

    public static class Auto {
        public static final double LINEAR_ACCEL_TIME_SECS = 4.0/3.0;
        public static final double ANGULAR_ACCEL_TIME_SECS = 4.0/3.0;
        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                Drivetrain.MAX_LINEAR_SPEED, Drivetrain.MAX_LINEAR_SPEED * LINEAR_ACCEL_TIME_SECS,
                Constants.Drivetrain.MAX_ANGULAR_SPEED,
                Constants.Drivetrain.MAX_ANGULAR_SPEED * ANGULAR_ACCEL_TIME_SECS);

        public static final PathConstraints STAGE_PATH_CONSTRAINTS = new PathConstraints(
                3.0, 3.0,
                1,
                0.3);
    }

    public static class Indexer {
        public static final int INTAKE_ID = 13;
        public static final int FEEDER_ID = 15;

        public static final double FEEDER_KP = 0.5;
        public static final double FEEDER_KI = 0.1;
        public static final double FEEDER_KD = 0.001;
        public static final double FEEDER_KS = 0.0;
        public static final double FEEDER_KV = 0.0;
        public static final double FEEDER_GEAR_RATIO = 1.0;
        public static final double INTAKE_GEAR_RATIO = 1.0;
        public static final double INTAKE_MOI = 0.025;
        public static final double FEEDER_MOI = 0.025;
        public static final int VELOCITY_ERR_TOLERANCE_RPM = 10;
    }


    public static class Vision {
        public static class AprilTag {
            public static final String FRONT_CAMERA_NAME = "limelight-front";
            public static final String REAR_CAMERA_NAME = "limelight-rear";
            public static final Pose3d FRONT_CAMERA_POSE = new Pose3d(0.086018, 0, 0.627079, new Rotation3d(0, Math.toRadians(-40.843), 0));
            public static final Pose3d REAR_CAMERA_POSE = new Pose3d(0.046049, 0, 0.540510, new Rotation3d(Math.PI, Math.toRadians(10), Math.PI+Math.toRadians(1.55)));
            public static final boolean TAKE_SNAPSHOTS = true;
            public static final double SNAPSHOT_PERIOD_SECS = 1;
            public static final double XY_STD_DEV_COEFF = 0.1;
            public static final double ROT_STD_DEV_COEFF = 0.5;
            public static final double MAX_AMBIGUITY_RATIO = 0.3;
            public static final double MAX_ACCEPTED_ROT_SPEED_RAD_PER_SEC = 1.0;
            public static final double MAX_ACCEPTED_LINEAR_SPEED_MPS = 4.0;
            public static final double MIN_ACCEPTED_NUM_TAGS = 1;
            public static final double MAX_ACCEPTED_AVG_TAG_DIST_METERS = 8.0;
            public static final double MAX_ACCEPTED_ELEVATOR_SPEED_MPS = 0.05;
            public static final int SIM_RES_WIDTH = 1280;
            public static final int SIM_RES_HEIGHT = 960;
            public static final Rotation2d SIM_DIAGONAL_FOV = Rotation2d.fromDegrees(70);
            public static final double SIM_FPS = 14.5;
            public static final double SIM_AVG_LATENCY_MS = 100;
        }

        public static class Gamepiece {
            public static final String CAMERA_NAME = "limelight-vision";
            public static final int PIPELINE_INDEX = 0;
            public static final Pose3d CAMERA_POSE = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)); //TODO FIND THIS POSE
        }
    }


    public static class Shooter {
        public static class Flywheels {
            public static final int LEFT_ID = 12;
            public static final int RIGHT_ID = 11;

            public static final double GEAR_RATIO = 24.0 / 36.0;
            public static final double SIM_MOI = 4.08232288e-4;

            public static final double KP = 0.3;
            public static final double KI = 0.2;
            public static final double KD = 0.0;
            public static final double KS = 0.26925;
            public static final double KV = 0.07485;

            public static final double ERROR_TOLERANCE_RPM = 2000;
            public static final double LEFT_RPM = 6750.0;
            public static final double RIGHT_RPM = 4500.0;
        }

        public static class Pivot {
            public static final int MOTOR_ID = 9;
            public static final int CANCODER_ID = 10;

            // TODO: determine ratios
            public static final double CANCODER_TO_PIVOT = 28.0 / 15.0;
            public static final double MOTOR_TO_CANCODER = 56.0;
            public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromRadians(0.01215595 - 0.00548239159444);

            public static final double CHAIN_FACTOR = 1.04;
            public static final double TOTAL_GEAR_RATIO = MOTOR_TO_CANCODER * CANCODER_TO_PIVOT;
            public static final double SIM_LENGTH_METERS = Units.inchesToMeters(12.910);
            // TODO: find the moi
            public static final double SIM_MOI = 0.04064471269;

            public static final Rotation2d MAX_ANGLE = Rotation2d.fromRotations(0.174);
            public static final Rotation2d MIN_ANGLE = Rotation2d.fromRotations(0.044);

            public static final Rotation2d REAL_ZEROED_ANGLE = Rotation2d.fromDegrees(8.5).minus(Rotation2d.fromRadians(0.3153));
            public static final double PIVOT_HEIGHT = Units.inchesToMeters(10.5);

            public static final double KP = 400.0;
            public static final double KI = 0.0;
            public static final double KD = 20.0;
            public static final double KS = 0.0;
            public static final double KG = 0.5;
            public static final double KV = 0.0;

            public static final double SIM_KP = 254;
            public static final double SIM_KI = 0.0;
            public static final double SIM_KD = 0.0;
            public static final double SIM_KS = 0.0;
            public static final double SIM_KG = 0.15;
            public static final double SIM_KV = 0.187;

            public static final double CRUISE_VELOCITY_RPS = 1.0;
            public static final double MAX_ACCEL_RPS2 = 2;
            public static final double JERK_RPS3 = 2000;

            public static final ShooterSetpoint HUB_SHOOT = new ShooterSetpoint(Rotation2d.fromRadians(1.06184));
            public static final ShooterSetpoint PODIUM_SHOOT = new ShooterSetpoint(0.07, 6000,5000);


            public static final Rotation2d ERROR_TOLERANCE = Rotation2d.fromDegrees(0.7);
        }
    }

    public static class Elevator {
        public static final double CHAIN_HEIGHT_METERS = Units.inchesToMeters(28.25);
        public static final double MINIMUM_HEIGHT = Units.inchesToMeters(-2); //TODO: Does this make it angry?
        public static final double CLIMBING_HOOKS_MINIMUM_HEIGHT = Units.inchesToMeters(12.0);
        public static final double MAX_HEIGHT = 0.52; //TODO: Fix these constants to be more accurate
        public static final double CLIMBING_HOOKS_MAX_HEIGHT = CLIMBING_HOOKS_MINIMUM_HEIGHT + MAX_HEIGHT - MINIMUM_HEIGHT;

        public static final double GEAR_RATIO = 11.571;
        public static final int LEADER_ID = 7; // RIGHT
        public static final int FOLLOWER_ID = 8; // LEFT

        public static final int LEFT_FLIPPER_ID = 1;

        public static final int RIGHT_FLIPPER_ID = 0;
        public static final double KS = 0.03178;
        public static final double KV = 0.82983;
        public static final double KA = 0.00;
        public static final double KP = 300;
        public static final double KI = 50;
        public static final double KD = 1;
        public static final double KG = 0;
        public static final double CRUISE_VELOCITY_MPS = 1.2;
        public static final double MAXIMUM_ACCELERATION_MPS2 = 50;
        public static final double JERK_MPS3 = 200;
        public static final double SPROCKET_RADIUS_M = Units.inchesToMeters(1.751/2);
        public static final double SPROCKET_CIRCUMFERENCE_M = 2 * SPROCKET_RADIUS_M * Math.PI;
        public static final double MOTOR_ROTS_PER_METER = GEAR_RATIO / SPROCKET_CIRCUMFERENCE_M;
        public static final double POS_ERR_TOLERANCE_METERS = 0.01;
        public static final double SIM_CARRIAGE_MASS_KG = 1.55; //TODO: check this number :)

        public enum ElevatorState {
            /**
             * At max height :D
             */
            TOP(MAX_HEIGHT),
            /**
             * At minimum height :D
             */
            BOTTOM(MINIMUM_HEIGHT),
            /**
             * At height for aligning for climbing
             */
            CLIMB(0.1),
            /**
             * At height for top of initial climb :D
             */
            AMP(0.3);

            public final double heightMeters;

            ElevatorState(double heightMeters) {
                this.heightMeters = heightMeters;
            }
        }
    }

    public static class Tramp {
        public static final double GEAR_RATIO = 9.0;
        public static final int MOTOR_ID = 17;
    }

    public static class Targeting {
        public static final double ROT_KP = 0.3;
        public static final double ROT_KI = 0.0;
        public static final double ROT_KD = 0.002;


        public static final double SPEAKER_CENTER_HEIGHT = Units.inchesToMeters(80.4375);
        public static final double STAGE_MAX_HEIGHT = Units.feetToMeters(7.365);

        private static final Pose2d SPEAKER_POSE =
                new Pose2d(Units.inchesToMeters(8.861), Units.inchesToMeters(218), new Rotation2d());
        private static final Pose2d SHUFFLE_POSE =
                new Pose2d(SPEAKER_POSE.getX() + 2, SPEAKER_POSE.getY() + 2, new Rotation2d());
        private static final Pose2d COUNTER_SHUFFLE_POSE =
                new Pose2d(SPEAKER_POSE.getX() + 8.27, SPEAKER_POSE.getY(), new Rotation2d());

        public static boolean getFlipped(){
            return DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red;
        }

        public static Pose2d getSpeakerPose() {
            return getFlipped()
                    ? GeometryUtil.flipFieldPose(SPEAKER_POSE)
                    : SPEAKER_POSE;
        }

        public static Pose2d getShufflePose() {
            return getFlipped()
                    ? GeometryUtil.flipFieldPose(SHUFFLE_POSE)
                    : SHUFFLE_POSE;
        }

        public static Pose2d getCounterShufflePose() {
            return getFlipped()
                    ? GeometryUtil.flipFieldPose(COUNTER_SHUFFLE_POSE)
                    : COUNTER_SHUFFLE_POSE;
        }
    }

    public static boolean isTuningMode() {
        return tuningMode && !DriverStation.isFMSAttached();
    }
}
