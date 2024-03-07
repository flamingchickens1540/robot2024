package org.team1540.robot2024;

import com.pathplanner.lib.path.PathConstraints;
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
    public static final boolean IS_COMPETITION_ROBOT = true; // Objects.equals(RobotController.getComments(), "comp");
    // Whether to pull PID constants from SmartDashboard
    private static final boolean tuningMode = true; // TODO: DO NOT SET TO TRUE FOR COMP
    private static final Mode simMode = Mode.SIM; // Can also be Mode.REPLAY

    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : simMode;
    public static final class Leds {
        public static final int LED_STRIP_PORT_PWM = 9;
        public static final int LED_STRIP_LENGTH= 80;
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
        public static final String CAN_BUS  = IS_COMPETITION_ROBOT ? "swerve" : "swerve";
        public static final double CAN_UPDATE_FREQUENCY_HZ = 200.0;

        public static final int FRONT_LEFT  = IS_COMPETITION_ROBOT ? 9 : 1;
        public static final int FRONT_RIGHT = IS_COMPETITION_ROBOT ? 2 : 7;
        public static final int BACK_LEFT   = IS_COMPETITION_ROBOT ? 5 : 4;
        public static final int BACK_RIGHT  = IS_COMPETITION_ROBOT ? 6 : 3;

        public static final int PIGEON_ID = 9;
    }

    public static class Drivetrain {
        public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
        public static final boolean IS_TURN_MOTOR_INVERTED = true;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(1.967);

        public static final double MAX_LINEAR_SPEED = Units.feetToMeters(16);
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
    }

    public static class Indexer {
        public static final int BEAM_BREAK_ID = IS_COMPETITION_ROBOT ? 7 : 8;
        public static final int INTAKE_ID = 13;
        public static final int FEEDER_ID = 15;

        // TODO: fix these constants
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
        public static final String FRONT_CAMERA_NAME = "limelight-front";
        public static final String REAR_CAMERA_NAME = "limelight-rear";

        //0.341306
        //0.609832
        // TODO: measure these offsets
        public static final Pose3d FRONT_CAMERA_POSE = new Pose3d(0.0975290, 0, 0.665479, new Rotation3d(0, Math.toRadians(-25), 0));
        public static final Pose3d REAR_CAMERA_POSE = new Pose3d(0.03639, 0, 0.715274, new Rotation3d(Math.PI, 0, Math.PI));

        // TODO: find these values
        public static final double MAX_AMBIGUITY_RATIO = 0.3;
        public static final double MAX_VISION_DELAY_SECS = 0.08;
        public static final double MAX_ACCEPTED_ROT_SPEED_RAD_PER_SEC = 1.0;
        public static final double MAX_ACCEPTED_LINEAR_SPEED_MPS = 4.0;
        public static final double MIN_ACCEPTED_NUM_TAGS = 1;
        public static final double MAX_ACCEPTED_AVG_TAG_DIST_METERS = 8.0;
        public static final double MAX_ACCEPTED_ELEVATOR_SPEED_MPS = 0.05;

        public static final int SIM_RES_WIDTH = 1280;
        public static final int SIM_RES_HEIGHT = 960;
        public static final Rotation2d SIM_DIAGONAL_FOV = Rotation2d.fromDegrees(100);
        public static final double SIM_FPS = 14.5;
        public static final double SIM_AVG_LATENCY_MS = 67.0;
    }


    public static class Shooter {
        public static class Flywheels {
            // TODO: determine ids
            public static final int LEFT_ID = 12;
            public static final int RIGHT_ID = 11;

            public static final double GEAR_RATIO = 24.0 / 36.0;
            public static final double SIM_MOI = 4.08232288e-4;

            // TODO: if it's tuned in simulation, it's tuned in real life
            public static final double KP = 0.53;
            public static final double KI = 0.2;
            public static final double KD = 0.0;
            public static final double KS = 0.26925;
            public static final double KV = 0.07485; // TODO: this is what recalc says, may have to tune

            public static final double ERROR_TOLERANCE_RPM = 100;
        }

        public static class Pivot {
            // TODO: determine ids
            public static final int MOTOR_ID = 9;
            public static final int CANCODER_ID = 10;

            // TODO: figure this out
            public static final double CANCODER_OFFSET_ROTS = -0.2502;
            // TODO: determine ratios
            public static final double CANCODER_TO_PIVOT = 28.0 / 15.0;
            public static final double MOTOR_TO_CANCODER = 56.0;
            public static final double TOTAL_GEAR_RATIO = MOTOR_TO_CANCODER * CANCODER_TO_PIVOT;
            public static final double SIM_LENGTH_METERS = Units.inchesToMeters(12.910);
            // TODO: find the moi
            public static final double SIM_MOI = 0.04064471269;

            public static final Rotation2d MAX_ANGLE = Rotation2d.fromRotations(0.14);
            public static final Rotation2d MIN_ANGLE = Rotation2d.fromRotations(0.01);

            public static final Rotation2d REAL_ZEROED_ANGLE = Rotation2d.fromDegrees(7.5); //TODO Need this number

            public static final double PIVOT_HEIGHT = Units.inchesToMeters(10.5);

            // TODO: tune pid
            public static final double KP = 80.0;
            public static final double KI = 40.0;
            public static final double KD = 0.0;
            public static final double KS = 0.0;
            public static final double KG = 0.0;
            public static final double KV = 0.0;

            public static final double SIM_KP = 254;
            public static final double SIM_KI = 0.0;
            public static final double SIM_KD = 0.0;
            public static final double SIM_KS = 0.0;
            public static final double SIM_KG = 0.15;
            public static final double SIM_KV = 0.187;

            public static final double CRUISE_VELOCITY_RPS = 1.0;
            public static final double MAX_ACCEL_RPS2 = 0.8;
            public static final double JERK_RPS3 = 2000;


            public static final ShooterSetpoint HUB_SHOOT = new ShooterSetpoint(0.125, 4800,4000);
            public static final ShooterSetpoint PODIUM_SHOOT = new ShooterSetpoint(0.07, 6000,5000);


            public static final Rotation2d ERROR_TOLERANCE = Rotation2d.fromDegrees(0.7);
        }
    }

    public static class Elevator {
        public static final double CHAIN_HEIGHT_METERS = Units.inchesToMeters(28.25);
        public static final double MINIMUM_HEIGHT = Units.inchesToMeters(0.0);
        public static final double CLIMBING_HOOKS_MINIMUM_HEIGHT = Units.inchesToMeters(12.0);
        public static final double MAX_HEIGHT = MINIMUM_HEIGHT + Units.inchesToMeters(21.0); //TODO: Fix these constants to be more accurate
        public static final double CLIMBING_HOOKS_MAX_HEIGHT = CLIMBING_HOOKS_MINIMUM_HEIGHT + MAX_HEIGHT - MINIMUM_HEIGHT;

        public static final double GEAR_RATIO = 11.571; //TODO: Get constants right sometime
        public static final int LEADER_ID = 7;
        public static final int FOLLOWER_ID = 8;
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
        public static final double POS_ERR_TOLERANCE_METERS = 0.03;
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
            AMP(0.3); //TODO: Find these values :D

            public final double heightMeters;

            ElevatorState(double heightMeters) {
                this.heightMeters = heightMeters;
            }
        }
    }

    public static class Tramp {
        public static final int BEAM_BREAK_CHANNEL = 9;
        public static final double GEAR_RATIO = 9.0;
        public static final double TRAP_SCORING_TIME_SECONDS = 1.114; //TODO: Find these values :D
        public static final int MOTOR_ID = 17;
    }

    public static class Targeting {
        // TODO: tune these
        public static final double ROT_KP = 0.6;
        public static final double ROT_KI = 0.0;
        public static final double ROT_KD = 0.028;

        public static final Pose2d SPEAKER_POSE =
                new Pose2d(Units.inchesToMeters(8.861), Units.inchesToMeters(218), new Rotation2d());
        public static final Double SPEAKER_CENTER_HEIGHT = Units.inchesToMeters(80.4375);
    }

    public static boolean isTuningMode() {
        return tuningMode && !DriverStation.isFMSAttached();
    }
}
