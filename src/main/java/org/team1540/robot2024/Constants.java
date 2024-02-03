package org.team1540.robot2024;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean IS_COMPETITION_ROBOT = true;
    // Whether to pull PID constants from SmartDashboard
    public static final boolean tuningMode = true; // TODO: DO NOT SET TO TRUE FOR COMP
    private static final Mode simMode = Mode.SIM; // Can also be Mode.REPLAY
    
    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : simMode;

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
        public static final String CAN_BUS = IS_COMPETITION_ROBOT ? "" : "";
        public static final int FRONT_LEFT  = IS_COMPETITION_ROBOT ? 3 : 0;
        public static final int FRONT_RIGHT = IS_COMPETITION_ROBOT ? 4 : 0;
        public static final int BACK_LEFT   = IS_COMPETITION_ROBOT ? 7 : 0;
        public static final int BACK_RIGHT  = IS_COMPETITION_ROBOT ? 1 : 0;
    }

    public static class Drivetrain {
        public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        public static final double TURN_GEAR_RATIO = 150.0 / 7.0;
        public static final boolean IS_TURN_MOTOR_INVERTED = true;
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

        public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5);
        public static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
        public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
        public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
        public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    }

    public static class Shooter {
        public static class Flywheels {
            // TODO: determine ids
            public static final int LEFT_ID = 0;
            public static final int RIGHT_ID = 0;

            public static final double GEAR_RATIO = 24.0 / 36.0;
            public static final double SIM_MOI = 4.08232288e-4;

            // TODO: if it's tuned in simulation, it's tuned in real life
            public static final double KP = 0.4;
            public static final double KI = 0.0;
            public static final double KD = 0.0;
            public static final double KS = 0.01146;
            public static final double KV = 0.07485; // TODO: this is what recalc says, may have to tune

            public static final double ERROR_TOLERANCE_RPM = 50;
        }

        public static class Pivot {
            // TODO: determine ids
            public static final int MOTOR_ID = 0;
            public static final int CANCODER_ID = 0;

            // TODO: figure this out
            public static final double CANCODER_OFFSET_ROTS = 0;
            // TODO: determine ratios
            public static final double CANCODER_TO_PIVOT = 60.0 / 20.0;
            public static final double MOTOR_TO_CANCODER = 33.0;
            public static final double TOTAL_GEAR_RATIO = MOTOR_TO_CANCODER * CANCODER_TO_PIVOT;
            public static final double SIM_LENGTH_METERS = Units.inchesToMeters(12.910);
            // TODO: find the moi
            public static final double SIM_MOI = 0.22552744227754662;

            public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(60.0);
            public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(8.0);

            // TODO: tune pid
            public static final double KP = 0.1;
            public static final double KI = 0.0;
            public static final double KD = 0.0;
            public static final double KS = 0.0;
            public static final double KG = 0.1;
            public static final double KV = 0.1;

            public static final double CRUISE_VELOCITY_RPS = 4.0;
            public static final double MAX_ACCEL_RPS2 = 40.0;
            public static final double JERK_RPS3 = 2000;

            public static final Rotation2d ERROR_TOLERANCE = Rotation2d.fromDegrees(0.2);
        }
    }

    public static class Elevator {
        public static final double CHAIN_HEIGHT_METERS = Units.inchesToMeters(28.25);
        public static final double ELEVATOR_MAX_HEIGHT = Units.inchesToMeters(48.0);
        public static final double ELEVATOR_MINIMUM_HEIGHT = Units.inchesToMeters(27.0);
        public static final double CLIMBING_HOOKS_MINIMUM_HEIGHT = Units.inchesToMeters(12.0);
        public static final double CLIMBING_HOOKS_MAX_HEIGHT = CLIMBING_HOOKS_MINIMUM_HEIGHT + ELEVATOR_MAX_HEIGHT - ELEVATOR_MINIMUM_HEIGHT;

        public enum ElevatorState {
            /**
             * At max height :D
             */
            TOP(ELEVATOR_MAX_HEIGHT),
            /**
             * At minimum height :D
             */
            BOTTOM(ELEVATOR_MINIMUM_HEIGHT),
            /**
             * At height for top of initial climb :D
             */
            CLIMB(254.0), //TODO: Find these values :D
            /**
             * At height for trap doing :D
             */
            TRAP(254.0), //TODO: Find these values :D
            /**
             * At height for top of initial climb :D
             */
            AMP(254.0); //TODO: Find these values :D

            public final double heightMeters;
            ElevatorState(double heightMeters) {
                this.heightMeters = heightMeters;
            }
        }
    }

    public static class Tramp {
        public static final double GEAR_RATIO = 3.0 / 1.0;
        public static final double TRAP_SCORING_TIME_SECONDS = 1.114; //TODO: Find these values :D
        public static final int TRAMP_MOTOR_ID = -1; //TODO: Configure this later
        public static final int TRAMP_BEAM_BREAK_CHANNEL = -1; //TODO: Configure this later
    }
}
