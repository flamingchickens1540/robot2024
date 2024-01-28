package org.team1540.robot2024;

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
    public static class Indexer {
        // TODO: fix these constants
        public static final int INTAKE_ID = 0;
        public static final int FEEDER_ID = 0;
        public static final double FEEDER_KP = 0.0;
        public static final double FEEDER_KI = 0.0;
        public static final double FEEDER_KD = 0.0;
        public static final double FEEDER_KS = 0.0;
        public static final double FEEDER_KV = 0.0;
        public static final double FEEDER_GEAR_RATIO = 1.0;
        public static final double INTAKE_GEAR_RATIO = 1.0;

    }
}
