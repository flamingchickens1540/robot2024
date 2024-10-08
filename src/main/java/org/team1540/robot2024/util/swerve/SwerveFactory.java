package org.team1540.robot2024.util.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import static org.team1540.robot2024.Constants.Drivetrain.TURN_GEAR_RATIO;
import static org.team1540.robot2024.Constants.SwerveConfig.CAN_BUS;


public class SwerveFactory {
    private static final double[] moduleOffsetsRots = new double[]{
//            -0.7834472656 + 0.5,      // Module 1
//            0.6655273438,   // Module 2
//            -0.7197265,      // Module 3
//            -0.66479 - 0.25-0.5,         // Module 4
//            -0.41162109375,  // Module 5
//            -0.594970703125, // Module 6
//            -0.826660,       // Module 7
//            0.0,             // Module 8
//            0.2563476562    // Module 9
    };

    public static SwerveModuleHW getModuleMotors(int id, SwerveCorner corner) {
        return new SwerveModuleHW(id, corner, CAN_BUS);
    }

    public enum SwerveCorner {
        FRONT_LEFT(0),
        FRONT_RIGHT(0.25),
        BACK_LEFT(0.75),
        BACK_RIGHT(0.5);

        private final double offsetRots;

        SwerveCorner(double offsetRots) {
            this.offsetRots = offsetRots;
        }
    }

    public static class SwerveModuleHW {
        public final TalonFX driveMotor;
        public final TalonFX turnMotor;
        public final CANcoder cancoder;
        private SwerveModuleHW(int id, SwerveCorner corner, String canbus) {
            if (id < 1 || id > 9) {
                throw new IllegalArgumentException("Swerve module id must be between 1 and 9");
            }
            if (canbus == null) {
                canbus = "";
            }

            boolean invertTurn = true;
            boolean invertDrive = true;
            switch (corner) {
                case FRONT_LEFT:
                    invertTurn = false;
                    invertDrive = false;
                    break;
                case FRONT_RIGHT:
                    invertTurn = true;
                    invertDrive = false;
                    break;
                case BACK_LEFT:
                    invertTurn = true;
                    invertDrive = false;
                    break;
                case BACK_RIGHT:
                    invertTurn = false;
                    invertDrive = true;
                    break;
            }

            int driveID = 30 + id;
            int turnID = 20 + id;
            int canCoderID = 10 + id;
            this.driveMotor = new TalonFX(driveID, canbus);
            this.cancoder = new CANcoder(canCoderID, canbus);
            this.turnMotor = new TalonFX(turnID, canbus);

            TalonFXConfiguration driveConfig = new TalonFXConfiguration();
            TalonFXConfiguration turnConfig = new TalonFXConfiguration();
            CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
            cancoder.getConfigurator().refresh(canCoderConfig);

            driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
            driveConfig.CurrentLimits.SupplyCurrentThreshold = 50.0;
            driveConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
            driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            driveConfig.MotorOutput.Inverted = invertDrive ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive;

            turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
            turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            turnConfig.MotorOutput.Inverted = invertTurn ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            turnConfig.Feedback.FeedbackRemoteSensorID = canCoderID;
            turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            turnConfig.Feedback.SensorToMechanismRatio = 1.0;
            turnConfig.Feedback.RotorToSensorRatio = TURN_GEAR_RATIO;

//            canCoderConfig.MagnetSensor.MagnetOffset = moduleOffsetsRots[id - 1] + corner.offsetRots;
            canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

            this.driveMotor.getConfigurator().apply(driveConfig);
            this.cancoder.getConfigurator().apply(canCoderConfig);
            this.turnMotor.getConfigurator().apply(turnConfig);
        }
    }
}
