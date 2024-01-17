package org.team1540.robot2024.util.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import static org.team1540.robot2024.Constants.SwerveConfig.CAN_BUS;;

public class SwerveFactory {
    private static final double[] moduleOffsetsRots = new double[]{
            -0.9130859, // Module 1
            0.0, // Module 2
            -0.7197265, // Module 3
            -0.7722, // Module 4
            0.0, // Module 5
            0.0, // Module 6
            -0.826660, // Module 7
            0.0  // Module 8
    };

    public static SwerveModuleHW getModuleMotors(int id, SwerveCorner corner) {
        return new SwerveModuleHW(id, corner, CAN_BUS);
    }

    public enum SwerveCorner {
        FRONT_LEFT(0),
        FRONT_RIGHT(90),
        BACK_LEFT(270),
        BACK_RIGHT(180);

        private final double offset;
        SwerveCorner(double offset) {
            this.offset = offset;
        }
    }

    public static class SwerveModuleHW {
        public final TalonFX driveMotor;
        public final TalonFX turnMotor;
        public final CANcoder cancoder;
        public final Rotation2d cancoderOffset;

        private SwerveModuleHW(int id, SwerveCorner corner, String canbus) {
            if (id < 1 || id > 8) {
                throw new IllegalArgumentException("Swerve module id must be between 1 and 8");
            }
            if (canbus == null) {
                canbus = "";
            }
            TalonFXConfiguration driveConfig = new TalonFXConfiguration();
            TalonFXConfiguration turnConfig = new TalonFXConfiguration();
            CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

            driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
            driveConfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
            driveConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
            driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
            turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            canCoderConfig.MagnetSensor.MagnetOffset = moduleOffsetsRots[id-1];
            canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

            this.driveMotor = new TalonFX(30 + id, canbus);
            this.driveMotor.getConfigurator().apply(driveConfig);

            this.turnMotor = new TalonFX(20 + id, canbus);

            this.turnMotor.getConfigurator().apply(turnConfig);

            this.cancoder = new CANcoder(10 + id, canbus);


            this.cancoder.getConfigurator().apply(canCoderConfig);

            this.cancoderOffset = Rotation2d.fromDegrees(corner.offset);


        }
    }
}
