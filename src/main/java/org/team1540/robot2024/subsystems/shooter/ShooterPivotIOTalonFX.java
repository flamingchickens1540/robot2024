package org.team1540.robot2024.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.geometry.Rotation2d;


import static org.team1540.robot2024.Constants.Shooter.Pivot.*;

public class ShooterPivotIOTalonFX implements ShooterPivotIO {
    private final TalonFX motor = new TalonFX(MOTOR_ID);
    private final CANcoder cancoder = new CANcoder(CANCODER_ID);

    private final StatusSignal<Double> position = motor.getPosition();
    private final StatusSignal<Double> absolutePosition = cancoder.getAbsolutePosition();
    private final StatusSignal<Double> velocity = motor.getVelocity();
    private final StatusSignal<Double> appliedVoltage = motor.getMotorVoltage();
    private final StatusSignal<Double> current = motor.getSupplyCurrent();
    private final StatusSignal<Double> temp = motor.getDeviceTemp();
    private final StatusSignal<ForwardLimitValue> forwardLimit = motor.getForwardLimit();
    private final StatusSignal<ReverseLimitValue> reverseLimit = motor.getReverseLimit();

    private final MotionMagicVoltage positionCtrlReq = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageCtrlReq = new VoltageOut(0);

    public ShooterPivotIOTalonFX() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        // TODO: find invert
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
//        motorConfig.Feedback.FeedbackRemoteSensorID = CANCODER_ID;
        motorConfig.Feedback.SensorToMechanismRatio = CANCODER_TO_PIVOT*MOTOR_TO_CANCODER;
        motorConfig.Feedback.RotorToSensorRatio = 1;


        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.getRotations();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.getRotations();

        motorConfig.Slot0.kP = KP;
        motorConfig.Slot0.kI = KI;
        motorConfig.Slot0.kD = KD;
        motorConfig.Slot0.kS = KS;
        motorConfig.Slot0.kG = KG;
        motorConfig.Slot0.kV = KV;
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY_RPS;
        motorConfig.MotionMagic.MotionMagicAcceleration = MAX_ACCEL_RPS2;
        motorConfig.MotionMagic.MotionMagicJerk = JERK_RPS3;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 10;
        motorConfig.CurrentLimits.SupplyCurrentThreshold = 0.1;
        motorConfig.CurrentLimits.SupplyTimeThreshold = 15;

        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
//        cancoderConfig.MagnetSensor.MagnetOffset = CANCODER_OFFSET_ROTS;
        // TODO: find invert
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        cancoder.getConfigurator().apply(cancoderConfig);
        motor.getConfigurator().apply(motorConfig);
//        motor.setPosition(absolutePosition.getValue()*MOTOR_TO_CANCODER);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                position,
                absolutePosition,
                velocity,
                appliedVoltage,
                current,
                temp,
                forwardLimit,
                reverseLimit);

        motor.optimizeBusUtilization();
        cancoder.optimizeBusUtilization();
        motor.setPosition(
                Rotation2d.fromRotations(absolutePosition.getValueAsDouble() / CANCODER_TO_PIVOT * CHAIN_FACTOR).plus(ENCODER_OFFSET).getRotations()
        );
    }

    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {
        BaseStatusSignal.refreshAll(position, absolutePosition, velocity, appliedVoltage, current, temp, forwardLimit, reverseLimit);
        inputs.isAtForwardLimit = forwardLimit.getValue() == ForwardLimitValue.ClosedToGround;
        inputs.isAtReverseLimit = reverseLimit.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.position = Rotation2d.fromRotations(position.getValueAsDouble());
        inputs.absolutePosition = Rotation2d.fromRotations(absolutePosition.getValueAsDouble() / CANCODER_TO_PIVOT * CHAIN_FACTOR).plus(ENCODER_OFFSET);
        inputs.velocityRPS = velocity.getValueAsDouble();
        inputs.appliedVolts = appliedVoltage.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }

    @Override
    public void setPosition(Rotation2d position) {
        motor.setControl(positionCtrlReq.withPosition(position.getRotations()));
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageCtrlReq.withOutput(volts));
    }

    @Override
    public void setBrakeMode(boolean isBrakeMode) {
        motor.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void configPID(double kP, double kI, double kD, double kG) {
        Slot0Configs pidConfigs = new Slot0Configs();
        motor.getConfigurator().refresh(pidConfigs);
        pidConfigs.kP = kP;
        pidConfigs.kI = kI;
        pidConfigs.kD = kD;
        pidConfigs.kG = kG;
        motor.getConfigurator().apply(pidConfigs);
    }
    @Override
    public void setEncoderPosition(double rots) {
        motor.setPosition(rots);
    }
}
