package org.team1540.robot2024.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.Servo;
import org.team1540.robot2024.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leader = new TalonFX(Constants.Elevator.LEADER_ID);
    private final TalonFX follower = new TalonFX(Constants.Elevator.FOLLOWER_ID);

    private final MotionMagicVoltage motionMagicOut = new MotionMagicVoltage(0).withEnableFOC(true);
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
    private final Follower followerReq = new Follower(leader.getDeviceID(), false);

    private final Servo leftFlipper = new Servo(Constants.Elevator.LEFT_FLIPPER_ID);
    private final Servo rightFlipper = new Servo(Constants.Elevator.RIGHT_FLIPPER_ID);

    private final StatusSignal<Double> leaderPosition = leader.getPosition();
    private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
    private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
    private final StatusSignal<Double> followerAppliedVolts = follower.getMotorVoltage();
    private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();
    private final StatusSignal<Double> followerCurrent = follower.getStatorCurrent();
    private final StatusSignal<Double> leaderTemp = leader.getDeviceTemp();
    private final StatusSignal<Double> followerTemp = follower.getDeviceTemp();
    private final StatusSignal<ForwardLimitValue> topLimitStatus = leader.getForwardLimit();
    private final StatusSignal<ReverseLimitValue> bottomLimitStatus = leader.getReverseLimit();
    private final TalonFXConfiguration config;


    public ElevatorIOTalonFX() {
        config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentThreshold = 60.0;
        config.CurrentLimits.SupplyTimeThreshold = 0.1;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = Constants.Elevator.MOTOR_ROTS_PER_METER;

        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
                leaderPosition,
                leaderVelocity,
                leaderAppliedVolts,
                followerAppliedVolts,
                leaderCurrent,
                followerCurrent,
                leaderTemp,
                followerTemp,
                topLimitStatus,
                bottomLimitStatus);
        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();

        // setting slot 0 gains
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kS = Constants.Elevator.KS;
        slot0Configs.kV = Constants.Elevator.KV;
        slot0Configs.kA = Constants.Elevator.KA;
        slot0Configs.kP = Constants.Elevator.KP;
        slot0Configs.kI = Constants.Elevator.KI;
        slot0Configs.kD = Constants.Elevator.KD;
        slot0Configs.kG = Constants.Elevator.KG;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0 = slot0Configs;

        // setting Motion Magic Settings
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Elevator.CRUISE_VELOCITY_MPS;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Elevator.MAXIMUM_ACCELERATION_MPS2;
        motionMagicConfigs.MotionMagicJerk = Constants.Elevator.JERK_MPS3;
        config.MotionMagic = motionMagicConfigs;

        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
        leader.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);
        follower.setControl(followerReq);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leaderPosition,
                leaderVelocity,
                leaderAppliedVolts,
                followerAppliedVolts,
                leaderCurrent,
                followerCurrent,
                leaderTemp,
                followerTemp,
                topLimitStatus,
                bottomLimitStatus);

        inputs.positionMeters = leaderPosition.getValueAsDouble();
        inputs.velocityMPS = leaderVelocity.getValueAsDouble();
        inputs.voltage = new double[]{leaderAppliedVolts.getValueAsDouble(), followerAppliedVolts.getValueAsDouble()};
        inputs.currentAmps = new double[]{leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
        inputs.tempCelsius = new double[]{leaderTemp.getValueAsDouble(), followerTemp.getValueAsDouble()};
        inputs.atUpperLimit = topLimitStatus.getValue() == ForwardLimitValue.ClosedToGround;
        inputs.atLowerLimit = bottomLimitStatus.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.flipperAngleDegrees = leftFlipper.getAngle();
    }

    @Override
    public void setSetpointMeters(double positionMeters) {
        leader.setControl(motionMagicOut.withPosition(positionMeters));
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        leader.set(dutyCycle);
    }

    @Override
    public void setBrakeMode(boolean isBrakeMode) {
        leader.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        follower.setNeutralMode(isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        follower.setControl(followerReq);
    }

    @Override
    public void setFlipper(boolean flipped) {
        leftFlipper.set(flipped ? 0.63 : 1);
        rightFlipper.set(flipped ? 1 : 0);
    }

    @Override
    public void configPID(double kP, double kI, double kD) {
        Slot0Configs configs = new Slot0Configs();
        leader.getConfigurator().refresh(configs);
        configs.kP = kP;
        configs.kI = kI;
        configs.kD = kD;
        leader.getConfigurator().apply(configs);
        follower.setControl(followerReq);
    }
}
