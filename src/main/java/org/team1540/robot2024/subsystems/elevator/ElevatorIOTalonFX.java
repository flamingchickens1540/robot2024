package org.team1540.robot2024.subsystems.elevator;

import org.team1540.robot2024.Constants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final MotionMagicVoltage motionMagicOut = new MotionMagicVoltage(0).withEnableFOC(true);
    private final TalonFX leader = new TalonFX(Constants.Elevator.TALON_ID_1);
    private final TalonFX follower = new TalonFX(Constants.Elevator.TALON_ID_2);

    private final StatusSignal<Double> leaderPosition = leader.getPosition();
    private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
    private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
    private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();
    private final StatusSignal<Double> followerCurrent = follower.getStatorCurrent();
    private final StatusSignal<ForwardLimitValue> topLimitStatus = leader.getForwardLimit();
    private final StatusSignal<ReverseLimitValue> bottomLimitStatus = leader.getReverseLimit();


    public ElevatorIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentThreshold = 60.0;
        config.CurrentLimits.SupplyTimeThreshold = 0.1;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Feedback.SensorToMechanismRatio = Constants.Elevator.ELEVATOR_GEAR_RATIO;
        // TODO: this might not actually be inverted, so fix it if neccesary
        follower.setControl(new Follower(leader.getDeviceID(), true));

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent, topLimitStatus, bottomLimitStatus);
        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();

        // setting slot 0 gains
        Slot0Configs slot0Configs = config.Slot0;
        slot0Configs.kS = Constants.Elevator.KS;
        slot0Configs.kV = Constants.Elevator.KV;
        slot0Configs.kA = Constants.Elevator.KA;
        slot0Configs.kP = Constants.Elevator.KP;
        slot0Configs.kI = Constants.Elevator.KI;
        slot0Configs.kD = Constants.Elevator.KD;
        slot0Configs.kG = Constants.Elevator.KG;

        // setting Motion Magic Settings
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Elevator.MOTION_MAGIC_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Elevator.MOTION_MAGIC_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = Constants.Elevator.MOTION_MAGIC_JERK;

        config.HardwareLimitSwitch.ForwardLimitEnable = true;
        config.HardwareLimitSwitch.ReverseLimitEnable = true;
        leader.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent, topLimitStatus, bottomLimitStatus);

        inputs.positionMeters = leaderPosition.getValue();
        inputs.velocityRPM = leaderVelocity.getValue();
        inputs.voltage = leaderAppliedVolts.getValue();
        inputs.current = new double[] {leaderCurrent.getValue(), followerCurrent.getValue()};
        inputs.upperLimit = topLimitStatus.getValue() == ForwardLimitValue.Open;
        inputs.lowerLimit = bottomLimitStatus.getValue() == ReverseLimitValue.Open;
    }

    @Override
    public void setSetpoint(double position) {
        leader.setControl(motionMagicOut.withPosition(position));
    }

    @Override
    public void stop() {
        leader.set(0);
    }
}
