package org.team1540.robot2024.subsystems.elevator;

import org.team1540.robot2024.Constants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final TalonFX leader = new TalonFX(Constants.Elevator.talonId1);
    private final TalonFX follower = new TalonFX(Constants.Elevator.talonId2);

    private final StatusSignal<Double> leaderPosition = leader.getPosition();
    private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
    private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
    private final StatusSignal<Double> leaderCurrent = leader.getStatorCurrent();
    private final StatusSignal<Double> followerCurrent = follower.getStatorCurrent();

    public ElevatorIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentThreshold = 60.0;
        config.CurrentLimits.SupplyTimeThreshold = 0.1;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leader.getConfigurator().apply(config);
        leader.getConfigurator().apply(config);
        // TODO: this might not actually be inverted, so fix it if neccesary
        follower.setControl(new Follower(leader.getDeviceID(), true));

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
        leader.optimizeBusUtilization();
        follower.optimizeBusUtilization();
    }

}
