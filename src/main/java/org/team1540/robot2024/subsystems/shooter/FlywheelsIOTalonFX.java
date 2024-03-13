package org.team1540.robot2024.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static org.team1540.robot2024.Constants.Shooter.Flywheels.*;

public class FlywheelsIOTalonFX implements FlywheelsIO {
    private final TalonFX leftMotor = new TalonFX(LEFT_ID);
    private final TalonFX rightMotor = new TalonFX(RIGHT_ID);

    private final StatusSignal<Double> leftVelocity = leftMotor.getVelocity();
    private final StatusSignal<Double> leftAppliedVolts = leftMotor.getMotorVoltage();
    private final StatusSignal<Double> leftCurrent = leftMotor.getSupplyCurrent();

    private final StatusSignal<Double> rightVelocity = rightMotor.getVelocity();
    private final StatusSignal<Double> rightAppliedVolts = rightMotor.getMotorVoltage();
    private final StatusSignal<Double> rightCurrent = rightMotor.getSupplyCurrent();

    private final VelocityVoltage leftVelocityCtrlReq =
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VoltageOut leftVoltageCtrlReq =
            new VoltageOut(0).withEnableFOC(true);

    private final VelocityVoltage rightVelocityCtrlReq =
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VoltageOut rightVoltageCtrlReq =
            new VoltageOut(0).withEnableFOC(true);

    public FlywheelsIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Shooter current limits are banned
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.Feedback.RotorToSensorRatio = 1.0;

        config.Slot0.kP = KP;
        config.Slot0.kI = KI;
        config.Slot0.kD = KD;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;

        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);
        leftMotor.setInverted(true);
        rightMotor.setInverted(false);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                leftVelocity,
                leftAppliedVolts,
                leftCurrent,
                rightVelocity,
                rightAppliedVolts,
                rightCurrent);

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftVelocity,
                leftAppliedVolts,
                leftCurrent,
                rightVelocity,
                rightAppliedVolts,
                rightCurrent);

        inputs.leftVelocityRPM = leftVelocity.getValueAsDouble() * 60;
        inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
        inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();

        inputs.rightVelocityRPM = rightVelocity.getValueAsDouble() * 60;
        inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
        inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
    }

    @Override
    public void setSpeeds(double leftRPM, double rightRPM) {
        leftMotor.setControl(leftVelocityCtrlReq.withVelocity(leftRPM / 60));
        rightMotor.setControl(rightVelocityCtrlReq.withVelocity(rightRPM / 60));
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        leftMotor.setControl(leftVoltageCtrlReq.withOutput(leftVolts));
        rightMotor.setControl(rightVoltageCtrlReq.withOutput(rightVolts));
    }

    @Override
    public void configPID(double kP, double kI, double kD) {
        Slot0Configs pidConfigs = new Slot0Configs();
        leftMotor.getConfigurator().refresh(pidConfigs);
        pidConfigs.kP = kP;
        pidConfigs.kI = kI;
        pidConfigs.kD = kD;
        leftMotor.getConfigurator().apply(pidConfigs);
        rightMotor.getConfigurator().apply(pidConfigs);
    }
}
