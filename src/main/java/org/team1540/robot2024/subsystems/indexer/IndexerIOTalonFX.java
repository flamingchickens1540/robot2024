package org.team1540.robot2024.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

import static org.team1540.robot2024.Constants.Indexer.*;

public class IndexerIOTalonFX implements IndexerIO {
    private final TalonFX intakeMotor = new TalonFX(INTAKE_ID);
    private final TalonFX feederMotor = new TalonFX(FEEDER_ID);
    private final DigitalInput indexerBeamBreak = new DigitalInput(BEAM_BREAK_ID);

    private final VoltageOut feederVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);
    private final VelocityVoltage feederVelocityCtrlReq = new VelocityVoltage(0).withEnableFOC(true);
    private final StatusSignal<Double> feederVoltage = feederMotor.getMotorVoltage();
    private final StatusSignal<Double> feederCurrent = feederMotor.getSupplyCurrent();
    private final StatusSignal<Double> feederVelocity = feederMotor.getVelocity();
    private final StatusSignal<Double> feederTemp = feederMotor.getDeviceTemp();

    private final VoltageOut intakeVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);
    private final StatusSignal<Double> intakeVoltage = intakeMotor.getMotorVoltage();
    private final StatusSignal<Double> intakeCurrent = intakeMotor.getSupplyCurrent();
    private final StatusSignal<Double> intakeVelocity = intakeMotor.getVelocity();
    private final StatusSignal<Double> intakeTemp = intakeMotor.getDeviceTemp();

    public IndexerIOTalonFX() {
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 55;
        intakeConfig.CurrentLimits.SupplyCurrentThreshold = 80;
        intakeConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

        TalonFXConfiguration feederConfig  = new TalonFXConfiguration();
        feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        feederConfig.CurrentLimits.SupplyCurrentLimit = 60;
        feederConfig.CurrentLimits.SupplyCurrentThreshold = 80;
        feederConfig.CurrentLimits.SupplyTimeThreshold = 0.1;
        feederConfig.Slot0.kP = FEEDER_KP;
        feederConfig.Slot0.kI = FEEDER_KI;
        feederConfig.Slot0.kD = FEEDER_KD;
        feederConfig.Slot0.kV = FEEDER_KV;

        intakeMotor.getConfigurator().apply(intakeConfig);
        feederMotor.getConfigurator().apply(feederConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(50,
                feederVoltage,
                feederCurrent,
                feederVelocity,
                feederTemp,
                intakeVoltage,
                intakeCurrent,
                intakeVelocity,
                intakeTemp);
        intakeMotor.optimizeBusUtilization();
        feederMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                intakeVoltage,
                intakeCurrent,
                intakeVelocity,
                intakeTemp,
                feederVoltage,
                feederCurrent,
                feederVelocity,
                intakeTemp);
        inputs.intakeVoltage = intakeVoltage.getValueAsDouble();
        inputs.intakeCurrentAmps = intakeCurrent.getValueAsDouble();
        inputs.intakeVelocityRPM = intakeVelocity.getValueAsDouble();
        inputs.intakeTempCelsius = intakeTemp.getValueAsDouble();
        inputs.feederVoltage = feederVoltage.getValueAsDouble();
        inputs.feederCurrentAmps = feederCurrent.getValueAsDouble();
        inputs.feederVelocityRPM = feederVelocity.getValueAsDouble();
        inputs.feederTempCelsius = feederTemp.getValueAsDouble();
        inputs.feederVelocityError = inputs.feederVelocityRPM - feederVelocityCtrlReq.Velocity;
        inputs.noteInIntake = !indexerBeamBreak.get();
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotor.setControl(intakeVoltageCtrlReq.withOutput(volts));
    }

    @Override
    public void setFeederVoltage(double voltage) {
        feederMotor.setControl(feederVoltageCtrlReq.withOutput(voltage));
    }

    @Override
    public void setFeederVelocity(double velocity) {
        feederMotor.setControl(feederVelocityCtrlReq.withVelocity(velocity));
    }

    @Override
    public void configureFeederPID(double p, double i, double d) {
        Slot0Configs pidConfigs = new Slot0Configs();
        feederMotor.getConfigurator().refresh(pidConfigs);
        pidConfigs.kP = p;
        pidConfigs.kI = i;
        pidConfigs.kD = d;
        feederMotor.getConfigurator().apply(pidConfigs);
    }
}
