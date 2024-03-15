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
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

import static org.team1540.robot2024.Constants.Indexer.*;

public class IndexerIOTalonFXSparkMax implements IndexerIO {
    private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final TalonFX feederMotor = new TalonFX(FEEDER_ID);
    private final DigitalInput indexerBeamBreak = new DigitalInput(7);

    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    private final VoltageOut voltageCtrlReq = new VoltageOut(0).withEnableFOC(true);
    private final VelocityVoltage velocityCtrlReq = new VelocityVoltage(0).withEnableFOC(true);
    private final StatusSignal<Double> feederVoltage = feederMotor.getMotorVoltage();
    private final StatusSignal<Double> feederCurrent = feederMotor.getSupplyCurrent();
    private final StatusSignal<Double> feederVelocity = feederMotor.getVelocity();

    public IndexerIOTalonFXSparkMax() {
        intakeMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        intakeMotor.enableVoltageCompensation(12.0);
        intakeMotor.setSmartCurrentLimit(55);

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

        feederMotor.getConfigurator().apply(feederConfig);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(feederVoltage, feederCurrent, feederVelocity);
        inputs.intakeVoltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.intakeVelocityRPM = intakeEncoder.getVelocity();
        inputs.feederVoltage = feederVoltage.getValueAsDouble();
        inputs.feederCurrentAmps = feederCurrent.getValueAsDouble();
        inputs.feederVelocityRPM = feederVelocity.getValueAsDouble();
        inputs.feederVelocityError = inputs.feederVelocityRPM - velocityCtrlReq.Velocity;
        inputs.noteInIntake = !indexerBeamBreak.get();
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }

    @Override
    public void setFeederVoltage(double voltage) {
        feederMotor.setControl(voltageCtrlReq.withOutput(voltage));
    }

    @Override
    public void setFeederVelocity(double velocity) {
        feederMotor.setControl(velocityCtrlReq.withVelocity(velocity));
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
