package org.team1540.robot2024.subsystems.indexer;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;

import static org.team1540.robot2024.Constants.Indexer.*;


public class IndexerIOSparkMax implements IndexerIO {
    private final CANSparkMax intakeMotor = new CANSparkMax(INTAKE_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax feederMotor = new CANSparkMax(FEEDER_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final DigitalInput indexerBeamBreak = new DigitalInput(7);

    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private final RelativeEncoder feederEncoder = feederMotor.getEncoder();

    private final SparkPIDController feederPID = feederMotor.getPIDController();


    public IndexerIOSparkMax() {
        intakeMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        intakeMotor.enableVoltageCompensation(12.0);
        intakeMotor.setSmartCurrentLimit(55);

        feederMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
        feederMotor.setInverted(true);
        feederMotor.enableVoltageCompensation(12.0);
        feederMotor.setSmartCurrentLimit(60);

        feederPID.setP(FEEDER_KP, 0);
        feederPID.setI(FEEDER_KI, 0);
        feederPID.setD(FEEDER_KD, 0);
        feederPID.setFF(FEEDER_KV, 0);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.intakeCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.intakeVoltage = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
        inputs.intakeVelocityRPM = intakeEncoder.getVelocity();
        inputs.intakeTempCelsius = intakeMotor.getMotorTemperature();
        inputs.feederCurrentAmps = feederMotor.getOutputCurrent();
        inputs.feederVoltage = feederMotor.getBusVoltage() * feederMotor.getAppliedOutput();
        inputs.feederVelocityRPM = feederEncoder.getVelocity();
        inputs.feederTempCelsius = feederMotor.getMotorTemperature();
        inputs.noteInIntake = !indexerBeamBreak.get();
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }

    @Override
    public void setFeederVoltage(double volts) {
        feederMotor.setVoltage(volts);
    }

    @Override
    public void setFeederVelocity(double velocityRPM) {
        feederPID.setReference(
                velocityRPM * FEEDER_GEAR_RATIO,
                CANSparkBase.ControlType.kVelocity,
                0,
                FEEDER_KS,
                SparkPIDController.ArbFFUnits.kVoltage
        );
    }

    @Override
    public void configureFeederPID(double p, double i, double d) {
        feederPID.setP(p);
        feederPID.setI(i);
        feederPID.setD(d);
    }

    public void setIntakeBrakeMode(boolean isBrakeMode) {
        intakeMotor.setIdleMode(isBrakeMode ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
    }
}

