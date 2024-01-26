package org.team1540.robot2024.subsystems.indexer;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import static org.team1540.robot2024.Constants.Indexer.*;

public class IndexerIOReal implements IndexerIO {
    CANSparkMax intakeMotor = new CANSparkMax(INTAKE_ID, CANSparkLowLevel.MotorType.kBrushless);


    public IndexerIOReal() {
        intakeMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        intakeMotor.enableVoltageCompensation(12.0);
        intakeMotor.setSmartCurrentLimit(30);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.intakeCurrent = intakeMotor.getOutputCurrent();
        inputs.intakeVoltage = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }
}
