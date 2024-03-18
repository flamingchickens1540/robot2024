package org.team1540.robot2024.subsystems.tramp;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

import static org.team1540.robot2024.Constants.Shooter.Pivot.CANCODER_TO_PIVOT;
import static org.team1540.robot2024.Constants.Shooter.Pivot.MOTOR_TO_CANCODER;
import static org.team1540.robot2024.Constants.Tramp.BEAM_BREAK_CHANNEL;
import static org.team1540.robot2024.Constants.Tramp.MOTOR_ID;

public class TrampIOTalonFX implements TrampIO {
    private final DigitalInput beamBreak = new DigitalInput(BEAM_BREAK_CHANNEL);
    private final TalonFX motor = new TalonFX(MOTOR_ID);
    private final StatusSignal<Double> velocity = motor.getVelocity();
    private final StatusSignal<Double> position = motor.getPosition();
    private final StatusSignal<Double> appliedVoltage = motor.getMotorVoltage();
    private final StatusSignal<Double> current = motor.getSupplyCurrent();

    public TrampIOTalonFX() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
        motorConfig.CurrentLimits.SupplyCurrentThreshold = 80;
        motorConfig.CurrentLimits.SupplyTimeThreshold = 0.2;

        motor.getConfigurator().apply(motorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(50, position, velocity, appliedVoltage, current);
        motor.optimizeBusUtilization();
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(TrampIOInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity, appliedVoltage, current);
        inputs.noteInTramp = !(beamBreak.get());
        inputs.velocityRPM = velocity.getValueAsDouble();
        inputs.positionRots = position.getValueAsDouble();
        inputs.appliedVolts = appliedVoltage.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
    }
}
