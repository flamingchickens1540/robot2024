package org.team1540.robot2024.subsystems.tramp;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import org.team1540.robot2024.Constants.Tramp;

public class TrampIOSparkMax implements TrampIO {
    private final DigitalInput beamBreak = new DigitalInput(4);
    //TODO: Potentially change name :D
    private final CANSparkMax motor = new CANSparkMax(Tramp.TRAMP_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder motorEncoder = motor.getEncoder();

    public TrampIOSparkMax() {
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        //Potentially invert motor
        motor.setSmartCurrentLimit(40);
        motor.enableVoltageCompensation(12);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    public void updateInputs(TrampIOInputs inputs) {
        inputs.breamBreakTripped = !(beamBreak.get()); //I think returns false when broken... Returns true when broken now.
        inputs.velocityRPM = motorEncoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }
}
