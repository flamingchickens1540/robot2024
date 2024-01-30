package org.team1540.robot2024.subsystems.tramp;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import org.team1540.robot2024.Constants.Tramp;

public class TrampIOSparkMax implements TrampIO {
    private final DigitalInput beamBreak = new DigitalInput(0);
    //TODO: Potentially change name :D
    private final CANSparkMax neor = new CANSparkMax(Tramp.TRAMP_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder neorEncoder = neor.getEncoder();
    public TrampIOSparkMax() {
        neor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        //Potentially invert motor
        neor.setSmartCurrentLimit(40);
        neor.enableVoltageCompensation(12);
    }
    @Override
    public void setVoltage(double volts) {
        neor.setVoltage(volts);
    }

    public void updateInputs(TrampIOInputs inputs) {
        inputs.breamBreakTripped = !(beamBreak.get()); //I think returns false when broken...
        inputs.velocityRPM = neorEncoder.getVelocity();
        inputs.appliedVolts = neor.getAppliedOutput() * neor.getBusVoltage();
        inputs.currentAmps = neor.getOutputCurrent();
    }
}
