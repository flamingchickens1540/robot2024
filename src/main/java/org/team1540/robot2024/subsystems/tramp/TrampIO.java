package org.team1540.robot2024.subsystems.tramp;

import org.littletonrobotics.junction.AutoLog;


public interface TrampIO {
    @AutoLog
    class TrampIOInputs {
        public boolean breamBreakTripped = false;
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }
    void setVoltage(double volts);
    void updateInputs(TrampIOInputs inputs);
}
